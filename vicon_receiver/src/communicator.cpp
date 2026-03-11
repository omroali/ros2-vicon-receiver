#include "vicon_receiver/communicator.hpp"

using namespace ViconDataStreamSDK::CPP;

// Constructor for the Communicator class
Communicator::Communicator() : Node("vicon_client")
{
    // Declare parameters for hostname, buffer size, and namespace
    this->declare_parameter<std::string>("hostname", "127.0.0.1");
    this->declare_parameter<int>("buffer_size", 200);
    this->declare_parameter<std::string>("namespace", "vicon");
    this->declare_parameter<std::string>("world_frame", "map");
    this->declare_parameter<std::string>("vicon_frame", "vicon");
    this->declare_parameter<std::vector<double>>("map_xyz",  {0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("map_rpy",  {0.0, 0.0, 0.0});
    this->declare_parameter<bool>("map_rpy_in_degrees", false);
    
    // Declare parameters for publish modes
    this->declare_parameter<bool>("publish_segments", true);
    this->declare_parameter<bool>("publish_markers", false);
    this->declare_parameter<bool>("publish_unlabeled_markers", false);
    
    // Declare parameter for marker visualization size (in meters)
    this->declare_parameter<double>("marker_size", 0.02);

    // Retrieve parameters values
    this->get_parameter("hostname", hostname);
    this->get_parameter("buffer_size", buffer_size);
    this->get_parameter("namespace", ns_name);

    this->get_parameter("world_frame", world_frame);
    this->get_parameter("vicon_frame", vicon_frame);
    this->get_parameter("map_xyz", map_xyz);
    this->get_parameter("map_rpy", map_rpy);
    this->get_parameter("map_rpy_in_degrees", map_rpy_in_degrees);
    
    // Retrieve publish mode parameters
    this->get_parameter("publish_segments", publish_segments);
    this->get_parameter("publish_markers", publish_markers);
    this->get_parameter("publish_unlabeled_markers", publish_unlabeled_markers);
    
    // Retrieve marker visualization size
    this->get_parameter("marker_size", marker_size);

    // Publish static transform from map to vicon origin
    if (map_rpy_in_degrees) {
        for (unsigned int i=0; i<map_rpy.size(); i++) {
            map_rpy[i] = map_rpy[i] * M_PI / 180.0;
        }
    }
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->publish_static_transform();

    // Initialize the tf2 broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    // Create MarkerArray publishers for RViz2 visualization
    if (publish_markers) {
        labeled_markers_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            ns_name + "/markers_visualization", 10);
        cout << "Created MarkerArray publisher for labeled markers visualization" << endl;
    }
    
    if (publish_unlabeled_markers) {
        unlabeled_markers_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            ns_name + "/unlabeled_markers_visualization", 10);
        cout << "Created MarkerArray publisher for unlabeled markers visualization" << endl;
    }
    
    // Log publish modes
    cout << "Publish modes - Segments: " << (publish_segments ? "ON" : "OFF")
         << ", Markers: " << (publish_markers ? "ON" : "OFF")
         << ", Unlabeled Markers: " << (publish_unlabeled_markers ? "ON" : "OFF") << endl;
    cout << "Marker visualization size: " << marker_size << " meters" << endl;
}

// Publish the static transform from map to vicon origin
void Communicator::publish_static_transform()
{
    static_tf.header.stamp = this->get_clock()->now();
    static_tf.header.frame_id = world_frame;
    static_tf.child_frame_id = vicon_frame;

    static_tf.transform.translation.x = map_xyz[0];
    static_tf.transform.translation.y = map_xyz[1];
    static_tf.transform.translation.z = map_xyz[2];
    tf2::Quaternion q;
    q.setRPY(
        map_rpy[0],
        map_rpy[1],
        map_rpy[2]
    );
    static_tf.transform.rotation.x = q.x();
    static_tf.transform.rotation.y = q.y();
    static_tf.transform.rotation.z = q.z();
    static_tf.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(static_tf);

    string msg = "Published static transform from " + world_frame + " to " + vicon_frame;
    cout << msg << endl;
}


// Connect to the Vicon server
bool Communicator::connect()
{
    // Log connection attempt
    string msg = "Connecting to " + hostname + " ...";
    cout << msg << endl;

    int counter = 0;
    // Retry connection until successful
    while (!vicon_client.IsConnected().Connected && rclcpp::ok())
    {
        bool ok = (vicon_client.Connect(hostname).Result == Result::Success);
        if (!ok)
        {
            counter++;
            msg = "Connect failed, reconnecting (" + std::to_string(counter) + ")...";
            cout << msg << endl;
        }
    }
    if (!rclcpp::ok()) {
        std::cout << "Shutdown requested before connection established." << std::endl;
        return false;
    }

    // Log successful connection
    msg = "Connection successfully established with " + hostname;
    cout << msg << endl;

    // Enable various data streams from the Vicon server
    vicon_client.EnableSegmentData();
    vicon_client.EnableMarkerData();
    vicon_client.EnableUnlabeledMarkerData();
    vicon_client.EnableMarkerRayData();
    vicon_client.EnableDeviceData();
    vicon_client.EnableDebugData();

    // Set the stream mode and buffer size
    vicon_client.SetStreamMode(StreamMode::ClientPull);
    vicon_client.SetBufferSize(buffer_size);

    // Log initialization completion
    msg = "Initialization complete";
    cout << msg << endl;

    return true;
}

// Disconnect from the Vicon server
bool Communicator::disconnect()
{
    // If already disconnected, return true
    if (!vicon_client.IsConnected().Connected)
        return true;

    sleep(1); // Wait before disconnecting

    // Disable all data streams
    vicon_client.DisableSegmentData();
    vicon_client.DisableMarkerData();
    vicon_client.DisableUnlabeledMarkerData();
    vicon_client.DisableDeviceData();
    vicon_client.DisableCentroidData();

    // Log disconnection attempt
    string msg = "Disconnecting from " + hostname + "...";
    cout << msg << endl;

    // Disconnect from the server
    vicon_client.Disconnect();

    // Log successful disconnection
    msg = "Successfully disconnected";
    cout << msg << endl;

    // Verify disconnection
    return !vicon_client.IsConnected().Connected;
}

// Retrieve and process a frame of data from the Vicon server
void Communicator::get_frame()
{
    // Request a new frame
    vicon_client.GetFrame();
    Output_GetFrameNumber frame_number = vicon_client.GetFrameNumber();

    // Process data based on publish modes
    if (publish_segments) {
        process_segments();
    }
    
    if (publish_markers) {
        process_markers();
    }
    
    if (publish_unlabeled_markers) {
        process_unlabeled_markers();
    }
}

// Process segment data from Vicon server
void Communicator::process_segments()
{
    // Get the number of subjects in the frame
    unsigned int subject_count = vicon_client.GetSubjectCount().SubjectCount;

    map<string, Publisher>::iterator pub_it;

    // Iterate through each subject
    for (unsigned int subject_index = 0; subject_index < subject_count; ++subject_index)
    {
        // Get the subject name
        string subject_name = vicon_client.GetSubjectName(subject_index).SubjectName;

        // Get the number of segments for the subject
        unsigned int segment_count = vicon_client.GetSegmentCount(subject_name).SegmentCount;

        // Iterate through each segment
        for (unsigned int segment_index = 0; segment_index < segment_count; ++segment_index)
        {
            // Get the segment name
            string segment_name = vicon_client.GetSegmentName(subject_name, segment_index).SegmentName;

            // Retrieve the segment's global position and rotation
            Output_GetSegmentGlobalTranslation trans =
                vicon_client.GetSegmentGlobalTranslation(subject_name, segment_name);
            Output_GetSegmentGlobalRotationQuaternion quat =
                vicon_client.GetSegmentGlobalRotationQuaternion(subject_name, segment_name);

            // Build a TF message for this segment
            geometry_msgs::msg::TransformStamped tf_msg;

            // Use node clock to timestamp the transform
            tf_msg.header.stamp = this->get_clock()->now();

            // Parent and child frames: Vicon global origin -> subject_segment
            tf_msg.header.frame_id = vicon_frame;
            tf_msg.child_frame_id = subject_name + "_" + segment_name;

            // Vicon translations are in millimeters; convert to meters for ROS
            tf_msg.transform.translation.x = trans.Translation[0] / 1000.0;
            tf_msg.transform.translation.y = trans.Translation[1] / 1000.0;
            tf_msg.transform.translation.z = trans.Translation[2] / 1000.0;

            // Vicon quaternion order is [x, y, z, w]; copy directly
            tf_msg.transform.rotation.x = quat.Rotation[0];
            tf_msg.transform.rotation.y = quat.Rotation[1];
            tf_msg.transform.rotation.z = quat.Rotation[2];
            tf_msg.transform.rotation.w = quat.Rotation[3];

            // Publish the position data
            boost::mutex::scoped_try_lock lock(segment_mutex);
            if (lock.owns_lock())
            {
                // Check if a publisher exists for the segment
                pub_it = segment_pub_map.find(subject_name + "/" + segment_name);
                if (pub_it != segment_pub_map.end())
                {
                    Publisher & pub = pub_it->second;

                    if (pub.is_ready)
                    {
                        // Build a PoseStamped in the Vicon frame from the computed TransformStamped.
                        geometry_msgs::msg::PoseStamped vicon_pose_msg;

                        // Header: copy timestamp and frame_id ("vicon") from the transform header.
                        vicon_pose_msg.header = tf_msg.header;

                        // Position: copy the already meter-converted translation components.
                        vicon_pose_msg.pose.position.x = tf_msg.transform.translation.x;
                        vicon_pose_msg.pose.position.y = tf_msg.transform.translation.y;
                        vicon_pose_msg.pose.position.z = tf_msg.transform.translation.z;

                        // Orientation: copy the quaternion (x, y, z, w) directly from the transform.
                        vicon_pose_msg.pose.orientation = tf_msg.transform.rotation;

                        // Update timestamp of static transform
                        static_tf.header.stamp = tf_msg.header.stamp;

                        // Transform the pose to the global frame
                        geometry_msgs::msg::PoseStamped global_pose_msg;
                        tf2::doTransform(vicon_pose_msg, global_pose_msg, static_tf);

                        // Publish the transformed pose
                        pub.publish(global_pose_msg);
                    }
                }
                else
                {
                    // Create a publisher if it doesn't exist, de-duplicating concurrent attempts
                    std::string key = subject_name + "/" + segment_name;
                    if (pending_segment_publishers.find(key) == pending_segment_publishers.end())
                    {
                        pending_segment_publishers.insert(key);
                        lock.unlock();
                        create_segment_publisher(subject_name, segment_name);
                    }
                    else
                    {
                        // Another thread is already creating this publisher
                        lock.unlock();
                    }
                }
            }

            // Broadcast the transform
            tf_broadcaster_->sendTransform(tf_msg);
        }
    }
}

// Process labeled marker data from Vicon server
void Communicator::process_markers()
{
    // Get the number of subjects in the frame
    unsigned int subject_count = vicon_client.GetSubjectCount().SubjectCount;

    map<string, PointPublisher>::iterator pub_it;
    
    // Create MarkerArray for visualization
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;
    auto now = this->get_clock()->now();

    // Iterate through each subject
    for (unsigned int subject_index = 0; subject_index < subject_count; ++subject_index)
    {
        // Get the subject name
        string subject_name = vicon_client.GetSubjectName(subject_index).SubjectName;

        // Get the number of markers for the subject
        unsigned int marker_count = vicon_client.GetMarkerCount(subject_name).MarkerCount;

        // Iterate through each marker
        for (unsigned int marker_index = 0; marker_index < marker_count; ++marker_index)
        {
            // Get the marker name
            string marker_name = vicon_client.GetMarkerName(subject_name, marker_index).MarkerName;

            // Retrieve the marker's global position
            Output_GetMarkerGlobalTranslation trans =
                vicon_client.GetMarkerGlobalTranslation(subject_name, marker_name);

            // Skip occluded markers
            if (trans.Occluded)
            {
                continue;
            }

            // Convert position from mm to meters
            double x = trans.Translation[0] / 1000.0;
            double y = trans.Translation[1] / 1000.0;
            double z = trans.Translation[2] / 1000.0;

            // Build a TF message for this marker
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = now;
            tf_msg.header.frame_id = vicon_frame;
            tf_msg.child_frame_id = subject_name + "_marker_" + marker_name;

            tf_msg.transform.translation.x = x;
            tf_msg.transform.translation.y = y;
            tf_msg.transform.translation.z = z;

            // Markers don't have rotation, use identity quaternion
            tf_msg.transform.rotation.x = 0.0;
            tf_msg.transform.rotation.y = 0.0;
            tf_msg.transform.rotation.z = 0.0;
            tf_msg.transform.rotation.w = 1.0;
            
            // Add sphere marker for visualization
            visualization_msgs::msg::Marker sphere_marker;
            sphere_marker.header.frame_id = vicon_frame;
            sphere_marker.header.stamp = now;
            sphere_marker.ns = subject_name;
            sphere_marker.id = marker_id;
            sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
            sphere_marker.action = visualization_msgs::msg::Marker::ADD;
            sphere_marker.pose.position.x = x;
            sphere_marker.pose.position.y = y;
            sphere_marker.pose.position.z = z;
            sphere_marker.pose.orientation.w = 1.0;
            sphere_marker.scale.x = marker_size;
            sphere_marker.scale.y = marker_size;
            sphere_marker.scale.z = marker_size;
            // Green color for labeled markers
            sphere_marker.color.r = 0.0f;
            sphere_marker.color.g = 1.0f;
            sphere_marker.color.b = 0.0f;
            sphere_marker.color.a = 1.0f;
            sphere_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
            marker_array.markers.push_back(sphere_marker);
            
            // Add text marker for the label
            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = vicon_frame;
            text_marker.header.stamp = now;
            text_marker.ns = subject_name + "_labels";
            text_marker.id = marker_id;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose.position.x = x;
            text_marker.pose.position.y = y;
            text_marker.pose.position.z = z + marker_size + 0.01; // Slightly above the sphere
            text_marker.pose.orientation.w = 1.0;
            text_marker.scale.z = marker_size * 1.5; // Text height
            text_marker.color.r = 1.0f;
            text_marker.color.g = 1.0f;
            text_marker.color.b = 1.0f;
            text_marker.color.a = 1.0f;
            text_marker.text = marker_name;
            text_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
            marker_array.markers.push_back(text_marker);
            
            marker_id++;

            // Publish the position data to individual topics
            boost::mutex::scoped_try_lock lock(marker_mutex);
            if (lock.owns_lock())
            {
                // Check if a publisher exists for the marker
                pub_it = marker_pub_map.find(subject_name + "/" + marker_name);
                if (pub_it != marker_pub_map.end())
                {
                    PointPublisher & pub = pub_it->second;

                    if (pub.is_ready)
                    {
                        // Build a PointStamped in the Vicon frame
                        geometry_msgs::msg::PointStamped vicon_point_msg;
                        vicon_point_msg.header.stamp = now;
                        vicon_point_msg.header.frame_id = vicon_frame;

                        vicon_point_msg.point.x = x;
                        vicon_point_msg.point.y = y;
                        vicon_point_msg.point.z = z;

                        // Update timestamp of static transform
                        static_tf.header.stamp = now;

                        // Transform the point to the global frame
                        geometry_msgs::msg::PointStamped global_point_msg;
                        tf2::doTransform(vicon_point_msg, global_point_msg, static_tf);

                        // Publish the transformed point
                        pub.publish(global_point_msg);
                    }
                }
                else
                {
                    // Create a publisher if it doesn't exist, de-duplicating concurrent attempts
                    std::string key = subject_name + "/" + marker_name;
                    if (pending_marker_publishers.find(key) == pending_marker_publishers.end())
                    {
                        pending_marker_publishers.insert(key);
                        lock.unlock();
                        create_marker_publisher(subject_name, marker_name);
                    }
                    else
                    {
                        // Another thread is already creating this publisher
                        lock.unlock();
                    }
                }
            }

            // Broadcast the transform
            tf_broadcaster_->sendTransform(tf_msg);
        }
    }
    
    // Publish the MarkerArray for visualization
    if (!marker_array.markers.empty() && labeled_markers_viz_pub_) {
        labeled_markers_viz_pub_->publish(marker_array);
    }
}

// Process unlabeled marker data from Vicon server
void Communicator::process_unlabeled_markers()
{
    // Get the number of unlabeled markers
    unsigned int unlabeled_marker_count = vicon_client.GetUnlabeledMarkerCount().MarkerCount;

    map<unsigned int, PointPublisher>::iterator pub_it;
    
    // Create MarkerArray for visualization
    visualization_msgs::msg::MarkerArray marker_array;
    auto now = this->get_clock()->now();

    // Iterate through each unlabeled marker
    for (unsigned int marker_index = 0; marker_index < unlabeled_marker_count; ++marker_index)
    {
        // Retrieve the unlabeled marker's global position
        Output_GetUnlabeledMarkerGlobalTranslation trans =
            vicon_client.GetUnlabeledMarkerGlobalTranslation(marker_index);

        // Convert position from mm to meters
        double x = trans.Translation[0] / 1000.0;
        double y = trans.Translation[1] / 1000.0;
        double z = trans.Translation[2] / 1000.0;

        // Build a TF message for this unlabeled marker
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now;
        tf_msg.header.frame_id = vicon_frame;
        tf_msg.child_frame_id = "unlabeled_marker_" + std::to_string(marker_index);

        tf_msg.transform.translation.x = x;
        tf_msg.transform.translation.y = y;
        tf_msg.transform.translation.z = z;

        // Markers don't have rotation, use identity quaternion
        tf_msg.transform.rotation.x = 0.0;
        tf_msg.transform.rotation.y = 0.0;
        tf_msg.transform.rotation.z = 0.0;
        tf_msg.transform.rotation.w = 1.0;
        
        // Add sphere marker for visualization
        visualization_msgs::msg::Marker sphere_marker;
        sphere_marker.header.frame_id = vicon_frame;
        sphere_marker.header.stamp = now;
        sphere_marker.ns = "unlabeled";
        sphere_marker.id = marker_index;
        sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
        sphere_marker.action = visualization_msgs::msg::Marker::ADD;
        sphere_marker.pose.position.x = x;
        sphere_marker.pose.position.y = y;
        sphere_marker.pose.position.z = z;
        sphere_marker.pose.orientation.w = 1.0;
        sphere_marker.scale.x = marker_size;
        sphere_marker.scale.y = marker_size;
        sphere_marker.scale.z = marker_size;
        // Orange color for unlabeled markers
        sphere_marker.color.r = 1.0f;
        sphere_marker.color.g = 0.5f;
        sphere_marker.color.b = 0.0f;
        sphere_marker.color.a = 1.0f;
        sphere_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        marker_array.markers.push_back(sphere_marker);
        
        // Add text marker for the index label
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = vicon_frame;
        text_marker.header.stamp = now;
        text_marker.ns = "unlabeled_labels";
        text_marker.id = marker_index;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position.x = x;
        text_marker.pose.position.y = y;
        text_marker.pose.position.z = z + marker_size + 0.01; // Slightly above the sphere
        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = marker_size * 1.5; // Text height
        text_marker.color.r = 1.0f;
        text_marker.color.g = 1.0f;
        text_marker.color.b = 1.0f;
        text_marker.color.a = 1.0f;
        text_marker.text = "unlabeled_" + std::to_string(marker_index);
        text_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        marker_array.markers.push_back(text_marker);

        // Publish the position data to individual topics
        boost::mutex::scoped_try_lock lock(unlabeled_marker_mutex);
        if (lock.owns_lock())
        {
            // Check if a publisher exists for this unlabeled marker
            pub_it = unlabeled_marker_pub_map.find(marker_index);
            if (pub_it != unlabeled_marker_pub_map.end())
            {
                PointPublisher & pub = pub_it->second;

                if (pub.is_ready)
                {
                    // Build a PointStamped in the Vicon frame
                    geometry_msgs::msg::PointStamped vicon_point_msg;
                    vicon_point_msg.header.stamp = now;
                    vicon_point_msg.header.frame_id = vicon_frame;

                    vicon_point_msg.point.x = x;
                    vicon_point_msg.point.y = y;
                    vicon_point_msg.point.z = z;

                    // Update timestamp of static transform
                    static_tf.header.stamp = now;

                    // Transform the point to the global frame
                    geometry_msgs::msg::PointStamped global_point_msg;
                    tf2::doTransform(vicon_point_msg, global_point_msg, static_tf);

                    // Publish the transformed point
                    pub.publish(global_point_msg);
                }
            }
            else
            {
                // Create a publisher if it doesn't exist, de-duplicating concurrent attempts
                if (pending_unlabeled_marker_publishers.find(marker_index) == pending_unlabeled_marker_publishers.end())
                {
                    pending_unlabeled_marker_publishers.insert(marker_index);
                    lock.unlock();
                    create_unlabeled_marker_publisher(marker_index);
                }
                else
                {
                    // Another thread is already creating this publisher
                    lock.unlock();
                }
            }
        }

        // Broadcast the transform
        tf_broadcaster_->sendTransform(tf_msg);
    }
    
    // Publish the MarkerArray for visualization
    if (!marker_array.markers.empty() && unlabeled_markers_viz_pub_) {
        unlabeled_markers_viz_pub_->publish(marker_array);
    }
}

// Create a publisher for a specific subject and segment
void Communicator::create_segment_publisher(const string subject_name, const string segment_name)
{
    // Launch a thread to create the publisher
    boost::thread(&Communicator::create_segment_publisher_thread, this, subject_name, segment_name);
}

// Thread function to create a segment publisher
void Communicator::create_segment_publisher_thread(const string subject_name, const string segment_name)
{
    // Construct the topic name and key
    std::string topic_name = ns_name + "/segments/" + subject_name + "/" + segment_name;
    std::string key = subject_name + "/" + segment_name;

    // Log publisher creation
    string msg = "Creating publisher for segment " + segment_name + " from subject " + subject_name;
    cout << msg << endl;

    // Create and store the publisher; then clear the pending flag
    boost::mutex::scoped_lock lock(segment_mutex);
    segment_pub_map.insert(std::map<std::string, Publisher>::value_type(key, Publisher(topic_name, this)));
    pending_segment_publishers.erase(key);
    lock.unlock();
}

// Create a publisher for a specific subject and marker
void Communicator::create_marker_publisher(const string subject_name, const string marker_name)
{
    // Launch a thread to create the publisher
    boost::thread(&Communicator::create_marker_publisher_thread, this, subject_name, marker_name);
}

// Thread function to create a marker publisher
void Communicator::create_marker_publisher_thread(const string subject_name, const string marker_name)
{
    // Construct the topic name and key
    std::string topic_name = ns_name + "/markers/" + subject_name + "/" + marker_name;
    std::string key = subject_name + "/" + marker_name;

    // Log publisher creation
    string msg = "Creating publisher for marker " + marker_name + " from subject " + subject_name;
    cout << msg << endl;

    // Create and store the publisher; then clear the pending flag
    boost::mutex::scoped_lock lock(marker_mutex);
    marker_pub_map.insert(std::map<std::string, PointPublisher>::value_type(key, PointPublisher(topic_name, this)));
    pending_marker_publishers.erase(key);
    lock.unlock();
}

// Create a publisher for an unlabeled marker
void Communicator::create_unlabeled_marker_publisher(const unsigned int marker_index)
{
    // Launch a thread to create the publisher
    boost::thread(&Communicator::create_unlabeled_marker_publisher_thread, this, marker_index);
}

// Thread function to create an unlabeled marker publisher
void Communicator::create_unlabeled_marker_publisher_thread(const unsigned int marker_index)
{
    // Construct the topic name (prefix with 'marker_' to avoid starting with a number)
    std::string topic_name = ns_name + "/unlabeled_markers/marker_" + std::to_string(marker_index);

    // Log publisher creation
    string msg = "Creating publisher for unlabeled marker " + std::to_string(marker_index);
    cout << msg << endl;

    // Create and store the publisher; then clear the pending flag
    boost::mutex::scoped_lock lock(unlabeled_marker_mutex);
    unlabeled_marker_pub_map.insert(std::map<unsigned int, PointPublisher>::value_type(marker_index, PointPublisher(topic_name, this)));
    pending_unlabeled_marker_publishers.erase(marker_index);
    lock.unlock();
}

// Main function
int main(int argc, char** argv)
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Communicator>();

    // Connect to the Vicon server
    node->connect();

    // Continuously retrieve frames while ROS 2 is running
    while (rclcpp::ok()){
        node->get_frame();
    }

    // Disconnect from the Vicon server and shut down ROS 2
    node->disconnect();
    rclcpp::shutdown();
    return 0;
}