//* Includes
#include "ros2_orb_slam3/stereo.hpp"

//* Constructor
StereoNode::StereoNode() :Node("stereo_camera_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("orb_slam3_config", "Zed_2i_camera.yaml");
    this->declare_parameter("camera_topic", "/mono_py_driver/img_msg");

    initializeOrbSLAM();

    subLeftImgMsgName = this->get_parameter("camera_topic").as_string(); // topic to receive RGB image messages
    subRightImgMsgName = this->get_parameter("right_camera_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "Left topic: %s", subLeftImgMsgName.c_str());
    RCLCPP_INFO(this->get_logger(), "Right topic: %s", subRightImgMsgName.c_str());
    
    auto node_namespace = this->get_namespace();
    auto node_name = this->get_name();

    RCLCPP_INFO(this->get_logger(), "node_namespace %s", node_namespace);
    RCLCPP_INFO(this->get_logger(), "node_name: %s", node_name);

    pubTransform = "tf";
    pubTransformStamped = "tf_stamped";
    pubOutput = subLeftImgMsgName + "/orbslam3";

    //* subscrbite to the image messages coming from the Python driver node
    subLeftImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::CompressedImage>(subLeftImgMsgName, 1, std::bind(&StereoNode::Left_callback, this, _1));
    subRightImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::CompressedImage>(subRightImgMsgName, 1, std::bind(&StereoNode::Right_callback, this, _1));

    transform_publisher_ = this->create_publisher<geometry_msgs::msg::Transform>(pubTransform, 1);
    transformStamped_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(pubTransformStamped, 1);

    output_publisher_ = this->create_publisher<ros2_orb_slam3::msg::TrackedCompressedImage>(pubOutput, 1);
}

//* Destructor
StereoNode::~StereoNode()
{   
    
    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
    pass;

}

//* Method to bind an initialized VSLAM framework to this node
void StereoNode::initializeOrbSLAM(){
    //* Find path to home directory
    std::string homeDir = getenv("HOME");
    // std::cout<<"Home: "<<homeDir<<std::endl;

    //* HARDCODED, set paths
    auto vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());

    auto configFileString = this->get_parameter("orb_slam3_config").as_string();

    //* Build .yaml`s file path
    auto settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/" + "Stereo" + "/" + configFileString; // "Stereo" will be a variable in a next version

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::STEREO; // Monocular, Stereo, RGBD; with/without IMU
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "StereoNode node initialized" << std::endl; // TODO needs a better message
}

void StereoNode::Left_callback(const sensor_msgs::msg::CompressedImage &msg)
{
    left_msg = msg;
    // RCLCPP_INFO(this->get_logger(), "Left image timestamp: %f", extract_timestamp_from_header(left_msg.header.stamp));
    // RCLCPP_INFO(this->get_logger(), "Right image timestamp: %f", extract_timestamp_from_header(right_msg.header.stamp));
    if (right_msg.header.stamp == left_msg.header.stamp)
    {
        Stereo_callback();
    }
}

void StereoNode::Right_callback(const sensor_msgs::msg::CompressedImage &msg)
{
    right_msg = msg;
    // RCLCPP_INFO(this->get_logger(), "Left image timestamp (from right): %f", extract_timestamp_from_header(left_msg.header.stamp));
    // RCLCPP_INFO(this->get_logger(), "Right image timestamp (from right): %f", extract_timestamp_from_header(right_msg.header.stamp));
    if (right_msg.header.stamp == left_msg.header.stamp)
    {
        Stereo_callback();
    }
}

//* Callback to process image message pair and run SLAM node
void StereoNode::Stereo_callback()
{
    // Initialize
    auto left_img_msg = left_msg;
    auto right_img_msg = right_msg;
    cv_bridge::CvImagePtr cv_left_ptr; //* Does not create a copy, memory efficient
    cv_bridge::CvImagePtr cv_right_ptr;
    // RCLCPP_INFO(this->get_logger(), "Received image");
    
    //* Convert ROS image to openCV image
    try
    {
        // RCLCPP_INFO(this->get_logger(), "Try creating pointer");
        cv_left_ptr = cv_bridge::toCvCopy(left_img_msg); // Local scope
        cv_right_ptr = cv_bridge::toCvCopy(right_img_msg);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    // RCLCPP_INFO(this->get_logger(), "Pointer successfully created");

    timestamp = extract_timestamp_from_header(cv_left_ptr->header.stamp);
    // RCLCPP_INFO(this->get_logger(), "Timer extracted from header");
    
    //* Perform all ORB-SLAM3 operations in Stereo mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    Sophus::SE3f Tcw = pAgent->TrackStereo(cv_left_ptr->image, cv_right_ptr->image, timestamp); 
    
    //* An example of what can be done after the pose w.r.t camera coordinate frame is computed by ORB SLAM3
    //Sophus::SE3f Twc = Tcw.inverse(); //* Pose with respect to global image coordinate, reserved for future use

    // Uses beluga_ros package
    auto transformOrbslam = tf2::toMsg(Tcw);

    // Convert to ROS coordinates
    auto transformMessage = geometry_msgs::msg::Transform();
    transformMessage.translation.x = -transformOrbslam.translation.z;
    transformMessage.translation.y = -transformOrbslam.translation.x;
    transformMessage.translation.z = transformOrbslam.translation.y;

    transformMessage.rotation.x = -transformOrbslam.rotation.z;
    transformMessage.rotation.y = -transformOrbslam.rotation.x;
    transformMessage.rotation.z = transformOrbslam.rotation.y;
    transformMessage.rotation.w = transformOrbslam.rotation.w;

    transform_publisher_->publish(transformMessage);

    auto transformStamped = geometry_msgs::msg::TransformStamped();
    transformStamped.header = cv_left_ptr->header;
    transformStamped.transform = transformMessage;
    transformStamped.child_frame_id;

    transformStamped_publisher_->publish(transformStamped);

    auto trackedCompressedImage_message = ros2_orb_slam3::msg::TrackedCompressedImage();
    trackedCompressedImage_message.transform = transformStamped;
    trackedCompressedImage_message.image = *cv_left_ptr->toCompressedImageMsg();

    output_publisher_->publish(trackedCompressedImage_message);
}

//*Helper that processes timestep on the image's header
double StereoNode::extract_timestamp_from_header(const builtin_interfaces::msg::Time &stamp)
{
    return stamp.sec*1.0E9 + stamp.nanosec*1.0;
}

//* Callback to process image message and run SLAM node
void StereoNode::Img_callback(const sensor_msgs::msg::CompressedImage &msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    // RCLCPP_INFO(this->get_logger(), "Received image");
    
    //* Convert ROS image to openCV image
    try
    {
        // RCLCPP_INFO(this->get_logger(), "Try creating pointer");
        cv_ptr = cv_bridge::toCvCopy(msg); // Local scope
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    // RCLCPP_INFO(this->get_logger(), "Pointer successfully created");

    timestamp = extract_timestamp_from_header(cv_ptr->header.stamp);
    // RCLCPP_INFO(this->get_logger(), "Timer extracted from header");
    
    //* Perform all ORB-SLAM3 operations in Monocular mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timestamp); 
    
    //* An example of what can be done after the pose w.r.t camera coordinate frame is computed by ORB SLAM3
    //Sophus::SE3f Twc = Tcw.inverse(); //* Pose with respect to global image coordinate, reserved for future use

    // Uses beluga_ros package
    auto transformOrbslam = tf2::toMsg(Tcw);

    // Convert to ROS coordinates
    auto transformMessage = geometry_msgs::msg::Transform();
    transformMessage.translation.x = -transformOrbslam.translation.z;
    transformMessage.translation.y = -transformOrbslam.translation.x;
    transformMessage.translation.z = transformOrbslam.translation.y;

    transformMessage.rotation.x = -transformOrbslam.rotation.z;
    transformMessage.rotation.y = -transformOrbslam.rotation.x;
    transformMessage.rotation.z = transformOrbslam.rotation.y;
    transformMessage.rotation.w = transformOrbslam.rotation.w;

    transform_publisher_->publish(transformMessage);

    auto transformStamped = geometry_msgs::msg::TransformStamped();
    transformStamped.header = cv_ptr->header;
    transformStamped.transform = transformMessage;
    transformStamped.child_frame_id;

    transformStamped_publisher_->publish(transformStamped);

    auto trackedCompressedImage_message = ros2_orb_slam3::msg::TrackedCompressedImage();
    trackedCompressedImage_message.transform = transformStamped;
    trackedCompressedImage_message.image = msg;

    output_publisher_->publish(trackedCompressedImage_message);
}


