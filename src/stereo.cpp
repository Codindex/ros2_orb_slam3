//* Includes
#include "ros2_orb_slam3/stereo.hpp"

//* Constructor
StereoNode::StereoNode() :Node("stereo_camera_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("orb_slam3_config", "Zed_left_camera.yaml");
    this->declare_parameter("camera_topic", "/mono_py_driver/img_msg");

    initializeOrbSLAM();

    subImgMsgName = this->get_parameter("camera_topic").as_string(); // topic to receive RGB image messages
    
    auto node_namespace = this->get_namespace();
    auto node_name = this->get_name();

    RCLCPP_INFO(this->get_logger(), "node_namespace %s", node_namespace);
    RCLCPP_INFO(this->get_logger(), "node_name: %s", node_name);

    pubTransform = "tf";
    pubTransformStamped = "tf_stamped";
    pubOutput = subImgMsgName + "/orbslam3";

    //* subscrbite to the image messages coming from the Python driver node
    subImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::CompressedImage>(subImgMsgName, 1, std::bind(&StereoNode::Img_callback, this, _1));

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
    auto settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/" + "Monocular" + "/" + configFileString; // "Monocular" will be a variable in a next version

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::MONOCULAR; // Monocular, Stereo, RGBD; with/without IMU
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "StereoNode node initialized" << std::endl; // TODO needs a better message
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


