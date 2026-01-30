/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Azmyin Md. Kamal
Date: 01/01/24

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

//* Includes
#include "ros2_orb_slam3/monocular.hpp"

//* Constructor
MonocularNode::MonocularNode() :Node("mono_camera_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent
    // this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name
    // this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file
    
    this->declare_parameter("orb_slam3_config", "Zed_left_camera.yaml");
    // this->declare_parameter("", "");

    //* Watchdog, populate default values
    nodeName = "not_set";
    // vocFilePath = "file_not_set";
    // settingsFilePath = "file_not_set";

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());

    // rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    // vocFilePath = param2.as_string();

    // rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    // settingsFilePath = param3.as_string();

    orbSLAM3settingsFile = this->get_parameter("orb_slam3_config").as_string();

    // rclcpp::Parameter param4 = this->get_parameter("settings_file_name_arg");
    
    
    initializeOrbSLAM(orbSLAM3settingsFile);

    // RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());

    subImgMsgName = "/mono_py_driver/img_msg"; // topic to receive RGB image messages

    //* subscrbite to the image messages coming from the Python driver node
    subImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 1, std::bind(&MonocularNode::Img_callback, this, _1));

    
    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
}

//* Destructor
MonocularNode::~MonocularNode()
{   
    
    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
    pass;

}

//* Method to bind an initialized VSLAM framework to this node
void MonocularNode::initializeOrbSLAM(std::string &configFileString){
    //* Find path to home directory
    homeDir = getenv("HOME");
    // std::cout<<"Home: "<<homeDir<<std::endl;
    //* HARDCODED, set paths
    vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());

    //* Build .yaml`s file path
    settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/" + "Monocular" + "/" + configFileString; // "Monocular" will be a variable in a next version

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::MONOCULAR; 
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularNode node initialized" << std::endl; // TODO needs a better message
}

//*Helper that processes timestep on the image's header
double MonocularNode::extract_timestamp_from_header(const builtin_interfaces::msg::Time &stamp)
{
    return stamp.sec*1.0E9 + stamp.nanosec*1.0;
}

//* Callback to process image message and run SLAM node
void MonocularNode::Img_callback(const sensor_msgs::msg::Image &msg)
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
    
    // std::cout<<std::fixed<<"Timestep: "<<timeStep<<std::endl; // Debug
    // RCLCPP_INFO(this->get_logger(), "Pointer successfully created");
    timestamp = extract_timestamp_from_header(cv_ptr->header.stamp);
    
    //* Perform all ORB-SLAM3 operations in Monocular mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timestamp); 
    
    //* An example of what can be done after the pose w.r.t camera coordinate frame is computed by ORB SLAM3
    //Sophus::SE3f Twc = Tcw.inverse(); //* Pose with respect to global image coordinate, reserved for future use

}


