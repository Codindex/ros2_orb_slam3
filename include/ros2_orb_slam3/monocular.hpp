// Include file 
#ifndef ZED_LEFT_CAMERA_HPP  // Header guard to prevent multiple inclusions
#define ZED_LEFT_CAMERA_HPP

// C++ includes
#include <iostream> // The iostream library is an object-oriented library that provides input and output functionality using streams
#include <algorithm> // The header <algorithm> defines a collection of functions especially designed to be used on ranges of elements.
#include <fstream> // Input/output stream class to operate on files.
#include <chrono> // c++ timekeeper library
#include <vector> // vectors are sequence containers representing arrays that can change in size.
#include <queue>
#include <thread> // class to represent individual threads of execution.
#include <mutex> // A mutex is a lockable object that is designed to signal when critical sections of code need exclusive access, preventing other threads with the same protection from executing concurrently and access the same memory locations.
#include <cstdlib> // to find home directory

#include <cstring>
#include <sstream> // String stream processing functionalities

//* ROS2 includes
//* std_msgs in ROS 2 https://docs.ros2.org/foxy/api/std_msgs/index-msg.html
#include "rclcpp/rclcpp.hpp"

// #include "your_custom_msg_interface/msg/custom_msg_field.hpp" // Example of adding in a custom message
#include <std_msgs/msg/header.hpp>
#include "std_msgs/msg/float64.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include "sensor_msgs/msg/image.hpp"
// #include "ros2_orb_slam3/msg/tracked_compressed_image.hpp"
using std::placeholders::_1; //* TODO why this is suggested in official tutorial

// Include Eigen
// Quick reference: https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
#include <Eigen/Dense> // Includes Core, Geometry, LU, Cholesky, SVD, QR, and Eigenvalues header file

// Include cv-bridge
#include <cv_bridge/cv_bridge.h>

#include <beluga/beluga.hpp>
#include <beluga_ros/tf2_sophus.hpp>

// Include OpenCV computer vision library
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp> // Image processing tools
#include <opencv2/highgui/highgui.hpp> // GUI tools
#include <opencv2/core/eigen.hpp>
#include <image_transport/image_transport.hpp>

//* ORB SLAM 3 includes
#include "orb_slam3/include/System.h" //* Also imports the ORB_SLAM3 namespace

//* Gobal defs
#define pass (void)0 // Python's equivalent of "pass" i.e. no operation


//* Node specific definitions
class MonocularNode : public rclcpp::Node
{   
    //* This slam node inherits from both rclcpp and ORB_SLAM3::System classes
    //* public keyword needs to come before the class constructor and anything else
    public:
    double timestamp; // Timestep data received from the python node

    //* Class constructor
    MonocularNode(); // Constructor 

    ~MonocularNode(); // Destructor
        
    private:
        
        // Class internal variables
        std::string packagePath = "ros2_ws/src/ros2_orb_slam3/"; //! Change to match path to your workspace
        std::string OPENCV_WINDOW = ""; // Set during initialization
        bool bSettingsFromPython = false; // Flag set once when experiment setting from python node is received

        std::string subImgMsgName = ""; // Topic to subscribe to receive RGB images from a python node
        std::string pubTransform = ""; // Topic to publish the tf2 output only
        std::string pubTransformStamped = ""; // Topic to publish the tf2 output only, with timestamp
        std::string pubOutput = ""; // Topic to publish the OrbSLAM3 output

        //* Definitions of publisher and subscribers
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subImgMsg_subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr transform_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transformStamped_publisher_;
        // rclcpp::Publisher<ros2_orb_slam3::msg::TrackedCompressedImage>::SharedPtr output_publisher_;

        //* ORB_SLAM3 related variables
        ORB_SLAM3::System *pAgent; // pointer to a ORB SLAM3 object
        ORB_SLAM3::System::eSensor sensorType;
        bool enablePangolinWindow = false; // Shows Pangolin window output
        bool enableOpenCVWindow = false; // Shows OpenCV window output

        //* ROS callbacks
        void Img_callback(const sensor_msgs::msg::CompressedImage &msg); // Callback to process RGB image and semantic matrix sent by Python node
        void Img_callback_compressed(const sensor_msgs::msg::CompressedImage &msg); // Callback to process RGB image and semantic matrix sent by Python node
        
        //* Helper functions
        // ORB_SLAM3::eigenMatXf convertToEigenMat(const std_msgs::msg::Float32MultiArray& msg); // Helper method, converts semantic matrix eigenMatXf, a Eigen 4x4 float matrix
        void initializeOrbSLAM(); //* Method to bind an initialized VSLAM framework to this node
        double extract_timestamp_from_header(const builtin_interfaces::msg::Time &stamp);
};

#endif
