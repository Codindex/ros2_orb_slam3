#!/usr/bin/env python3

# "mono_driver_node.py" copy with a few changes:
# - Remove global path definitions and variables needed before to process a list of prepared images;
# - Add a new subscription to get CompressedImage from the associated topic;
# - Remove "work variables", we don't need them anymore;
# - remove unused "get_image_dataset_asl" method;
# - run_py_node => camera_image_callback
#   - Change the sender function to be a callback for the new subscribed topic;
#   - Extract the timestamp from the header before submitting;


# Imports
#* Import Python modules
import time # Python timing module
import cv2 # OpenCV

#* ROS2 imports
import rclpy
from rclpy.node import Node

# Import ROS2 message templates
from sensor_msgs.msg import Image, CompressedImage # http://wiki.ros.org/sensor_msgs
from std_msgs.msg import String, Float64 # ROS2 string message template
from cv_bridge import CvBridge, CvBridgeError # Library to convert image messages to numpy array

#* Class definition
class MonoDriver(Node):
    def __init__(self, node_name = "zed_left_camera_mono_py_node"):
        super().__init__(node_name) # Initializes the rclpy.Node class. It expects the name of the node

        # Initialize parameters to be passed from the command line (or launch file)
        self.declare_parameter("settings_name","Zed_left_camera")
        self.declare_parameter("image_seq","NULL")

        #* Parse values sent by command line
        self.settings_name = str(self.get_parameter('settings_name').value) 
        self.image_seq = str(self.get_parameter('image_seq').value)

        # DEBUG
        print(f"-------------- Received parameters --------------------------\n")
        print(f"self.settings_name: {self.settings_name}")
        print(f"self.image_seq: {self.image_seq}")
        print()

        # Global variables
        self.node_name = "zed_left_camera_mono_py_node"

        # Define a CvBridge object
        self.br = CvBridge()

        #* ROS2 publisher/subscriber variables [HARDCODED]
        self.pub_img_to_agent_name = "/mono_py_driver/img_msg"

        # NEW: Subscriber to receive images (CompressedImage type)
        self.subscribe_img_msg_ = self.create_subscription(CompressedImage,
                                                           "/zed/zed_node/left_raw/image_raw_color/compressed", # HARDCODED
                                                           self.camera_image_callback, 10)
        self.subscribe_img_msg_

        # Publisher to send RGB image
        self.publish_img_msg_ = self.create_publisher(CompressedImage, self.pub_img_to_agent_name, 1)


        print()
        print(f"MonoDriver initialized")
    # ****************************************************************************************

    # ****************************************************************************************
    def camera_image_callback(self, image: CompressedImage):
        """
        Master function that sends the RGB image message to the CPP node, from subscibed topic.
        """
        try:
            # print(image.format)
            # cv_img = self.br.compressed_imgmsg_to_cv2(image)
            # img_msg = self.br.cv2_to_imgmsg(cv_img)
            # img_msg.header = image.header
            # print(img_msg.encoding)

            # Publish RGB image, timestamp is in the header :)
            self.publish_img_msg_.publish(image)
        except CvBridgeError as e:
            print(e)
    # ****************************************************************************************
        

# main function
def main(args = None):
    rclpy.init(args=args) # Initialize node
    mono_driver = MonoDriver("zed_left_camera_mono_py_node") #* Initialize the node

    rclpy.spin(mono_driver)

    # Cleanup
    cv2.destroyAllWindows() # Close all image windows
    mono_driver.destroy_node() # Release all resource related to this node
    rclpy.shutdown()

# Dunders, this .py is the main file
if __name__=="__main__":
    main()
