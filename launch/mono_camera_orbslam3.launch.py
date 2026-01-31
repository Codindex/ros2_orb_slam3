from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    orbslam_namespace = LaunchConfiguration('orbslam_namespace')
    orbslam_node_name = LaunchConfiguration('orbslam_node_name')
    driver_node_name = LaunchConfiguration('driver_node_name')
    ros_parameters_file = LaunchConfiguration('ros_parameters_file')

    orbslam_namespace_launch_arg = DeclareLaunchArgument(
        'orbslam_namespace',
        default_value='mono_py_driver'
    )
    orbslam_node_name_launch_arg = DeclareLaunchArgument(
        'orbslam_node_name',
        default_value='driver_test'
    )
    driver_node_name_launch_arg = DeclareLaunchArgument(
        'driver_node_name',
        default_value='py_driver'
    )
    ros_parameters_file_launch_arg = DeclareLaunchArgument(
        'ros_parameters_file',
        default_value='rosbag2_2024_09_19.yaml'
    )

    python_driver_node = Node(
        package='ros2_orb_slam3',
        executable='zed_left_driver_node.py',
        namespace=orbslam_namespace,
        name=driver_node_name,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('ros2_orb_slam3'),
                'config',
                ros_parameters_file
            ])
        ]
    )

    orbslam3_node = Node(
        package='ros2_orb_slam3',
        executable='mono_camera_node_cpp',
        namespace=orbslam_namespace,
        name=orbslam_node_name,
        output='screen',
        emulate_tty=True,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('ros2_orb_slam3'),
                'config',
                ros_parameters_file
            ])
        ]
    )

    return LaunchDescription([
        orbslam_namespace_launch_arg,
        orbslam_node_name_launch_arg,
        driver_node_name_launch_arg,
        ros_parameters_file_launch_arg,
        python_driver_node,
        orbslam3_node,
    ])
