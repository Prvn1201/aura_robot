import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate the default RealSense launch file
    realsense_pkg_dir = get_package_share_directory('realsense2_camera')
    realsense_launch_file = os.path.join(realsense_pkg_dir, 'launch', 'rs_launch.py')

    # 2. Include the RealSense node with your custom parameters
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            'camera_name': 'camera',
            'enable_color': 'true',
            'enable_depth': 'true',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'enable_rgbd': 'true',
            'publish_tf': 'true',
            # You can set resolution and FPS here if the default is too heavy
            # 'rgb_camera.profile': '640x480x30',
            # 'depth_module.profile': '640x480x30',
        }.items()
    )

    return LaunchDescription([
        realsense_node
    ])