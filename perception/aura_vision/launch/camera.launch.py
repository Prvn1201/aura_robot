import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Find the official realsense2_camera package
    realsense_pkg = get_package_share_directory('realsense2_camera')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_pkg, 'launch', 'rs_launch.py')
            ),
            launch_arguments={
                'camera_name': 'camera',
                'depth_module.profile': '640x480x30',
                'rgb_camera.profile': '640x480x30',
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': '2',
                'align_depth.enable': 'true',
                'pointcloud.enable': 'true', 
                'pointcloud.stream_filter': '2', 
            }.items()
        )
    ])