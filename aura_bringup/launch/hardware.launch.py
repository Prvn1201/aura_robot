import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # 1. Bring in your RealSense Camera
    vision_pkg = get_package_share_directory('aura_vision')
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(vision_pkg, 'launch', 'realsense.launch.py'))
    )

    # 2. Bring in your physical Slamtec LiDAR
    lidar_pkg = get_package_share_directory('sllidar_ros2')
    # Change 'sllidar_a1_launch.py' to match your specific LiDAR model (A1, A2, S1, etc.)
    # Note: We use the base launch file (not the 'view_sllidar' one) to avoid opening RViz automatically
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lidar_pkg, 'launch', 'sllidar_a1_launch.py')),
        launch_arguments={'serial_port': '/dev/ttyUSB0'}.items()
    )

    return LaunchDescription([
        realsense_launch,
        lidar_launch
    ])