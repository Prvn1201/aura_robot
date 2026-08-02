import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the parameter file
    pkg_aura_sim_bringup = get_package_share_directory('aura_sim_bringup')
    params_file = os.path.join(pkg_aura_sim_bringup, 'config', 'autonomy_params.yaml')

    # 1. Line Follower Vision Node (Calculates lane error)
    line_follower_node = Node(
        package='aura_vision',
        executable='line_follower',
        name='line_follower_camera',
        parameters=[params_file],
        output='screen'
    )

    # 2. Line Follower Controller Node (Subscribes to lane error, outputs to twist_mux)
    line_follower_controller_node = Node(
        package='aura_control',
        executable='line_follower_controller',
        name='line_follower_controller',
        parameters=[params_file],
        output='screen'
    )

    # 3. Traffic Light Detector Node
    traffic_light_node = Node(
        package='aura_vision',
        executable='traffic_light_detector',
        name='traffic_light_detector',
        parameters=[params_file],
        output='screen'
    )

    # 4. Tunnel Wall Follower Node
    tunnel_follower_node = Node(
        package='aura_lidar',
        executable='tunnel_follower',
        name='tunnel_wall_follower',
        parameters=[params_file],
        output='screen'
    )

    return LaunchDescription([
        line_follower_node,
        line_follower_controller_node,
        traffic_light_node,
        tunnel_follower_node
    ])
