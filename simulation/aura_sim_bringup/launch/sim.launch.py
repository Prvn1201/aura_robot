import os
import re
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    # --- AURA PACKAGE DIRECTORIES ---
    pkg_aura_description = get_package_share_directory('aura_description')
    pkg_aura_sim_bringup = get_package_share_directory('aura_sim_bringup')
    pkg_aura_gazebo_plugins = get_package_share_directory('aura_gazebo_plugins')

    # Gazebo core paths
    gazebo_core_resource_path = '/usr/share/gazebo-11'

    # Models are now in aura_sim_bringup
    models_dir = os.path.join(pkg_aura_sim_bringup, 'models')
    nxgv_track_models_dir = os.path.join(models_dir, 'nxgv_track_models')

    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            models_dir, ':',
            nxgv_track_models_dir, ':',
            os.path.join(gazebo_core_resource_path, 'models'), ':',
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')
        ]
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=[
            models_dir, ':',
            nxgv_track_models_dir, ':',
            gazebo_core_resource_path, ':',
            EnvironmentVariable('GAZEBO_RESOURCE_PATH', default_value='')
        ]
    )

    gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=[
            os.path.join(pkg_aura_gazebo_plugins, '..', '..', 'lib'), ':',
            '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins', ':',
            EnvironmentVariable('GAZEBO_PLUGIN_PATH', default_value='')
        ]
    )

    # 1. URDF is in aura_description
    xacro_file = os.path.join(pkg_aura_description, 'urdf', 'robot.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()
    robot_desc_clean = re.sub(r'<!--.*?-->', '', robot_desc, flags=re.DOTALL)

    # 2. World file is in aura_sim_bringup
    world_file = os.path.join(pkg_aura_sim_bringup, 'worlds', 'litar_sim_nxgv.world')

    # 3. Launch Gazebo 
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')]),
        launch_arguments={'world': world_file}.items()
    )

    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')])
    )

    delayed_gzclient = TimerAction(
        period=5.0,
        actions=[gzclient_launch]
    )

    # 4. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc_clean,
            'use_sim_time': True
        }]
    )

    # 5. Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'ackermann_car',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.0',
                   '-Y', '0.0'],
        output='screen'
    )

    # 6. Spawners Controller
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    front_steering_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["front_steering_position_controller", "--controller-manager", "/controller_manager"],
    )

    rear_velocity_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rear_wheel_velocity_controller", "--controller-manager", "/controller_manager"],
    )

    # 7. Node C++ Ackermann (Moved to aura_control)
    ackermann_controller = Node(
        package='aura_control',
        executable='ackermann_controller_node',
        output='screen'
    )

    # 8. RViz2 (Config is in aura_description)
    rviz_config_file = os.path.join(pkg_aura_description, 'rviz', 'robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo_model_path,
        gazebo_resource_path,
        gazebo_plugin_path,
        gzserver_launch,
        delayed_gzclient,
        robot_state_publisher,
        spawn_entity,
        rviz_node,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[front_steering_spawner, rear_velocity_spawner],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=rear_velocity_spawner,
                on_exit=[ackermann_controller],
            )
        ),
    ])
