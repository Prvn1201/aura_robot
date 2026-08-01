"""
web_dashboard.launch.py
------------------------
Standalone launch file for the phone/tablet control dashboard.

Run this ALONGSIDE your existing simulation launch (e.g. sim.launch.py),
in a second terminal — it doesn't touch or replace anything from that file:

    # terminal 1 (unchanged)
    ros2 launch aura_sim_bringup sim.launch.py

    # terminal 2 (new)
    ros2 launch aura_sim_dashboard web_dashboard.launch.py

It starts three things:
  1. rosbridge_websocket  - exposes ROS topics over a WebSocket (port 9090)
     so the browser page can talk to ROS directly (cmd_vel, /scan).
  2. web_video_server      - re-serves /image_raw as an MJPEG HTTP stream
     (port 8080) that any <img> tag can display, no WebRTC/H264 needed.
  3. a plain Python HTTP server serving this package's web/ folder
     (port 8000) - that's the actual dashboard page.

Then, on your phone/tablet (same WiFi as the laptop), open:
    http://<laptop-ip>:8000
"""

import socket
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, LogInfo, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def _local_ip():
    """Best-effort guess of this machine's LAN IP, for the friendly banner."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    except OSError:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip


def generate_launch_description():
    # --- UPDATED TO AURA PACKAGE ---
    pkg_share = get_package_share_directory('aura_sim_dashboard')
    web_dir = os.path.join(pkg_share, 'web')
    recorder_script = os.path.join(pkg_share, 'scripts', 'data_recorder_node.py')

    dashboard_port = 8000
    rosbridge_port = 9090
    video_port = 8080

    dataset_dir_arg = DeclareLaunchArgument(
        'dataset_dir',
        default_value=os.path.expanduser('~/nxgv_datasets'),
        description='Root folder where recording sessions (frames/ + log.csv) are saved.'
    )

    # 1. rosbridge websocket server
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rosbridge_server'),
            'launch', 'rosbridge_websocket_launch.xml')]),
        launch_arguments={'port': str(rosbridge_port)}.items(),
    )

    # 2. web_video_server (MJPEG re-streamer for /image_raw)
    web_video = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{'port': video_port}],
    )

    # 3. static file server for the dashboard page itself
    dashboard_http = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', str(dashboard_port)],
        cwd=web_dir,
        output='screen',
    )

    # 4. data recorder node — Fasa 2 item 1: sync frame + steering + speed -> CSV
    data_recorder = ExecuteProcess(
        cmd=['python3', recorder_script,
             '--ros-args', '-p', ['output_dir:=', LaunchConfiguration('dataset_dir')]],
        output='screen',
    )

    ip = _local_ip()
    banner = LogInfo(msg=(
        '\n'
        '========================================================\n'
        ' AURA Simulation Control Deck is up.\n'
        f'   Buka di phone/tablet (WiFi sama):  http://{ip}:{dashboard_port}\n'
        f'   rosbridge websocket:               ws://{ip}:{rosbridge_port}\n'
        f'   camera MJPEG stream:               http://{ip}:{video_port}/stream?topic=/image_raw\n'
        '   Tekan butang REC dalam dashboard untuk mula/stop rakam dataset.\n'
        '   Dataset disimpan di: ' + os.path.expanduser('~/nxgv_datasets') + ' (ubah dengan dataset_dir:=<path>)\n'
        '========================================================\n'
    ))

    return LaunchDescription([
        dataset_dir_arg,
        banner,
        rosbridge,
        web_video,
        dashboard_http,
        data_recorder,
    ])
