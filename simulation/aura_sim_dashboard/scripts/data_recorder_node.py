#!/usr/bin/env python3
"""
data_recorder_node.py
----------------------
Fasa 2 / item 1 — data collection untuk CNN lane-following.

Setiap kali /recording/enable menerima True, satu sesi baru dimulakan:
    <output_dir>/session_YYYYMMDD_HHMMSS/
        frames/frame_000000.jpg, frame_000001.jpg, ...
        log.csv   -> frame_file, timestamp_sec, steering_angle_rad, speed_mps

Label (steering_angle, speed) diambil dari /joint_states — iaitu nilai
YANG BENAR-BENAR TERCAPAI secara fizikal dalam simulasi (bukan cmd_vel
mentah pemandu/joystick), supaya ia padan dengan apa yang CNN akan
predict & publish balik nanti (steering_angle terus, bukan Twist).

  steering_angle = purata position front_left_steering_joint & front_right_steering_joint (rad)
  speed          = purata velocity rear_left_wheel_joint & rear_right_wheel_joint (rad/s) * wheel_radius (m)

Setiap frame kamera yang sampai semasa recording aktif dipasangkan dengan
nilai /joint_states TERKINI yang diterima (bukan interpolasi) — cukup
tepat sebab joint_states (~100Hz) jauh lebih laju dari kamera (~30Hz).

Kawalan recording:
  Subscribe : /recording/enable   (std_msgs/Bool)   True=start, False=stop
  Publish   : /recording/active   (std_msgs/Bool)   status semasa, 2Hz
  Publish   : /recording/frame_count (std_msgs/Int32) jumlah frame semasa sesi ini, 2Hz
"""

import os
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Int32
from cv_bridge import CvBridge
import cv2

# Kena padan dengan robot_dimensions.yaml / robot.urdf.xacro
WHEEL_RADIUS_M = 0.0325

STEER_JOINTS = ['front_left_steering_joint', 'front_right_steering_joint']
WHEEL_JOINTS = ['rear_left_wheel_joint', 'rear_right_wheel_joint']


class DataRecorderNode(Node):

    def __init__(self):
        super().__init__('data_recorder_node')

        self.declare_parameter('output_dir', os.path.expanduser('~/nxgv_datasets'))
        self.declare_parameter('image_topic', '/camera_sensor/image_raw')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('record_topic', '/recording/enable')
        self.declare_parameter('jpeg_quality', 90)

        self.output_root = self.get_parameter('output_dir').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        joint_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value
        record_topic = self.get_parameter('record_topic').get_parameter_value().string_value
        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value

        os.makedirs(self.output_root, exist_ok=True)

        self.bridge = CvBridge()
        self.latest_steering = 0.0
        self.latest_speed = 0.0
        self.have_joint_state = False

        self.recording = False
        self.session_dir = None
        self.frames_dir = None
        self.csv_file = None
        self.csv_writer = None
        self.frame_count = 0

        self.create_subscription(Image, image_topic, self.on_image, 10)
        self.create_subscription(JointState, joint_topic, self.on_joint_state, 50)
        self.create_subscription(Bool, record_topic, self.on_record_toggle, 10)

        self.status_pub = self.create_publisher(Bool, '/recording/active', 10)
        self.count_pub = self.create_publisher(Int32, '/recording/frame_count', 10)
        self.create_timer(0.5, self.publish_status)

        self.get_logger().info(f'Data recorder ready. output_dir={self.output_root}')
        self.get_logger().info(f'  camera topic       = {image_topic}')
        self.get_logger().info(f'  joint_states topic  = {joint_topic}')
        self.get_logger().info(f'  record toggle topic = {record_topic}')

    # ---------------- joint state (ground-truth label) ----------------
    def on_joint_state(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}

        steer_vals = []
        for j in STEER_JOINTS:
            i = idx.get(j)
            if i is not None and i < len(msg.position):
                steer_vals.append(msg.position[i])

        wheel_vals = []
        for j in WHEEL_JOINTS:
            i = idx.get(j)
            if i is not None and i < len(msg.velocity):
                wheel_vals.append(msg.velocity[i])

        if steer_vals:
            self.latest_steering = sum(steer_vals) / len(steer_vals)
        if wheel_vals:
            avg_wheel_rad_s = sum(wheel_vals) / len(wheel_vals)
            self.latest_speed = avg_wheel_rad_s * WHEEL_RADIUS_M

        if steer_vals or wheel_vals:
            self.have_joint_state = True

    # ---------------- recording control ----------------
    def on_record_toggle(self, msg: Bool):
        if msg.data and not self.recording:
            self.start_session()
        elif not msg.data and self.recording:
            self.stop_session()

    def start_session(self):
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.session_dir = os.path.join(self.output_root, f'session_{stamp}')
        self.frames_dir = os.path.join(self.session_dir, 'frames')
        os.makedirs(self.frames_dir, exist_ok=True)

        csv_path = os.path.join(self.session_dir, 'log.csv')
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['frame_file', 'timestamp_sec', 'steering_angle_rad', 'speed_mps'])

        self.frame_count = 0
        self.recording = True
        self.get_logger().info(f'>>> RECORDING START: {self.session_dir}')

    def stop_session(self):
        self.recording = False
        if self.csv_file:
            self.csv_file.close()
        self.get_logger().info(f'>>> RECORDING STOP: {self.frame_count} frames -> {self.session_dir}')
        self.csv_file = None
        self.csv_writer = None
        self.session_dir = None
        self.frames_dir = None

    # ---------------- camera frame (paired with latest label) ----------------
    def on_image(self, msg: Image):
        if not self.recording:
            return
        if not self.have_joint_state:
            # elak simpan frame dengan label default 0.0 yang palsu
            # sebelum /joint_states pertama sampai
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge gagal convert frame: {e}')
            return

        t = self.get_clock().now().nanoseconds / 1e9
        frame_name = f'frame_{self.frame_count:06d}.jpg'
        frame_path = os.path.join(self.frames_dir, frame_name)
        cv2.imwrite(frame_path, cv_img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])

        self.csv_writer.writerow([
            os.path.join('frames', frame_name),
            f'{t:.6f}',
            f'{self.latest_steering:.6f}',
            f'{self.latest_speed:.6f}',
        ])
        self.frame_count += 1

    # ---------------- status broadcast (drives dashboard REC indicator) ----------------
    def publish_status(self):
        active_msg = Bool()
        active_msg.data = self.recording
        self.status_pub.publish(active_msg)

        count_msg = Int32()
        count_msg.data = self.frame_count
        self.count_pub.publish(count_msg)

    def destroy_node(self):
        if self.recording:
            self.stop_session()
        super().destroy_node()


def main():
    rclpy.init()
    node = DataRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
