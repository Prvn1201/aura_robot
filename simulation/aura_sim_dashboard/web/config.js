// ============================================================
// ackermann_web_dashboard — connection config
//
// Nothing here needs editing for normal use: the dashboard is
// opened from the same laptop that runs rosbridge/web_video_server,
// so we just reuse whatever hostname/IP is in the browser's URL bar.
//
// Only touch this if you move rosbridge / web_video_server to a
// different machine than the one serving this page.
// ============================================================
window.NXGV_CONFIG = {
  ROSBRIDGE_PORT: 9090,
  WEB_VIDEO_PORT: 8080,
  CAMERA_TOPIC: '/image_raw',
  SCAN_TOPIC: '/scan',
  IMU_TOPIC: '/imu',
  CMD_VEL_TOPIC: '/cmd_vel',
  RECORD_ENABLE_TOPIC: '/recording/enable',
  RECORD_ACTIVE_TOPIC: '/recording/active',
  RECORD_COUNT_TOPIC: '/recording/frame_count',

  // How fast the joystick publishes /cmd_vel while being held (Hz)
  PUBLISH_RATE_HZ: 15,

  // Max forward speed (m/s) at full joystick deflection
  MAX_LINEAR_SPEED: 0.6,

  // Max turn rate (rad/s) at full joystick deflection
  // (the vehicle's ackermann_controller_node clamps the resulting
  // steering angle itself, so it's safe to send a generous value here)
  MAX_ANGULAR_RATE: 1.8,

  // LiDAR proximity thresholds (metres) for the side rails + radar colour
  PROX_DANGER_M: 0.30,
  PROX_CAUTION_M: 0.80,
  PROX_MAX_DISPLAY_M: 2.5,   // bar/radar fills "full" at this distance
};
