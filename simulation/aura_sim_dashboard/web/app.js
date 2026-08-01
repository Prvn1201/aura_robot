// === IP AUTODETECT & SAMBUNGAN ROS ===
const hostIP = window.location.hostname || 'localhost';
const rosbridgeURL = `ws://${hostIP}:9090`;
const cameraStreamURL = `http://${hostIP}:8080/stream?topic=/camera_sensor/image_raw&type=mjpeg`;

document.getElementById('camera-stream').src = cameraStreamURL;

const ros = new ROSLIB.Ros({ url: rosbridgeURL });

ros.on('connection', () => {
  document.getElementById('connection-status').innerText = 'WEBSOCKET: CONNECTED';
  document.getElementById('connection-status').className = 'status-badge connected';
});
ros.on('error', () => {
  document.getElementById('connection-status').innerText = 'WEBSOCKET: ERROR';
  document.getElementById('connection-status').className = 'status-badge disconnected';
});
ros.on('close', () => {
  document.getElementById('connection-status').innerText = 'WEBSOCKET: DISCONNECTED';
  document.getElementById('connection-status').className = 'status-badge disconnected';
});

// === PUBLISHER KERETA (/cmd_vel) ===
const cmdVelTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/cmd_vel',
  messageType: 'geometry_msgs/msg/Twist'
});
cmdVelTopic.advertise(); // Beritahu ROS awal-awal kita nak memandu

// === TELEMETRI IMU (/imu) ===
const imuTopic = new ROSLIB.Topic({ ros: ros, name: '/imu', messageType: 'sensor_msgs/msg/Imu' });
imuTopic.subscribe((msg) => {
  const q = msg.orientation;
  const sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  const cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  const roll = Math.atan2(sinr_cosp, cosr_cosp) * (180 / Math.PI);
  const sinp = 2 * (q.w * q.y - q.z * q.x);
  const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * (Math.PI / 2) * (180 / Math.PI) : Math.asin(sinp) * (180 / Math.PI);
  const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  const yaw = Math.atan2(siny_cosp, cosy_cosp) * (180 / Math.PI);

  document.getElementById('val-imu-roll').innerText = roll.toFixed(1) + '°';
  document.getElementById('val-imu-pitch').innerText = pitch.toFixed(1) + '°';
  document.getElementById('val-imu-yaw').innerText = yaw.toFixed(1) + '°';
});

// === LIDAR RADAR SCANNER (/scan) ===
const scanTopic = new ROSLIB.Topic({ ros: ros, name: '/scan', messageType: 'sensor_msgs/msg/LaserScan' });
const canvas = document.getElementById('lidar-canvas');
const ctx = canvas.getContext('2d');
const centerX = canvas.width / 2;
const centerY = canvas.height / 2;
const scale = 25;

scanTopic.subscribe((msg) => {
  ctx.fillStyle = 'rgba(0, 9, 17, 0.3)';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  
  ctx.strokeStyle = 'rgba(0, 243, 255, 0.3)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(centerX, 0); ctx.lineTo(centerX, canvas.height);
  ctx.moveTo(0, centerY); ctx.lineTo(canvas.width, centerY);
  ctx.stroke();

  ctx.fillStyle = '#ff0055';
  ctx.beginPath();
  ctx.arc(centerX, centerY, 4, 0, 2 * Math.PI);
  ctx.fill();

  ctx.fillStyle = '#00ff88';
  let angle = msg.angle_min;
  for (let i = 0; i < msg.ranges.length; i++) {
    const range = msg.ranges[i];
    if (range > msg.range_min && range < msg.range_max) {
      const x = centerX - (range * Math.sin(angle) * scale);
      const y = centerY - (range * Math.cos(angle) * scale);
      ctx.fillRect(x, y, 2, 2);
    }
    angle += msg.angle_increment;
  }
});

// === BUTANG RAKAMAN DATASET (FASA 2) ===
let isRecording = false;
const recActiveTopic = new ROSLIB.Topic({ ros: ros, name: '/recording/active', messageType: 'std_msgs/msg/Bool' });
const recFramesTopic = new ROSLIB.Topic({ ros: ros, name: '/recording/frame_count', messageType: 'std_msgs/msg/Int32' });
const toggleRecTopic = new ROSLIB.Topic({ ros: ros, name: '/recording/enable', messageType: 'std_msgs/msg/Bool' });

recActiveTopic.subscribe((msg) => {
  isRecording = msg.data;
  const btn = document.getElementById('btn-rec');
  const badge = document.getElementById('rec-status');
  if (isRecording) {
    btn.innerText = 'STOP RAKAM (REC ON)';
    btn.style.background = 'rgba(255,0,85,0.8)';
    badge.innerText = 'REC: ACTIVE';
    badge.className = 'status-badge rec-on';
  } else {
    btn.innerText = 'MULA RAKAM (REC)';
    btn.style.background = 'rgba(255,0,85,0.15)';
    badge.innerText = 'REC: OFF';
    badge.className = 'status-badge rec-off';
  }
});

recFramesTopic.subscribe((msg) => document.getElementById('val-frames').innerText = msg.data);
document.getElementById('btn-rec').addEventListener('click', () => {
  toggleRecTopic.publish(new ROSLIB.Message({ data: !isRecording }));
});

// =========================================================
// === JOYSTICK KASTAM (REBUILT DARI AWAL, TANPA NIPPLEJS) ===
// =========================================================
const joyBase = document.getElementById('custom-joystick-base');
const joyStick = document.getElementById('custom-joystick-stick');

let linX = 0.0;
let angZ = 0.0;
let publishInterval = null;
let isDragging = false;

// KELAJUAN MAXIMUM (Sama seperti teleop)
const MAX_LIN_VEL = 1.5;   
const MAX_ANG_VEL = 2.0;   
const MAX_RADIUS = 80; // 140px/2

function updateJoystick(clientX, clientY) {
  const rect = joyBase.getBoundingClientRect();
  const centerX = rect.left + MAX_RADIUS;
  const centerY = rect.top + MAX_RADIUS;

  let dx = clientX - centerX;
  let dy = clientY - centerY;

  // Pastikan stick tak terkeluar dari bulatan base
  const dist = Math.sqrt(dx * dx + dy * dy);
  if (dist > MAX_RADIUS) {
    dx = (dx / dist) * MAX_RADIUS;
    dy = (dy / dist) * MAX_RADIUS;
  }

  // Animasi pergerakan stick
  joyStick.style.transform = `translate(${dx}px, ${dy}px)`;
  joyStick.style.transition = 'none';

  // MATEMATIK ROS2: 
  // dy negatif bermakna joystick ditolak ke Atas (Gerak depan -> Linear X Positif)
  // dx negatif bermakna joystick ditarik ke Kiri (Pusing kiri -> Angular Z Positif)
  linX = (-dy / MAX_RADIUS) * MAX_LIN_VEL;
  angZ = (-dx / MAX_RADIUS) * MAX_ANG_VEL;

  // Paparkan Nilai
  document.getElementById('val-lin-x').innerHTML = `${linX.toFixed(2)} <small>m/s</small>`;
  document.getElementById('val-ang-z').innerHTML = `${angZ.toFixed(2)} <small>rad</small>`;
}

function startDrag(e) {
  e.preventDefault();
  isDragging = true;
  const clientX = e.clientX || (e.touches && e.touches[0].clientX);
  const clientY = e.clientY || (e.touches && e.touches[0].clientY);
  updateJoystick(clientX, clientY);

  // Mula tembak arahan 20 kali sesaat (20Hz)
  if (!publishInterval) {
    publishInterval = setInterval(() => {
      const twist = new ROSLIB.Message({
        linear: { x: Number(linX), y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: Number(angZ) }
      });
      cmdVelTopic.publish(twist);
    }, 50);
  }
}

function moveDrag(e) {
  if (!isDragging) return;
  e.preventDefault();
  const clientX = e.clientX || (e.touches && e.touches[0].clientX);
  const clientY = e.clientY || (e.touches && e.touches[0].clientY);
  updateJoystick(clientX, clientY);
}

function endDrag(e) {
  if (!isDragging) return;
  isDragging = false;
  
  // Joystick melantun balik ke tengah
  joyStick.style.transform = `translate(0px, 0px)`;
  joyStick.style.transition = 'transform 0.2s cubic-bezier(0.175, 0.885, 0.32, 1.275)';
  
  linX = 0.0;
  angZ = 0.0;

  // Hentikan tembakan berkala
  if (publishInterval) {
    clearInterval(publishInterval);
    publishInterval = null;
  }

  // Brek kecemasan supaya kereta betul-betul berhenti
  const stopTwist = new ROSLIB.Message({
    linear: { x: 0.0, y: 0.0, z: 0.0 },
    angular: { x: 0.0, y: 0.0, z: 0.0 }
  });
  cmdVelTopic.publish(stopTwist);

  document.getElementById('val-lin-x').innerHTML = `0.00 <small>m/s</small>`;
  document.getElementById('val-ang-z').innerHTML = `0.00 <small>rad</small>`;
}

// Event Listeners untuk Mouse (PC)
joyBase.addEventListener('mousedown', startDrag);
document.addEventListener('mousemove', moveDrag);
document.addEventListener('mouseup', endDrag);

// Event Listeners untuk Touch (Tablet/Telefon)
joyBase.addEventListener('touchstart', startDrag, { passive: false });
document.addEventListener('touchmove', moveDrag, { passive: false });
document.addEventListener('touchend', endDrag);
