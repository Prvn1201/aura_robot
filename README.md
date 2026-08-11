# AURA (Autonomous UMPSA Robotic Architecture)

Welcome to the **AURA** ROS 2 workspace repository. This project contains the complete software stack for an Ackermann-steered autonomous robot (based on the RDK X5 and STM32). It features a full Gazebo simulation environment, custom hardware bridges, dynamic simulation plugins, and a browser-based teleoperation and data collection dashboard.

---

## 📂 Repository Architecture

The repository is highly modularized into three main domains: **Core**, **Hardware**, and **Simulation**. This ensures that the real-world deployment and simulation environments share the same logic without conflicting.

```text
aura_robot/
├── core/                           # Shared assets between Sim and Real Robot
│   ├── aura_bringup/               # Main launch files for the physical robot
│   ├── aura_control/               # C++ Ackermann controller and kinematics
│   └── aura_description/           # URDF, Xacro, 3D meshes, and RViz configurations
│
├── hardware/                       # Packages specific to the physical RDK X5 robot
│   ├── aura_custom_sensors/        # Drivers for custom I2C/SPI sensors
│   ├── aura_dashboard/             # Hardware-specific diagnostic dashboard
│   ├── aura_hw_bringup/            # Hardware initialization scripts
│   ├── aura_lidar/                 # LiDAR driver and filtering nodes
│   ├── aura_stm32_bridge/          # Serial communication bridge for the STM32
│   └── aura_vision/                # Camera drivers and OpenCV processing pipelines
│
└── simulation/                     # Packages specific to the Gazebo environment
    ├── aura_gazebo_plugins/        # C++ plugins for dynamic objects (Traffic Lights, Boom Gates)
    ├── aura_sim_bringup/           # Gazebo worlds, custom track models, and simulation launch files
    └── aura_sim_dashboard/         # Web dashboard for teleop and dataset recording via rosbridge
```

---

# 🚀 Simulation Setup

## 📋 Prerequisites

Before building this workspace, ensure you have:

- ROS 2 installed
- Gazebo 11 installed

Install the required ROS packages:

```bash
sudo apt update

sudo apt install \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-rosbridge-server \
    ros-$ROS_DISTRO-web-video-server \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-joint-state-broadcaster
```

---

## 🛠️ Installation

### 1. Create the workspace

```bash
mkdir -p ~/aura_ws/src
cd ~/aura_ws/src

git clone https://github.com/Prvn1201/aura_robot.git
```

---

### 2. Install ROS dependencies

```bash
cd ~/aura_ws

rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

### 3. Build the workspace

Using `--symlink-install` is recommended because Python scripts and launch files can be edited without rebuilding.

```bash
cd ~/aura_ws

colcon build --symlink-install
```

---

### 4. Source the workspace

```bash
source ~/aura_ws/install/setup.bash
```

To automatically source the workspace every time you open a terminal:

```bash
echo "source ~/aura_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

# 🎮 Running the Simulation

Launch the complete Gazebo simulation:

```bash
ros2 launch aura_sim_bringup sim.launch.py
```

This launch file automatically:

- Starts **Gazebo Server (`gzserver`)**
- Waits 5 seconds for Gazebo initialization
- Starts **Gazebo Client (`gzclient`)**
- Launches `robot_state_publisher`
- Spawns the robot into the simulation
- Opens RViz2

The simulation includes:

- Custom AutoRace track
- Dynamic traffic lights
- Boom gate plugins
- Ackermann robot model
- RViz2 visualization

---

# 🌐 Running the Web Dashboard

Open a **new terminal**.

Source the workspace:

```bash
source ~/aura_ws/install/setup.bash
```

Launch the dashboard:

```bash
ros2 launch aura_sim_dashboard web_dashboard.launch.py
```

This starts:

- ROSBridge WebSocket (**Port 9090**)
- Web Video Server (**Port 8080**)
- HTTP Dashboard Server (**Port 8000**)

---

# 📱 Accessing the Dashboard

### On the same computer

Open:

```
http://localhost:8000
```

### On another device (same Wi-Fi)

The launch terminal prints your local IP address.

Open:

```
http://<your_local_ip>:8000
```

Example:

```
http://192.168.0.105:8000
```

The dashboard provides:

- 📷 Live camera streaming
- 🎮 Virtual joystick teleoperation
- 📊 Steering and throttle monitoring
- 🔴 One-click dataset recording

---

# 📁 Dataset Recording

Press the **REC** button on the dashboard to start recording.

The recorder saves:

- Camera frames
- Steering commands
- Throttle commands
- `log.csv`

Default save location:

```text
~/nxgv_datasets
```

---

# 👨‍💻 Team Members

- **PRAVIN THIRUCHELVAM**
- **SAPIUDDIN BIN SANAWI**
- **RONALD LEE DENG**
- **THAVATCHAI DENGPRADIT A/L EDAF**
- **ONG HANG LE**
- **LAW JING KANG**
- **GABRIEL KOH SUN YWEN**
Testing for peace

---