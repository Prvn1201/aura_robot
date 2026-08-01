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