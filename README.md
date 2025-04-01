# ROS2 Mobile Robot Gazebo Simulation

This repository contains a ROS2-based simulation environment for a mobile robot using **Gazebo**, **SLAM Toolbox**, and **Navigation2**.

## 🚀 Getting Started

### 🏗️ Gazebo Simulation
To launch the Gazebo simulation, run:
```bash
ros2 launch bot_gazebo gazebo.launch.py
```

---

## 🗺️ Mapping using SLAM Toolbox
First, install the required packages:
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

Then, export the RMW implementation:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
```

Install SLAM Toolbox:
```bash
sudo apt install ros-humble-slam-toolbox
```

Launch SLAM Toolbox for mapping:
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

---

## 🧭 Navigation using Navigation2
To launch navigation:
```bash
ros2 launch bot_navigation navigation.launch.py
```

---

## 📜 Additional Notes
- Ensure ROS2 Humble is properly installed.
- Make sure to source the ROS2 workspace before launching:
  ```bash
  source ~/ros2_ws/install/setup.bash
  ```
- Modify the robot parameters in `bot_description` if necessary.

---

**Happy Coding! 🚀🤖**
