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


# The parameters can be set with ros2 param CLI before calling the save service:
##  For saving trajectory
```bash
ros2 param set /trajectory_saver_node save_format yaml
```
```bash
ros2 param set /trajectory_saver_node save_duration 10.0
```
```bash
ros2 service call /save_trajectory std_srvs/srv/Trigger "{}"
```
    save_format: "yaml", "csv", or "json" (default: "yaml")

    save_duration: seconds of recent data to save (default 0 = all)
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
