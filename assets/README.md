# ArUco-Based Relocalization for AMCL (ROS 2)

This module provides automatic AMCL relocalization using an ArUco marker. When AMCL loses track of the robot due to poor scan matching or drift, this node uses a known ArUco marker in the environment to compute the robot’s correct pose and reset AMCL.

---

## 🚀 How It Works

1. **AMCL Monitoring**  
   The node listens to `/amcl_pose` and checks the covariance.  
   If uncertainty rises above a threshold, AMCL is considered **lost**.

2. **Marker Detection**  
   The camera publishes the marker pose relative to the camera on `/aruco_pose`.

3. **TF Lookup**  
   Using the TF tree, the node reads the fixed transform between:

4. **Marker Map Pose (from YAML)**  
The true, fixed position of the marker in the map is defined in:

5. **Pose Calculation**  
The node computes the robot’s actual map pose using three transforms:
- map → marker  *(from YAML)*
- marker → camera  *(from /aruco_pose)*
- camera → base_link  *(from TF)*

6. **Relocalization**  
The final corrected robot pose is published to:
