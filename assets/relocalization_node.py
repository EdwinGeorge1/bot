#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import yaml
import numpy as np
import os 

# ROS 2 Messages and Libraries
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
# We use tf_transformations for matrix math
from tf_transformations import quaternion_matrix, inverse_matrix, translation_from_matrix, quaternion_from_matrix

class ArUcoRelocalizer(Node):

    def __init__(self):
        super().__init__('aruco_relocalization_node')

        self.get_logger().info("ArUco Relocalizer node starting...")

        # --- Hardcoded Path (For easy testing, replace with parameter for production) ---
        self.config_file = '/home/edwin/ros2_ws/src/bot/assets/aruco_markers.yaml'
        
        # 1. Parameter Initialization
        self._declare_parameters()
        
        # Load marker configuration
        self.marker_map = self._load_marker_config()

        # State and TF components
        self.relocalization_needed = False
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # The expected frame ID of the marker broadcast by the detector
        self.ARUCO_MARKER_FRAME_PREFIX = "aruco_marker_" 
        
        # 2. Publishers and Subscribers
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            1
        )
        
        self.create_subscription(
            PoseWithCovarianceStamped, 
            '/amcl_pose', 
            self.amcl_callback, 
            1
        )
        self.create_subscription(
            PoseStamped, 
            '/aruco_pose', 
            self.aruco_callback, 
            1
        )

        self.get_logger().info("ArUco Relocalizer initialized. Monitoring AMCL...")

    def _declare_parameters(self):
        """Declares necessary node parameters with defaults."""
        
        threshold_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        self.declare_parameter('amcl_threshold', 0.5, threshold_descriptor)
        self.AMCL_LOST_THRESHOLD = self.get_parameter('amcl_threshold').value

        camera_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        self.declare_parameter('camera_frame', 'camera_link', camera_descriptor)
        self.CAMERA_FRAME = self.get_parameter('camera_frame').value

    def _load_marker_config(self):
        """
        Loads marker data from the hardcoded configuration YAML file.
        
        CORRECTION applied here to fix the 'string indices must be integers' error 
        by explicitly checking the structure.
        """
        if not os.path.exists(self.config_file):
            self.get_logger().error(f"Configuration file NOT FOUND at path: {self.config_file}")
            return {}
            
        try:
            with open(self.config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            if 'markers' not in config or not isinstance(config['markers'], list):
                self.get_logger().error("YAML file is missing the 'markers' key or it is not a list under the top level.")
                return {}

            marker_map = {
                item['id']: item['pose'] 
                for item in config['markers'] 
                if isinstance(item, dict) and 'id' in item and 'pose' in item
            }
            
            self.get_logger().info(f"Successfully loaded {len(marker_map)} marker configuration(s).")
            return marker_map
            
        except Exception as e:
            # The original error was caught here. The structure check above makes this robust.
            self.get_logger().error(f"Failed to load marker config from {self.config_file}: {e}")
            return {}


    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """Monitors the uncertainty of the AMCL pose."""
        # Check covariance of X and Y position
        cov_x = msg.pose.covariance[0] 
        cov_y = msg.pose.covariance[7] 
        uncertainty_trace = cov_x + cov_y

        if uncertainty_trace > self.AMCL_LOST_THRESHOLD:
            if not self.relocalization_needed:
                self.get_logger().warn(f"AMCL uncertainty ({uncertainty_trace:.3f}) exceeds threshold. Relocalization ENABLED.")
            self.relocalization_needed = True
        elif uncertainty_trace < 0.1 and self.relocalization_needed:
            self.relocalization_needed = False

    def aruco_callback(self, msg: PoseStamped):
        """
        Processes detected ArUco marker pose (T_camera_marker) and performs relocalization.
        """
        if not self.relocalization_needed:
            return

        # Assuming marker ID 1 is the one the detector is currently tracking
        # If your detector supports multiple IDs, you must extract the ID from msg.header.frame_id
        # For this setup, we stick to a single target ID check:
        marker_id = 1 

        if marker_id not in self.marker_map:
            self.get_logger().warn(f"Detected marker ID {marker_id} is not configured.")
            return
            
        self.get_logger().info(f"Detected known marker ID {marker_id} while lost. Attempting relocalization.")

        try:
            # 1. T_map_marker (from YAML)
            map_marker_pose_data = self.marker_map[marker_id]
            T_map_marker_mat = self._pose_data_to_matrix(map_marker_pose_data)
            
            # 2. T_camera_marker (from subscription msg)
            T_camera_marker_mat = self._pose_msg_to_matrix(msg.pose)
            
            # 3. T_base_camera (from TF lookup)
            T_base_camera_stamped: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link', 
                self.CAMERA_FRAME, 
                rclpy.time.Time()
            )
            T_base_camera_mat = self._transform_stamped_to_matrix(T_base_camera_stamped)
            
            # 4. Inverses
            T_marker_camera_mat = inverse_matrix(T_camera_marker_mat)
            T_camera_base_mat = inverse_matrix(T_base_camera_mat)

            # 5. Composition: T_map_base = T_map_marker * T_marker_camera * T_camera_base
            # T_map_base = T_map_marker * T_marker_camera * T_camera_base 
            T_map_camera_mat = np.dot(T_map_marker_mat, T_marker_camera_mat)
            T_map_base_mat = np.dot(T_map_camera_mat, T_camera_base_mat)
            
            # 6. Extract final Pose
            base_t = translation_from_matrix(T_map_base_mat)
            base_q = quaternion_from_matrix(T_map_base_mat)
            
            base_x, base_y = base_t[0], base_t[1]
            base_ori = [base_q[0], base_q[1], base_q[2], base_q[3]]

            # 7. Publish the relocalization pose
            self._publish_relocalization_pose(base_x, base_y, base_ori)
            self.relocalization_needed = False
            self.get_logger().info(f"Relocalization complete. AMCL reset to X:{base_x:.2f}, Y:{base_y:.2f}.")
            
        except TransformException as e:
            self.get_logger().error(f"TF Error: Could not lookup base_link to {self.CAMERA_FRAME}: {e}")
        except Exception as e:
            self.get_logger().error(f"Error during calculation: {e}")


    # --- Matrix Utility Functions ---
    def _pose_data_to_matrix(self, pose_data):
        """Converts YAML pose dictionary to 4x4 matrix (T_map_marker)."""
        t = pose_data['position']
        q = pose_data['orientation']
        q_ros = [q[0], q[1], q[2], q[3]]
        M = quaternion_matrix(q_ros)
        M[0:3, 3] = t
        return M

    def _pose_msg_to_matrix(self, pose_msg):
        """Converts Pose message to 4x4 matrix (T_camera_marker)."""
        t = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        q = [pose_msg.orientation.x, pose_msg.orientation.y, 
             pose_msg.orientation.z, pose_msg.orientation.w]
        M = quaternion_matrix(q)
        M[0:3, 3] = t
        return M

    def _transform_stamped_to_matrix(self, tf_msg: TransformStamped):
        """Converts TransformStamped message to 4x4 matrix (T_base_camera)."""
        t = [tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z]
        q = [tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, 
             tf_msg.transform.rotation.z, tf_msg.transform.rotation.w]
        M = quaternion_matrix(q)
        M[0:3, 3] = t
        return M


    def _publish_relocalization_pose(self, x, y, orientation_q):
        """Publishes the high-confidence pose to /initialpose."""
        reloc_pose = PoseWithCovarianceStamped()
        reloc_pose.header.stamp = self.get_clock().now().to_msg()
        reloc_pose.header.frame_id = 'map'
        
        reloc_pose.pose.pose.position.x = x
        reloc_pose.pose.pose.position.y = y
        reloc_pose.pose.pose.position.z = 0.0 
        
        reloc_pose.pose.pose.orientation.x = orientation_q[0]
        reloc_pose.pose.pose.orientation.y = orientation_q[1]
        reloc_pose.pose.pose.orientation.z = orientation_q[2]
        reloc_pose.pose.pose.orientation.w = orientation_q[3]

        # Set very low covariance for high confidence reset
        reloc_pose.pose.covariance[0] = 0.001 
        reloc_pose.pose.covariance[7] = 0.001 
        reloc_pose.pose.covariance[35] = 0.001 

        self.initialpose_pub.publish(reloc_pose)


def main(args=None):
    rclpy.init(args=args)
    aruco_relocalizer = ArUcoRelocalizer()
    rclpy.spin(aruco_relocalizer)
    
    aruco_relocalizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()