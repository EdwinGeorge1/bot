#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
import numpy as np
# Backwards-compatibility shim: newer NumPy removed aliases like `np.float`.
# Some third-party packages (e.g. older transforms3d) still reference `np.float`.
# Define it here as the builtin `float` if it does not exist to avoid import errors.
if not hasattr(np, 'float'):
    np.float = float
import tf2_ros
import tf_transformations

# OpenCV ArUco compatibility helpers.
# Newer OpenCV versions provide `aruco.ArucoDetector`; older versions use
# `aruco.detectMarkers` + `aruco.DetectorParameters_create()`.
def _make_aruco_tools(dict_id=aruco.DICT_4X4_50):
    # Dictionary (handle older name variants)
    try:
        dictionary = aruco.getPredefinedDictionary(dict_id)
    except Exception:
        # fallback for very old APIs
        dictionary = getattr(aruco, 'Dictionary_get', lambda x: None)(dict_id)

    # Detector parameters (try both factory names)
    params = None
    if hasattr(aruco, 'DetectorParameters_create'):
        try:
            params = aruco.DetectorParameters_create()
        except Exception:
            params = None
    elif hasattr(aruco, 'DetectorParameters'):
        try:
            params = aruco.DetectorParameters()
        except Exception:
            params = None

    # New API: ArucoDetector
    detector = None
    if hasattr(aruco, 'ArucoDetector'):
        try:
            if params is not None:
                detector = aruco.ArucoDetector(dictionary, params)
            else:
                detector = aruco.ArucoDetector(dictionary)
        except Exception:
            detector = None

    return dictionary, params, detector


class ArucoTFDetector(Node):

    def __init__(self):
        super().__init__("aruco_tf_detector")

        # Parameters
        self.declare_parameter("marker_id", 0)
        self.declare_parameter("marker_size", 0.05)  # meters
        self.declare_parameter("camera_frame", "camera_link")

        self.marker_id = self.get_parameter("marker_id").value
        self.marker_size = self.get_parameter("marker_size").value
        self.camera_frame = self.get_parameter("camera_frame").value

        # Camera intrinsics from /camera/camera_info
        self.camera_matrix = None
        self.dist_coeffs = None

        # Subscribers
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10)

        self.caminfo_sub = self.create_subscription(
            CameraInfo, "/camera/camera_info", self.caminfo_callback, 10)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, "/aruco_pose", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ArUco dictionary + detector (compat across OpenCV versions)
        self.dictionary, self.parameters, self._detector = _make_aruco_tools(aruco.DICT_4X4_50)

        self.get_logger().info("Aruco TF Detector Node Started")


    def caminfo_callback(self, msg: CameraInfo):
        """Load camera calibration from /camera/camera_info."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.camera_frame = msg.header.frame_id
            self.get_logger().info(f"Loaded calibration from {self.camera_frame}")


    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn("Waiting for /camera/camera_info...")
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers (use new ArucoDetector when available)
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            # Legacy API fallback
            if self.parameters is not None:
                corners, ids, _ = aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
            else:
                corners, ids, _ = aruco.detectMarkers(gray, self.dictionary)

        if ids is None:
            cv2.imshow("Aruco Detection", frame)
            cv2.waitKey(1)
            return

        ids = ids.flatten()

        for marker_corner, marker_id in zip(corners, ids):

            if marker_id != self.marker_id:
                continue

            self.get_logger().info(f"Detected Marker ID: {marker_id}")

            # Draw marker on image
            # Use available drawDetectedMarkers (should exist in most versions)
            try:
                aruco.drawDetectedMarkers(frame, [marker_corner])
            except Exception:
                # fallback: do nothing
                pass

            # Define marker 3D corners for solvePnP
            s = self.marker_size / 2.0
            obj_points = np.array([
                [-s,  s, 0],
                [ s,  s, 0],
                [ s, -s, 0],
                [-s, -s, 0]
            ], dtype=np.float32)

            img_points = marker_corner.reshape(4, 2).astype(np.float32)

            # If using new ArucoDetector API, we already have detected corners/ids.
            # Now perform pose estimation using SolvePnP
            ok, rvec, tvec = cv2.solvePnP(
                obj_points,
                img_points,
                self.camera_matrix,
                self.dist_coeffs
            )

            if not ok:
                continue

            # Convert rotation vector to quaternion
            R, _ = cv2.Rodrigues(rvec)
            quat = tf_transformations.quaternion_from_matrix(
                np.vstack((np.hstack((R, [[0], [0], [0]])), [0, 0, 0, 1]))
            )

            # ----------------------
            # PUBLISH PoseStamped
            # ----------------------
            pose = PoseStamped()
            pose.header.frame_id = self.camera_frame
            pose.header.stamp = msg.header.stamp
            pose.pose.position.x = float(tvec[0, 0]) if tvec.ndim > 1 else float(tvec[0])
            pose.pose.position.y = float(tvec[1, 0]) if tvec.ndim > 1 else float(tvec[1])
            pose.pose.position.z = float(tvec[2, 0]) if tvec.ndim > 1 else float(tvec[2])
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            self.pose_pub.publish(pose)

            # ----------------------
            # BROADCAST TF
            # ----------------------
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.camera_frame
            t.child_frame_id = "aruco_marker"
            # t.child_frame_id = f"aruco_marker_{self.marker_id}"


            t.transform.translation.x = pose.pose.position.x
            t.transform.translation.y = pose.pose.position.y
            t.transform.translation.z = pose.pose.position.z
            t.transform.rotation = pose.pose.orientation

            self.tf_broadcaster.sendTransform(t)

            break  # Only first match

        # Show image
        cv2.imshow("Aruco Detection", frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = ArucoTFDetector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
