import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
import cv2 as cv
from cv2 import aruco
import numpy as np
from scipy.spatial.transform import Rotation as R
from ament_index_python.packages import get_package_share_directory
import serial

class ArUcoPoseEstimator(Node):
    def __init__(self):
        super().__init__('aruco_pose_estimator')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Image, 'pose_estimation/image', 10)
        self.MARKER_SIZE = 12.14  # centimeters
        self.pixels_per_cm = 170 / 20.3  # approx 8.37 pixels per cm
        self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        self.param_markers = aruco.DetectorParameters()
        self.rVecs_hist = []
        self.tVecs_hist = []
        self.HIST_SIZE = 5
        self.clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self.pose_publisher = self.create_publisher(Pose, 'aruco_marker/pose', 10)

        # --- Load calibration data ---
        calib_data_path = os.path.join(
            get_package_share_directory('ros2_aruco_perception'),
            'calib_data',
            'MultiMatrix.npz'
        )
        # calib_data_path = os.path.abspath(calib_data_path)
        calib_data = np.load(calib_data_path)
        self.cam_mat = calib_data["camMatrix"]
        self.dist_coef = calib_data["distCoef"]

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        gray_frame = self.clahe.apply(gray_frame)
        marker_corners, marker_IDs, _ = aruco.detectMarkers(gray_frame, self.marker_dict)

        if marker_corners is not None and len(marker_corners) > 0:
            obj_points = np.array([
                [-self.MARKER_SIZE/2,  self.MARKER_SIZE/2, 0],
                [ self.MARKER_SIZE/2,  self.MARKER_SIZE/2, 0],
                [ self.MARKER_SIZE/2, -self.MARKER_SIZE/2, 0],
                [-self.MARKER_SIZE/2, -self.MARKER_SIZE/2, 0]
            ], dtype=np.float32)

            for corners in marker_corners:
                corners_2d = corners.reshape((4, 2)).astype(np.float32)
                success, rvec, tvec = cv.solvePnP(
                    obj_points, corners_2d, self.cam_mat, self.dist_coef
                )
                self.rVecs_hist.append(rvec)
                self.tVecs_hist.append(tvec)
                if len(self.rVecs_hist) > self.HIST_SIZE:
                    self.rVecs_hist.pop(0)
                    self.tVecs_hist.pop(0)

            avg_rVec = np.mean(np.array(self.rVecs_hist), axis=0)
            avg_tVec = np.mean(np.array(self.tVecs_hist), axis=0)

            # Draw markers and pose axes
            aruco.drawDetectedMarkers(frame, marker_corners, marker_IDs)
            # Draw axes for the first detected marker (optional: loop for all)
            if avg_rVec.shape == (3, 1):
                aruco.drawAxis(frame, self.cam_mat, self.dist_coef, avg_rVec, avg_tVec, 4)
            else:
                for i in range(len(marker_IDs)):
                    aruco.drawAxis(frame, self.cam_mat, self.dist_coef, avg_rVec[i], avg_tVec[i], 4)

            # Publish pose
            rot_matrix, _ = cv.Rodrigues(avg_rVec)
            quat = R.from_matrix(rot_matrix).as_quat()  # [x, y, z, w]
            pose_msg = Pose()
            pose_msg.position.x = float(avg_tVec[0])
            pose_msg.position.y = float(avg_tVec[1])
            pose_msg.position.z = float(avg_tVec[2])
            pose_msg.orientation.x = quat[0]
            pose_msg.orientation.y = quat[1]
            pose_msg.orientation.z = quat[2]
            pose_msg.orientation.w = quat[3]
            self.pose_publisher.publish(pose_msg)

        cv.imshow("frame", frame)
        cv.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_pose_estimator = ArUcoPoseEstimator()
    rclpy.spin(aruco_pose_estimator)
    aruco_pose_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()