import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
from cv2 import aruco
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
from ament_index_python.packages import get_package_share_directory


class ImageListenerNode(Node):
    def __init__(self):
        super().__init__('image_listener_node')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.MARKER_SIZE = 12.14  # cm
        self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        self.detector_params = aruco.DetectorParameters()
        self.rVecs_hist = []
        self.tVecs_hist = []
        self.HIST_SIZE = 5
        self.clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        # Load camera calibration
        calib_data_path = os.path.join(
            get_package_share_directory('ros2_aruco_perception'),
            'calib_data',
            'MultiMatrix.npz'
        )
        calib_data = np.load(calib_data_path)
        self.cam_mat = calib_data["camMatrix"]
        self.dist_coef = calib_data["distCoef"]



    def image_callback(self, msg):
        #self.get_logger().info('Received an image frame!')

        try:
             self.get_logger().info('Recieved an image frame!')
             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        gray = self.clahe.apply(gray)

        corners, ids, _ = aruco.detectMarkers(gray, self.marker_dict) #, parameters=self.detector_params) #somthing here doesnt exist

        if ids is not None:
            #self.get_logger().info('procces 1 started')
            aruco.drawDetectedMarkers(frame, corners, ids)
            obj_points = np.array([
                [-self.MARKER_SIZE/2,  self.MARKER_SIZE/2, 0],
                [ self.MARKER_SIZE/2,  self.MARKER_SIZE/2, 0],
                [ self.MARKER_SIZE/2, -self.MARKER_SIZE/2, 0],
                [-self.MARKER_SIZE/2, -self.MARKER_SIZE/2, 0]
            ], dtype=np.float32)

            for marker in corners:
                img_points = marker.reshape((4, 2)).astype(np.float32)
                success, rvec, tvec = cv.solvePnP(obj_points, img_points, self.cam_mat, self.dist_coef)
                if success:
                    if rvec.shape != (3, 1):
                        rvec = rvec.reshape((3, 1))
                    if tvec.shape != (3, 1):
                        tvec = tvec.reshape((3, 1))
         #   self.get_logger().info('proccess 2 ended')
                    self.rVecs_hist.append(rvec)
                    self.tVecs_hist.append(tvec)
                    if len(self.rVecs_hist) > self.HIST_SIZE:
                        self.rVecs_hist.pop(0)
                        self.tVecs_hist.pop(0)

            if self.rVecs_hist:
                avg_tvec = np.mean(self.tVecs_hist, axis=0)
                x, y, z = avg_tvec.flatt
            if len(self.rVecs_hist) > 0:
                avg_rvec = np.mean(self.rVecs_hist, axis=0)
                avg_tvec = np.mean(self.tVecs_hist, axis=0)

                if avg_rvec.shape != (3, 1):
                    avg_rvec = avg_rvec.reshape((3, 1))
                if avg_tvec.shape != (3, 1):
                    avg_tvec = avg_tvec.reshape((3, 1))
                
                x, y, z = avg_tvec.flatten()
                self.get_logger().info(f"Detected ArUco marker at: x={x:.2f} cm, y={y:.2f} cm, z={z:.2f} cm")
                cv.drawFrameAxes(frame, self.cam_mat, self.dist_coef, avg_rvec, avg_tvec, 4)
        #cv.imshow("Aruco Detection", frame)
        #cv.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageListenerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
