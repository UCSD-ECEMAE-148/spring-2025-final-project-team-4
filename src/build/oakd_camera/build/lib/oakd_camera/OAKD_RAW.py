#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import cv2

class OAKDCameraNode(Node):
    def __init__(self):
        super().__init__('oakd_camera_node')
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        # Initialize DepthAI pipeline
        self.pipeline = dai.Pipeline()
        cam_rgb = self.pipeline.createColorCamera()
        cam_rgb.setPreviewSize(640, 480)
        cam_rgb.setInterleaved(False)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)

        xout = self.pipeline.createXLinkOut()
        xout.setStreamName("video")
        cam_rgb.preview.link(xout.input)

        # Connect to device and get output queue
        self.device = dai.Device(self.pipeline)
        self.video_queue = self.device.getOutputQueue(name="video", maxSize=4, blocking=False)

        self.get_logger().info("✔️ OAK-D camera initialized.")

        timer_period = 0.10  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        in_frame = self.video_queue.tryGet()
        if in_frame is None:
            self.get_logger().warn("⚠️ No frame received from OAK-D.")
            return

        frame = in_frame.getCvFrame()
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)

    def destroy_node(self):
        self.get_logger().info("🛑 Shutting down OAK-D node.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OAKDCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
