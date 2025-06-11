#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray
import serial
import math

class TaskPlanner(Node):
    def __init__(self):
        super().__init__('task_planner_node')

        # ---- Serial ---------------------------------------------------------
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info("✔︎ Serial /dev/ttyACM0 opened")
        except serial.SerialException as e:
            self.get_logger().error(f"‼︎ Serial open failed: {e}")
            self.serial_port = None

        # ---- ROS I/O --------------------------------------------------------
        self.joint_pub = self.create_publisher(Float32MultiArray,
                                               '/arm/joint_commands', 10)
        self.create_subscription(Pose, 'aruco_marker/pose',
                                 self.pose_callback, 10)

        # ---- Sweep parameters ----------------------------------------------
        self.sweeping      = True      # start in sweep mode
        self.base_angle    =   0.0     # start at one end
        self.step_deg      =  10.0     # 10° per sweep tick
        self.timer_period  =  1.15      # seconds between targets
        self.create_timer(self.timer_period, self.sweep_tick)

    # ------------------------------------------------------------------ sweep
    def sweep_tick(self):
        if not self.sweeping:
            return

        # Update angle
        self.base_angle += self.step_deg

        # Reverse at bounds 0 / 180
        if self.base_angle >= 180.0:
            self.base_angle = 180.0
            self.step_deg   = -abs(self.step_deg)
        elif self.base_angle <= 0.0:
            self.base_angle = 0.0
            self.step_deg   =  abs(self.step_deg)

        joints = [self.base_angle, 180.0, 180.0, 60.0]   # demo shoulder/elbow/wrist
        self.publish_and_send(joints)
        self.get_logger().info(f"Sweeping base → {self.base_angle:.1f}°")

    # ------------------------------------------------------------ pose callback
    def pose_callback(self, pose_msg: Pose):
        if self.sweeping:
            self.get_logger().info("✅ Marker detected – stopping sweep")
            self.sweeping = False

        joints = self.compute_dummy_joint_angles(pose_msg)   # returns four 0-180° angles
        self.publish_and_send(joints)

    # ---------------------------------------------------------------- helpers
    def dummy_ik(self, pose: Pose):
        # Very simple placeholder IK – clamp to range
        j0 = 90.0
        j1 = 70.0
        j2 = 135.0
        j3 = 60.0
        return [max(0, min(180, a)) for a in (j0, j1, j2, j3)]

    def publish_and_send(self, angles):
        # publish for RViz / debug
        msg = Float32MultiArray()
        msg.data = angles
        self.joint_pub.publish(msg)

        # clamp & send to Arduino
        if self.serial_port and self.serial_port.is_open:
            clamped = [max(0.0, min(180.0, a)) for a in angles]
            line = ','.join(f"{a:.2f}" for a in clamped) + '\n'
            try:
                self.serial_port.write(line.encode())
            except serial.SerialException as e:
                self.get_logger().warn(f"Serial write failed: {e}")

    def compute_dummy_joint_angles(self, pose_msg: Pose):
        # --------- extract & convert to consistent units ---------------
        x_m, y_m, z_m = pose_msg.position.x, pose_msg.position.y, pose_msg.position.z
        self.get_logger().debug(f"Tag @ ({x_m:.3f}, {y_m:.3f}, {z_m:.3f}) m")

        # Use millimetres to match L1/L2 units
        x = (z_m + 0.012 - 0.010) * 1000.0   # original pz+12 then -L3, in mm
        y =  y_m * 1000.0
        z =  x_m * 1000.0                    # original swap

        L1 = 120.0   # shoulder→elbow (mm)
        L2 = 125.0   # elbow→wrist  (mm)
        r_xy = math.hypot(x, y)

        # out-of-reach check
        if math.hypot(r_xy, z) > (L1 + L2):
            self.get_logger().warn("Tag out of reach")
            return [90.0, 90.0, 90.0, 90.0]

        # ------------- base yaw (deg, 0-180) ---------------------------
        j0 = math.degrees(math.atan2(y, x))
        if j0 < 0:
            j0 += 360.0
        if j0 > 180.0:
            j0 = 360.0 - j0
        j0 = max(0.0, min(180.0, j0))

        # ------------- planar IK for j1 & j2 ---------------------------
        wrist = math.hypot(r_xy, z)
        phi   = math.atan2(z, r_xy)
        acos1 = math.acos((L1**2 + wrist**2 - L2**2) / (2*L1*wrist))
        acos2 = math.acos((L1**2 + L2**2 - wrist**2) / (2*L1*L2))

        j1 = math.degrees(phi + acos1)
        j2 = 180.0 - math.degrees(acos2)     # elbow “fold” angle
        j3 = max(0.0, min(180.0, j2 - j1))   # simple wrist

        # --------- clamp everything to [0,180] -------------------------
        angles = [j0, j1, j2, j3]
        angles = [max(0.0, min(180.0, a)) for a in angles]
        return angles

# ------------------------------------------------------------------- main ---
def main(args=None):
    rclpy.init(args=args)
    node = TaskPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
