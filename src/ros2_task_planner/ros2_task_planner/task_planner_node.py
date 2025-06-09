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

        # ---- Serial -----------------------------------------------------------------
        try:
            self.serial_port = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
            self.get_logger().info('✔︎ Serial /dev/ttyACM0 @ 9600 opened')
        except serial.SerialException as e:
            self.get_logger().error(f'‼︎ Serial open failed: {e}')
            self.serial_port = None

        # ---- ROS I/O ----------------------------------------------------------------
        self.joint_pub = self.create_publisher(
            Float32MultiArray, '/arm/joint_commands', 10)

        self.subscription = self.create_subscription(
            Pose, 'aruco_marker/pose', self.pose_callback, 10)

        # ---- Sweep state -------------------------------------------------------------
        self.sweeping        = True
        self.servo_0_angle   = 180.0
        self.sweep_direction = -1       # +1 or -1

        # 10 Hz timer: drive the sweep
        self.timer = self.create_timer(0.5, self.sweep_callback)

    # --------------------------------------------------------------------------- sweep
    def sweep_callback(self):
        if not self.sweeping:
            return

        # increment / decrement
        self.servo_0_angle += self.sweep_direction * 4.0   # 4 deg per tick
        if self.servo_0_angle >= 180.0 or self.servo_0_angle <= 0.0:
            self.sweep_direction *= -1                     # bounce at ends

        self.servo_3_angle = 60.0
        # publish for debug / visualization
        msg = Float32MultiArray()
        msg.data = [self.servo_0_angle, 70.0, 135.0, self.servo_3_angle]
        self.joint_pub.publish(msg)

        # send to Arduino
        self.send_joint_angles(msg.data)

        self.get_logger().info(f'Sweeping servo_0: {self.servo_0_angle:.1f}')

    # --------------------------------------------------------------------- pose → IK →
    def pose_callback(self, pose_msg: Pose):
        if self.sweeping:
            self.get_logger().info('✅ Marker detected – stopping sweep')
            self.sweeping = False

        # Dummy inverse-kinematics  (replace with real IK later)
        joint_angles = self.compute_dummy_joint_angles(pose_msg) # modified to return home position
        # publish and send
        msg = Float32MultiArray()
        msg.data = joint_angles
        self.joint_pub.publish(msg)
        self.send_joint_angles(joint_angles)

    # ----------------------------------------------------------------- helpers --------
    def compute_dummy_joint_angles(self, pose_msg: Pose):
        # simple proportional mapping just for demonstration
        px, py, pz = pose_msg.position.x, pose_msg.position.y, pose_msg.position.z
        self.get_logger().info(f"Position: x={pose_msg.position.x:.2f}, y={pose_msg.position.y:.2f}, z={pose_msg.position.z:.2f}")
        pz=pz+12
        L1=12
        L2=12.5
        L3=10
        pz=pz-L3
        x=pz
        z=py
        y=px
        r=math.sqrt(x*x+y*y)
        if (math.sqrt(x*x+y*y+z*z)>(L1+L2)):
             self.get_logger().info(f"aruco too far away")
             return [180, 90, 0, -90]
        #j0 = 90.0 #max(0.0, min(180.0, px * 10))   # scale & clamp
        j0 = self.servo_0_angle-math.degrees(math.atan(y/x))
        #j1 = 90.0 #max(0.0, min(180.0, py * 10))
        j1= math.atan(z/math.sqrt(x*x+y*y))+math.acos((r*r+L1*L1-L2*L2)/(2*L1*r))
        #j2 = 0.0 #max(0.0, min(180.0, pz * 10))
        j2=(math.acos((L1*L1+L2*L2-r*r)/(2*L1*L2))-math.pi)*-1
        #j3 = 0.0 # 90.0                            # fixed wrist for now
        j3=j2-j1
        return [j0, math.degrees(j1), math.degrees(j2), math.degrees(j3)]

    def send_joint_angles(self, angles):
        if not self.serial_port or not self.serial_port.is_open:
            return
        line = ','.join(f'{a:.2f}' for a in angles) + '\n'
        try:
            self.serial_port.write(line.encode())
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial write failed: {e}')

# --------------------------------------------------------------------------- main -----
def main(args=None):
    rclpy.init(args=args)
    node = TaskPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
