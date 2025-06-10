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
            self.get_logger().info('✔︎ Serial /dev/ttyACM1 @ 9600 opened')
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
        self.servo_angles    = [180.0, 70.0, 135.0, 60.0, 0.0]  # Last value is grabber
        self.target_angles   = self.servo_angles.copy()
        self.sweep_direction = -1

        self.GRABBER_OPEN    = 0.0
        self.GRABBER_CLOSE   = 90.0
        self.grabbed         = False
        self.grab_tolerance  = 1.5  # cm

        # Timer: 10 Hz for smooth motion interpolation
        self.timer = self.create_timer(0.1, self.motion_callback)

    # ---------------------------------------------------------------- motion callback
    def motion_callback(self):
        # Smooth interpolation
        step_size = 4.0  # degrees per tick
        updated = False
        new_angles = []

        for i in range(len(self.servo_angles)):
            diff = self.target_angles[i] - self.servo_angles[i]
            if abs(diff) > step_size:
                self.servo_angles[i] += step_size if diff > 0 else -step_size
                updated = True
            else:
                self.servo_angles[i] = self.target_angles[i]  # snap to target
            new_angles.append(self.servo_angles[i])

        # Publish and send if changed
        if updated:
            msg = Float32MultiArray()
            msg.data = new_angles
            self.joint_pub.publish(msg)
            self.send_joint_angles(new_angles)

        if self.sweeping:
            self.target_angles[0] += self.sweep_direction * step_size
            if self.target_angles[0] >= 180.0 or self.target_angles[0] <= 0.0:
                self.sweep_direction *= -1
            self.target_angles[1] = 70.0
            self.target_angles[2] = 135.0
            self.target_angles[3] = 60.0
            self.target_angles[4] = self.GRABBER_OPEN

    # ----------------------------------------------------------------- pose callback
    def pose_callback(self, pose_msg: Pose):
        if self.sweeping:
            self.get_logger().info('✅ Marker detected – stopping sweep')
            self.sweeping = False

        # Compute inverse kinematics (dummy)
        angles = self.compute_dummy_joint_angles(pose_msg)

        # Distance check for grabbing
        distance = math.sqrt(pose_msg.position.x**2 + pose_msg.position.y**2 + pose_msg.position.z**2)
        if distance < self.grab_tolerance and not self.grabbed:
            self.get_logger().info('🤖 Object within reach – activating grabber')
            angles.append(self.GRABBER_CLOSE)
            self.grabbed = True
        else:
            angles.append(self.GRABBER_OPEN)

        self.target_angles = angles

    # ---------------------------------------------------------- dummy inverse kinematics
    def compute_dummy_joint_angles(self, pose_msg: Pose):
        px, py, pz = pose_msg.position.x, pose_msg.position.y, pose_msg.position.z
        self.get_logger().info(f"Position: x={px:.2f}, y={py:.2f}, z={pz:.2f}")
        
        pz += 12
        L1, L2, L3 = 12, 12.5, 10
        pz -= L3
        x, y, z = pz, px, py  # remapped
        r = math.sqrt(x*x + y*y)
        if math.sqrt(x*x + y*y + z*z) > (L1 + L2):
            self.get_logger().info("🛑 Marker too far")
            return [180, 90, 0, -90]

        j0 = self.servo_angles[0] - math.degrees(math.atan2(y, x))
        j1 = math.atan2(z, math.sqrt(x*x + y*y)) + math.acos((r*r + L1*L1 - L2*L2) / (2 * L1 * r))
        j2 = (math.acos((L1*L1 + L2*L2 - r*r) / (2 * L1 * L2)) - math.pi) * -1
        j3 = j2 - j1

        return [j0, math.degrees(j1), math.degrees(j2), math.degrees(j3)]

    # -------------------------------------------------------------------- send to Arduino
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
