#!/usr/bin/env python

# import the LaserScan module from sensor_msgs interface
from geometry_msgs.msg import Pose
import rclpy
import time
from rclpy.node import Node
import numpy as np
import math
from rclpy.qos import ReliabilityPolicy, QoSProfile
import serial
import serial.tools.list_ports

class String2go(Node):

    def __init__(self):
        super().__init__("arm_pos")
        self.get_logger().info('String2go node initialized.')
        self.port = self.find_arduino()
        self.arduino = serial.Serial(self.port, baudrate=9600, timeout=1)
        self.get_logger().info('Arduino found')
        self.subscriber = self.create_subscription(#this will have to change to pull from the perception node instead
            Pose,
            'aruco_marker/pose',
            self.Perception_callback,
            10)
        

    def Perception_callback(self, pose_msg:Pose):
        self.get_logger().info('Received pose data.')
        x = pose_msg.position.x
        y = pose_msg.position.y
        z = pose_msg.position.z
        r= math.sqrt(x*x+y*y+z*z)
        L1=120#length of first segment in mm
        L2=125#length of second segment in mm
        if (x>=0):
            if (r<=math.sqrt(L1*L1+L2*L2)):
                angles=self.find_angles(x,y,z)
                angles_str=self.array2string(angles)
                print("Calculated joint angles (degrees):", angles_str)
                self.arduino.write(angles_str)
            else:
                print("Requested position outside max radius, too far")
        else:
            print("Requested position outside max radius, behind car?")

    def find_arduino(port=None):
        """Get the name of the port that is connected to Arduino."""
        for p in serial.tools.list_ports.comports():
            if p.manufacturer and "Arduino" in p.manufacturer:
                return p.device
        raise IOError("No Arduino found")



    def find_angles(self,x,y,z):#+x should be forward, +y should be left and +Z should be up returns aray angles=[degrees(angle1), degrees(angle2), degrees(angle3), degrees(angle4)]
        #x=x+20#we will need some transformation to get from camera endvector to arm end vector, should just be shift of origin
        #y=y
        #z=z+20
        L1=120#length of first segment in mm
        L2=125#length of second segment in mm
        #L3=166#length of third segment in mm
        angle1=math.atan(y/x)# angle of servo one probably in radians need to change to degrees at end
        r=math.sqrt(x*x+y*y)
        angle2=math.atan(z/math.sqrt(x*x+y*y))+math.acos((r*r+L1*L1-L2*L2)/(2*L1*r))#lefty solution modern robotics pg 220-222
        angle3=math.acos((L1*L1+L2*L2-r*r)/(2*L1*L2))-math.pi
        angle4=angle3-angle2
        angles=[math.degrees(angle1), math.degrees(angle2), math.degrees(angle3), math.degrees(angle4)]
        return angles

    def array2string(self,array):#converts an array to a string and removes the brakets
        WholeString2=str(array)
        WholeString=WholeString2[1:-1]
        return WholeString



def main(args=None):
    rclpy.init(args=args)
    print('ok1')
    time.sleep(5)
    node=String2go()
    #G=find_angles(150,150,0)
    #G=aray2string(G)
    #print(G)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() #call the main function