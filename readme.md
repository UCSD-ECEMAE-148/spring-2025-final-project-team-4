<div id="top"></div>

<h1 align="center">ARMAR</h1>

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://jacobsschool.ucsd.edu/">
    <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/6/6c/University_of_California%2C_San_Diego_logo.svg/640px-University_of_California%2C_San_Diego_logo.svg.png" alt="UCSD Logo" width="400" height="100">
  </a>
<h3>ECE/MAE148 Final Project</h3>
<p>
Team 4 Spring 2025

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#setup-and-launch">Setup &amp; Launch</a></li>
    <li><a href="#team-members">Team Members</a></li>
    <li><a href="#final-project">Final Project</a></li>
      <ul>
        <li><a href="#original-goals">Original Goals</a></li>
          <ul>
            <li><a href="#goals-we-met">Goals We Met</a></li>
            <li><a href="#stretch-goals">Stretch Goals</a></li>
          </ul>
        <li><a href="#final-project-documentation">Final Project Documentation</a></li>
      </ul>
    <li><a href="#robot-design">Robot Design</a></li>
      <ul>
        <li><a href="#electronic-hardware">Electronic Hardware</a></li>
        <li><a href="#software">Software</a></li>
      </ul>
    <li><a href="#understanding-our-code">Understanding Our Code</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
    <li><a href="#authors">Authors</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>

## Setup & Launch

**Testing in WSL (Windows Admin)**
```bash
usbipd bind --busid <bus id>
usbipd attach --busid <bus id> --wsl
usbipd list  # verify attachment
```

**Launch Perception & Planner**
```bash
ros2 launch ros2_aruco_perception perception_and_planner.launch.py
```

**Final Presentation (Slides & Video of Hardware):**  
https://docs.google.com/presentation/d/1APFi1BjdTKXVH6-0_UxK7G_MsOrpkmDJQneW7MeSyVo/edit?slide=id.g365cdf15545_0_0

<img width="344" alt="Screen Shot 2025-06-13 at 10 55 59 AM" src="https://github.com/user-attachments/assets/01e80524-9195-473b-9471-04279aa8babe" />

## Team Members

<div align="center">
  <p align="center">Alex Corrow, Mathew Pope, Andy Cao, Nakul Nandhakumar</p>
</div>

<h4>Team Member Major and Class</h4>
<ul>
  <li>Alex Corrow - Mechanical and Aerospace Engineering (MAE) - Class of 2025</li>
  <li>Mathew Pope - Mechanical and Aerospace Engineering (MAE) - Class of 2025</li>
  <li>Andy Cao - Mathematics &amp; Computer Science - Class of 2027</li>
  <li>Nakul Nandhakumar - Electrical &amp; Computer Engineering (ECE) - Class of 2025</li>
</ul>

## Final Project

ARMAR (Autonomous Retrieval Machine and Arm Robot) is a small-scale autonomous vehicle equipped with a servo-driven robotic arm, capable of seeking, localizing, and grasping colored objects using ROS2, AprilTag localization, and serial-controlled servos via an Arduino microcontroller.

### Final Project Documentation

* [Final Presentation (Slides & Video)](https://docs.google.com/presentation/d/1APFi1BjdTKXVH6-0_UxK7G_MsOrpkmDJQneW7MeSyVo/edit)

## Original Goals

- **Autonomous Navigation**: Drive autonomously to a target object using ROS2 line-following and color-based detection.
- **Object Detection**: Detect and classify objects via color thresholding and AprilTag localization.
- **Robotic Arm Grasping**: Compute inverse kinematics and control five servos (claw + four joints) to grasp and lift the object.
- **Integrated Control**: Seamless communication between Jetson Nano, ROS2 nodes, and Arduino over serial.

## Goals We Met

- **Autonomous Navigation**: Completed color-based seeking and line-following integration.
- **AprilTag Localization**: Implemented pose estimation for precise arm alignment.
- **IK & Servo Control**: Developed task planner for joint angle computation and serial commands.
- **Grasp & Lift**: Successfully closed claw and lifted objects to home position.

## Stretch Goals

- **Object Delivery**: Plan and carry objects to a designated drop-off location.
- **Return-To-Home**: Autonomous return after task completion.

## Robot Design

### Electronic Hardware

* Circuit diagram and wiring handled via USB serial; Jetson Nano interfaces with Arduino.  

### Software

- **Embedded Systems**: Jetson Nano runs a Docker container with UCSD Robocar modules; communicates with Arduino via USB serial.  
- **ROS2**: Custom perception and planner launch file (`perception_and_planner.launch.py`) coordinating Aruco detection, task planning, and VESC control.  
- **Arduino Firmware**: PWM-based servo driver using the Adafruit ServoKit library on an Arduino Uno.

## Understanding Our Code

<img width="952" alt="Understanding Our Code" src="https://github.com/user-attachments/assets/b0a7a6b9-8c40-4d7f-a209-7ee18657d96d" />

## Acknowledgments

Thank you to Professor Jack Silberman and our TAs Winston and Alexander for guidance, and to Kiersten and Alexander for the README template.

## Authors

- Alex Corrow - MAE
- Mathew Pope - MAE
- Andy Cao - MATH
- Nakul Nandhakumar - ECE

## Contact

* Alex Corrow \| acorrow@ucsd.edu  
* Mathew Pope \| mpope@ucsd.edu  
* Andy Cao \| s8cao@ucsd.edu  
* Nakul Nandhakumar \| nnandhakumar@ucsd.edu

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
