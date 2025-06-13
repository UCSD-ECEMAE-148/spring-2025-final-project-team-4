# spring-2025-final-project-team-4

If testing in wsl in windows admin run:

 '''usbipd bind --busid <bus id>
 usbipd attach --busid <bus id> --wsl

 usbipd list to see if attached'''

ros2 launch ros2_aruco_perception perception_and_planner.launch.py

Link to final fresentation with video of hardwear
https://docs.google.com/presentation/d/1APFi1BjdTKXVH6-0_UxK7G_MsOrpkmDJQneW7MeSyVo/edit?slide=id.g365cdf15545_0_0#slide=id.g365cdf15545_0_0

Team Members
____________________________________

Alex Corrow-MAE
Mathew Pope-MAE
Andy Cao-Math CS
Nakul Nandhakumar

Abstract
______________

We decided to work on adding a robotic arm pioleted by servo motors to our car. With the objective of having the car drive to an object and pick it up. Additional objectives were discused such as bringing the object to another location or finding multiple objects but those objectives were not realised. This project incorporated an arduino micro controler to act as the servo driver which our jetson comunicated with through serial comunication
