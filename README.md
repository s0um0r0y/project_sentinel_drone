# project_sentinel_drone

I developed an autonomous quadcopter for the competition which has the ability to detect an anonymous yellow object placed on a map and return it's coordinates after detecting the block using QGIS API using georeferencing. I also developed an algorithm for a PID controller to stabilize the drone in the Gazebo environment on ROS. We made a scanning algorithm for the drone which scan the entire area autonomously. We used openCV to make an image matching algorithm to detect the yellow box kept on the map.

![Screenshot from 2022-12-09 10-54-11](https://user-images.githubusercontent.com/75070782/215648180-82b8b5ac-0eec-4842-8fff-ac7a7affe762.png)


## Task 1

The task is to build a PID controller for controlling position (x,y,z) of the quadcopter in Gazebo world.
The PID controller will be a closed loop controller with real time position of the quadcopter being fed-back to the controller as a feedback.
The output of the controller will be commands to the quadcopter as angle-setpoints which the quadcopter is supposed to tilt at.
The PID controller will be written as a rosnode written in python programming language
After the PID controller is build and tuned successfully, the quadcopter should be able to move and stabilise at the given setpoint [2,2,20] in the gazebo environment and stay within the error range of ±0.2m in all the coordinates.

![image](https://user-images.githubusercontent.com/75070782/215649603-a01f7cd1-13f3-4596-ba80-385336cd093c.png)

![Screenshot from 2023-01-31 08-16-47](https://user-images.githubusercontent.com/75070782/215649936-ad4df97e-4c4d-48bd-a49e-86ff97c69de8.png)


code is present in position_hold.py

## Task 2

The aim of this task is to write a wrapper over the existing PID control system, written in Task 1 to fly the quadcopter through a list of set points in the simulation environment in Gazebo.
The quadcopter should move through each set point in the gazebo environment
-Takeoff
- [0,0,23]
- [2,0,23]
- [2,2,23]
- [-2,2,23]
- [-2,-2,23]
- [2,-2,23]
- [2,0,23]
- [0,0,23]

A waypoint will be considered success if the drone gets within the error range of ±0.2m in all the coordinate axis for even one instance

code is present in waypoint_navigation.py

## Task 3

In this task I will use computer vision techniques to find the pixel co-ordinates of the yellow block kept on the arena.

Download the image named yellow_detect.jpeg. You will see a yellow block kept on the satellite image.
I used image processing technique to detect the yellow colored block kept on the arena.
I found the pixel co-ordinate of the center of the detected block.

![image](https://user-images.githubusercontent.com/75070782/215650187-7c5d265c-5d83-4ccb-8bd0-9428172f883d.png)

code is present in block_detection.py

## Task 4

I used computer vision techniques to find the location of the image taken from the drone where the object is detected

![image](https://user-images.githubusercontent.com/75070782/215649326-f15cb7d7-6df2-4824-81ea-8b9f63690a7a.png)

code is present in block_locator.py

