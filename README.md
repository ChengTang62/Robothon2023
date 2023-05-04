# Robothon 2023 Grand-Challenge Women in AI and Robotics Team Canada Report

Author: Cheng Tang, Jiyi Zhang, Yixuan Zhang

## Hardware Setup

Robot Platform:
Franka Emika Panda collaborative robot 

Gripper:
Franka Emika standard parallel gripper

Vision System:
Intel Real Sense RGBD camera (D435i)

## Software 

Our development is based on ROS (Robot Operating System). 

The task sequence is stated below:

- Fine tuned YOLOv8 network is used to detect the board and extract the labeled features.
- Coordinates of the detected key features are then used to calculate the position and orientation of the task board.
- Set the origin of the board at blue button.
- Calculate the orientation of the board by matching features.​
- Define the order of the task and call for execution.​
- Plan the motions with relative to the origin (blue button).​

### Localize the board & Press the blue button​

1. Use YOLO v8 to extract the coordinates of the key feature (blue button, red test point).​

2. Adjust the orientation by matching the features. ​

3. Approach down to the blue button while ensuring it stays in the designated location of the image frame.​

4. Set the absolute position of the robotic arm to the origin of the board.​

5. Press the button.​

The robot cature an rgb frame at home pose, identifies the blue button and red test point using YOLO and calculate the center of the circles. The position of the center of blue button is then used to localize the position of the reference frame of the board, the angle of the line upon connecting the center of the blue button and the test point is used to estimate the orientation of the board. 

The robot moves based on the position feedback of the blue button in the rgb frame, and turns a certain angle according to the calculation of the baord's orientation that was done above at home pose. 

Once the robot is right above the blue button, move straight downward and press on the blue button.

### Move the slider​

1. Move the slider to the middle so that the green target arrow shows up.​

2. Move up the robotic arm so that the screen can be seen.​

3. Use color filtering to determine the location of the indicator (yellow) and target (green) arrows.​

4. Scale up the distance between the indicator arrow and the target arrow according to the calibrated values.​

5. Move the slider to the desired position according to the calculated distance.​

### Relocate the plug​

1. Approach the plug and grab the plug​

2. Pull up the plug from black test point and move it above the red test point​

3. Put in the plug to the red test point, open the gripper and go up​

### Open the door and test the circuit​

1. Move to the above of the door handle and grab the door handle​

2. Open the door​

3. Push the door to ensure it is fully opened​

4. Adjust the orientation of the robot and pull out the probe from the probe holder​

5. Insert the probe tip into the circuit​

6. Remove the probe and place it onto the table​

### Route the cable and press the red button​

1. Approach to the side of the task board​

2. Close the gripper so the cable can move freely within the space between grippers​

3. Start wiring the cable from side to side​

4. Once wired up, grab the probe and place it back to the holder​

## Dependencies

Third parties softwares:
- [ROS](https://www.ros.org/)
- [OpenCV] (https://opencv.org/)
- [Roboflow] (https://roboflow.com/)

## Quick Start Guide

``
~/robohub/robothon ./uw_panda/start.sh robothon_latest
``

``
source ws/devel/setup.bash
``

``
roslaunch uw_robothon my.launch
``