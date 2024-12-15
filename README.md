# Autonomous-Navigation
# Autonomous Robotics Project: Lane Detection and Visual SLAM

## Overview
This project implements autonomous navigation using lane detection and visual SLAM for a TurtleBot3 robot, leveraging ROS. The system is designed to use camera calibration for lane detection and employ Visual SLAM using ORB-SLAM2 for navigation. Below, you'll find a breakdown of each step involved, the ROS packages used, and how to set up the system.

## Prerequisites
Before starting the project, ensure you have the following:
- ROS installed on your system (Melodic or Noetic recommended).
- TurtleBot3 setup on your robot, including the required dependencies.
- A Raspberry Pi Camera for image capturing.

## Packages Used
### 1. Camera Calibration
- **Library:** `camera_calibration` (from ROS)
- **Package:** `ros-turtlebot3_autorace_camera`

### 2. Lane Detection
- **Package:** `ros-turtlebot3_autorace_camera`  
  This package provides tools to work with the camera and perform lane detection. The following launch files are used for different calibration and detection modes:
  
  - `raspberry_pi_camera_publish`
  - `intrinsic_camera_calibration.launch mode:=action`
  - `extrinsic_camera_calibration.launch mode:=action`
  - `detect_lane.launch mode:=calibration`
  - `turtlebot3_autorace_detect detect_lane.launch mode:=action`

### 3. Autonomous Driving
- **Package:** `turtlebot3_autorace_driving`
  - This package handles the robot's control and lane-following behavior.
  - Launch file: `turtlebot3_autorace_control_lane.launch`

### 4. Bringing Up TurtleBot3
- **Package:** `turtlebot3_bringup`
  - The bringup package is used to initialize the robot's hardware interfaces and run the robot.
  - Launch file: `turtlebot3_robot.launch`

### 5. Visual SLAM with ORB-SLAM2
- **Library:** `ORB-SLAM2`
- **Package:** `orb_slam2_ros`
  - ORB-SLAM2 is used for visual SLAM, which creates a map of the environment using the camera input.
  - Launch file: `orb_slam2_r200_mono`

## Installation



#### 1. Install TurtleBot3 Packages
Make sure you have the TurtleBot3 packages installed:
```bash
sudo apt-get install ros-<ros-distro>-turtlebot3
```

#### 2. Install Camera Calibration and Lane Detection Packages

To install the `ros-turtlebot3_autorace_camera` package, use the following commands:

```bash
cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_autorace_camera.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

#### 3. Camera Calibration
To start the camera calibration process, launch the intrinsic and extrinsic calibration:

```bash

roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch mode:=action
roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=action
```

#### 4. Lane Detection
To detect lanes and calibrate the system, run the following:

```bash

roslaunch turtlebot3_autorace_camera detect_lane.launch mode:=calibration
roslaunch turtlebot3_autorace_detect detect_lane.launch mode:=action
3. Driving the Robot
After detecting the lanes, you can start the robot's lane-following behavior:
```

```bash

roslaunch turtlebot3_autorace_driving turtlebot3_autorace_control_lane.launch
```

#### 5. Bring Up the TurtleBot3
Ensure the robot hardware is initialized:
```bash

roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

#### 6. Install Visual SLAM (ORB-SLAM2)
To install ORB-SLAM2, follow the instructions provided in the official repository:

```bash
Copier le code
cd ~/catkin_ws/src
git clone https://github.com/raulmur/ORB_SLAM2.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
Usage
```

#### 7. Visual SLAM (ORB-SLAM2)
Finally, start ORB-SLAM2 for visual SLAM:

```bash
roslaunch orb_slam2_ros orb_slam2_r200_mono.launch
```
## Conclusion
This project provides an autonomous navigation system for TurtleBot3, using ROS for lane detection and ORB-SLAM2 for visual SLAM. By integrating camera calibration, lane-following algorithms, and visual SLAM.


## Future Work
In the future, we aim to improve the robustness of lane detection and expand the SLAM implementation to include 3D mapping and localization.
