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



### 1. Install TurtleBot3 Packages
Make sure you have the TurtleBot3 packages installed:
```bash
sudo apt-get install ros-<ros-distro>-turtlebot3
