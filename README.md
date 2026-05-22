# UAV System

A robotics-oriented UAV project containing stereo visual odometry resources, camera model components, demonstration material, and project documentation. The repository is suitable for UAV perception, navigation, autonomous control experiments, and engineering demonstration.

## Project Overview

This project focuses on UAV autonomous perception and navigation. The current repository includes a ROS-style stereo visual odometry package, camera model package, demo video, and project manual.

The system can be used as a foundation for UAV visual odometry, stereo perception, autonomous navigation experiments, flight-state estimation research, ROS-based UAV integration, and project demonstration.

## Demo Video

Click the link below to watch the UAV system demonstration:

[Watch the demo video](https://github.com/TurnXu/uav-system/raw/main/demo.mp4)

## System Architecture

```text
UAV System
+-- Sensor Layer
|   +-- Stereo camera
|   +-- Camera calibration parameters
|   +-- Optional IMU / flight controller data
+-- Perception Layer
|   +-- Camera model
|   +-- Feature extraction
|   +-- Stereo matching
|   +-- Visual odometry
+-- Estimation Layer
|   +-- Relative pose estimation
|   +-- Motion tracking
|   +-- Trajectory output
+-- Control / Navigation Layer
|   +-- Flight controller interface
|   +-- Path tracking
|   +-- Mission execution
+-- Application Layer
    +-- Demo video
    +-- Project report
    +-- Manual documentation
```

## Hardware Support

Typical supported components:

- UAV frame with onboard computer.
- Stereo camera or depth camera.
- Linux-based onboard computer.
- PX4 / ArduPilot flight controller.
- IMU, GPS, or optical flow sensor.
- Wireless telemetry or network communication module.

The current code structure mainly targets ROS-based visual odometry and camera modeling. Integration with flight controllers can be added through MAVROS, MAVSDK, or a custom communication layer.

## Flight Control

The repository can be integrated into a UAV flight-control pipeline as a perception and state-estimation module.

```text
Stereo images
-> visual odometry
-> relative pose estimation
-> navigation state update
-> flight control command
-> UAV motion execution
```

For real flight, the visual odometry output should be fused with IMU, GPS, barometer, or other sensors before being used as a control input.

## Functional Modules

### Camera Models

Located in:

```text
P2P2/camera_models/
```

This module provides camera model support and calibration-related utilities for visual perception.

### Stereo Visual Odometry

Located in:

```text
P2P2/stereo_vo_estimator/
```

This ROS package provides stereo visual odometry estimation, including stereo camera configuration, relative pose message definition, visual odometry node, RViz configuration, and launch files.

### Documentation and Demo

```text
project3_manual (1).pdf
P2P2/report.pdf
demo.mp4
```

## Environment Dependencies

Recommended environment:

```text
Ubuntu 18.04 / 20.04
ROS Melodic / ROS Noetic
CMake
catkin
C++14 or later
OpenCV
Eigen
RViz
```

For ROS dependencies:

```bash
sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport
sudo apt install ros-noetic-tf ros-noetic-rviz
```

## Deployment Steps

### 1. Enter the Project

```bash
cd "D:/github/无人机系统"
```

### 2. Prepare a Catkin Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

Copy or link the packages:

```bash
cp -r "<repo-path>/P2P2/camera_models" .
cp -r "<repo-path>/P2P2/stereo_vo_estimator" .
```

### 3. Build the Workspace

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 4. Launch Stereo Visual Odometry

```bash
roslaunch stereo_vo_estimator stereo_vo_bag.launch
```

### 5. Visualize in RViz

```bash
rviz -d P2P2/stereo_vo_estimator/config/stereo.rviz
```

## File Directory

```text
.
+-- README.md
+-- demo.mp4
+-- project3_manual (1).pdf
+-- P2P2
    +-- report.pdf
    +-- camera_models
    |   +-- CMakeLists.txt
    |   +-- package.xml
    |   +-- readme.md
    |   +-- include
    |   +-- src
    +-- stereo_vo_estimator
        +-- CMakeLists.txt
        +-- package.xml
        +-- config
        +-- include
        +-- launch
        +-- msg
        +-- src
```

## Safety Notes

- Test visual odometry with recorded data before real flight.
- Do not directly feed unvalidated visual odometry output into a flight controller.
- Use sensor fusion for real UAV navigation.
- Verify camera calibration before running pose estimation.
- Confirm topic names, coordinate frames, and timestamp synchronization.
- Keep manual override available during real-world UAV tests.

## License

This project is intended for UAV research, education, and engineering demonstration. Please refer to the repository license for detailed usage terms.
