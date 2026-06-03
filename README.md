# UAV System

UAV System is a robotics-oriented project for stereo visual odometry, autonomous perception, and engineering demonstration. The repository contains two main parts:

- `P2P2`: ROS-based stereo visual odometry and camera model code.
- `P3`: UAV system demonstration material, project manual, and video presentation assets.

The project is suitable for UAV perception experiments, stereo visual odometry validation, ROS-based system integration, and course or engineering project demonstration.

## Demo

The repository includes a UAV system demonstration video:

[Watch demo video](https://github.com/TurnXu/uav-system/raw/main/demo.mp4)

## Project Structure

```text
.
+-- README.md
+-- demo.mp4
+-- project3_manual (1).pdf
+-- P2P2/
    +-- README.md
    +-- LICENSE
    +-- report.pdf
    +-- camera_models/
    +-- stereo_vo_estimator/
```

## P2P2: Stereo Visual Odometry

`P2P2` is the code-oriented part of this repository. It provides a ROS Noetic / C++ stereo visual odometry pipeline for RealSense-style stereo image streams.

Main capabilities:

- Stereo image synchronization with ROS `message_filters`.
- Camera model and calibration utilities.
- Feature detection and optical-flow tracking.
- Stereo triangulation from calibrated left and right cameras.
- PnP/RANSAC-based relative pose estimation.
- Quality-aware feature filtering and PnP input selection.
- Lightweight IMU-assisted runtime diagnosis.
- RViz visualization of odometry, path, camera pose, point cloud, and confidence information.

Main directories:

```text
P2P2/
+-- camera_models/          # camera models and calibration utilities
+-- stereo_vo_estimator/    # ROS package for stereo visual odometry
```

Important files:

```text
P2P2/stereo_vo_estimator/src/stereo_vo_node.cpp
P2P2/stereo_vo_estimator/src/estimator.cpp
P2P2/stereo_vo_estimator/src/parameters.cpp
P2P2/stereo_vo_estimator/config/realsense_1/realsense_n3_unsync.yaml
P2P2/stereo_vo_estimator/launch/stereo_vo_bag.launch
```

Default input topics:

```text
/camera/infra1/image_rect_raw
/camera/infra2/image_rect_raw
/djiros/imu
```

Main output topics:

```text
/stereo_vo/Odometry
/stereo_vo/Camera_pose
/stereo_vo/Path
/stereo_vo/PointCloud
/stereo_vo/Relative_pose
/stereo_vo/vo_confidence
/stereo_vo/frame_quality_text
```

For detailed usage, see:

```text
P2P2/README.md
```

## P3: UAV System Demonstration

`P3` is the project demonstration and documentation part. It focuses on system-level presentation rather than standalone source code.

Included material:

```text
project3_manual (1).pdf    # project manual and system documentation
demo.mp4                   # UAV system demonstration video
P2P2/report.pdf            # stereo visual odometry report
```

This part can be used for:

- UAV system design explanation.
- Visual odometry experiment presentation.
- Demo video display.
- Project report submission.
- Engineering or course presentation.

## System Architecture

```text
UAV System
+-- Sensor Layer
|   +-- Stereo camera
|   +-- Camera calibration parameters
|   +-- Optional IMU or flight controller data
+-- Perception Layer
|   +-- Camera model
|   +-- Feature extraction
|   +-- Stereo matching
|   +-- Visual odometry
+-- Estimation Layer
|   +-- Relative pose estimation
|   +-- Motion tracking
|   +-- Trajectory output
+-- Navigation / Control Layer
|   +-- Flight controller interface
|   +-- State fusion
|   +-- Path tracking
+-- Application Layer
    +-- Demo video
    +-- Project report
    +-- Manual documentation
```

## Recommended Environment

Recommended environment for `P2P2`:

```text
Ubuntu 20.04
ROS Noetic
GCC 9
CMake
catkin
C++14 or later
OpenCV
Eigen
PCL
Ceres
RViz
```

Common ROS dependencies:

```bash
sudo apt update
sudo apt install -y ros-noetic-desktop-full
sudo apt install -y python3-rosdep python3-catkin-tools
sudo apt install -y build-essential cmake libeigen3-dev libopencv-dev libpcl-dev libceres-dev
```

## Build P2P2

Build inside a Linux filesystem path. Avoid compiling directly under mounted Windows paths that contain spaces or non-ASCII characters.

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cp -r /path/to/uav-system/P2P2/camera_models ~/catkin_ws/src/
cp -r /path/to/uav-system/P2P2/stereo_vo_estimator ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Run P2P2

Put a compatible ROS1 bag file at:

```text
~/catkin_ws/src/stereo_vo_estimator/bag/realsense_1.bag
```

Launch stereo visual odometry:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch stereo_vo stereo_vo_bag.launch
```

Open RViz in another terminal:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rviz -d ~/catkin_ws/src/stereo_vo_estimator/config/stereo.rviz
```

Check runtime confidence output:

```bash
rostopic echo -n 1 /stereo_vo/vo_confidence
rostopic echo -n 1 /stereo_vo/frame_quality_text
```

## Safety Notes

- Test visual odometry with recorded data before real flight.
- Do not directly feed unvalidated visual odometry output into a flight controller.
- Fuse visual odometry with IMU, GPS, barometer, or other sensors for real UAV navigation.
- Verify camera calibration, topic names, coordinate frames, and timestamp synchronization before running experiments.
- Keep manual override available during real-world UAV tests.

## License

The `P2P2/stereo_vo_estimator` package is MIT licensed. The `P2P2/camera_models` package may include third-party camera model code with separate upstream licensing terms. Review the relevant license files before redistribution.
