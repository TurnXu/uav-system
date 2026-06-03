# P2P2 Stereo Visual Odometry

P2P2 is a ROS Noetic / C++ stereo visual odometry project for RealSense-style stereo image streams. It estimates camera motion from synchronized stereo images, publishes trajectory and sparse point-cloud outputs for RViz, and includes quality-aware PnP input selection with lightweight IMU-assisted runtime diagnosis.

The system is designed for stereo VO experiments and visualization. It is not a full tightly coupled VIO or SLAM framework: the primary pose estimate comes from stereo feature tracking, triangulation, and PnP/RANSAC. IMU data is used conservatively for short-term gyro prediction, visual-IMU consistency checks, and short visual-failure recovery.

## Highlights

- ROS Noetic catkin workspace layout
- Stereo image synchronization with `message_filters`
- GFTT feature detection and LK optical-flow tracking
- Stereo triangulation from calibrated left/right camera models
- Keyframe-to-current-frame PnP/RANSAC pose estimation
- Quality-aware feature scoring and top-k PnP input selection
- Lightweight IMU gyro prediction from `/djiros/imu`
- Runtime VO confidence output for failure diagnosis
- RViz-compatible odometry, path, camera pose, and point-cloud topics

## Repository Layout

```text
.
├── README.md
├── RUN_GUIDE.md
├── LICENSE
├── camera_models/              # camodocal-style camera models and calibration utilities
└── stereo_vo_estimator/         # ROS package, package name: stereo_vo
    ├── config/realsense_1/      # camera calibration and runtime yaml
    ├── include/                 # estimator/node headers
    ├── launch/                  # rosbag playback launch file
    ├── msg/                     # relative pose message
    └── src/                     # node, estimator, parameter loading
```

## ROS Packages

```text
camera_models
stereo_vo
```

`stereo_vo_estimator/` builds as the ROS package `stereo_vo`.

## Main Files

```text
stereo_vo_estimator/src/stereo_vo_node.cpp       # ROS node, image/IMU subscriptions, publishers
stereo_vo_estimator/src/estimator.cpp            # VO pipeline, quality scoring, IMU assistance
stereo_vo_estimator/src/parameters.cpp           # YAML parameter loading
stereo_vo_estimator/include/estimator.h          # estimator state and algorithm interfaces
stereo_vo_estimator/include/stereo_vo.h          # ROS node wrapper
stereo_vo_estimator/config/realsense_1/*.yaml    # camera calibration and runtime settings
stereo_vo_estimator/launch/stereo_vo_bag.launch  # default rosbag playback launch
```

## Inputs

Default topics are configured in `stereo_vo_estimator/config/realsense_1/realsense_n3_unsync.yaml`:

```text
/camera/infra1/image_rect_raw
/camera/infra2/image_rect_raw
/djiros/imu
```

The IMU topic is optional at runtime when `use_imu: 0`.

## Outputs

```text
/stereo_vo/Odometry
/stereo_vo/Camera_pose
/stereo_vo/Path
/stereo_vo/PointCloud
/stereo_vo/Relative_pose
/stereo_vo/Relative_pose_vis
/stereo_vo/vo_confidence
/stereo_vo/frame_quality_text
```

## Requirements

Recommended environment:

```text
Ubuntu 20.04
ROS Noetic
GCC 9
OpenCV 4.2
Eigen 3
PCL 1.10
Ceres 1.14
RViz
```

Install common dependencies:

```bash
sudo apt update
sudo apt install -y ros-noetic-desktop-full
sudo apt install -y python3-rosdep python3-catkin-tools
sudo apt install -y build-essential cmake libeigen3-dev libopencv-dev libpcl-dev libceres-dev libdw-dev
```

## Build

Build inside a Linux filesystem path. Avoid compiling directly under mounted Windows paths that contain spaces or non-ASCII characters.

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cp -r /path/to/P2P2/camera_models ~/catkin_ws/src/
cp -r /path/to/P2P2/stereo_vo_estimator ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Expected result:

```text
[100%] Built target stereo_vo
```

## Run

This repository does not track large rosbag files. Put a compatible ROS1 bag at:

```text
~/catkin_ws/src/stereo_vo_estimator/bag/realsense_1.bag
```

Run the node and bag playback:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch stereo_vo stereo_vo_bag.launch
```

Run RViz in another terminal:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rviz -d ~/catkin_ws/src/stereo_vo_estimator/config/stereo.rviz
```

Check quality outputs:

```bash
rostopic echo -n 1 /stereo_vo/vo_confidence
rostopic echo -n 1 /stereo_vo/frame_quality_text
```

## Configuration

Runtime settings are in:

```text
stereo_vo_estimator/config/realsense_1/realsense_n3_unsync.yaml
```

Important switches:

```yaml
use_quality_aware: 1
use_quality_filtering: 1
use_weighted_pnp: 1
use_imu: 1
```

To restore the stereo-only baseline:

```yaml
use_quality_aware: 0
use_quality_filtering: 0
use_weighted_pnp: 0
use_imu: 0
```

## Algorithm Overview

```text
stereo image sync
  -> feature detection
  -> stereo LK matching
  -> triangulation
  -> keyframe-to-current-frame LK tracking
  -> quality-aware filtering
  -> PnP/RANSAC pose estimation
  -> IMU consistency diagnosis
  -> pose/path/pointcloud/confidence publishing
```

## Limitations

- No loop closure or global map optimization
- No tightly coupled IMU preintegration
- No IMU bias estimation or gravity alignment
- No sliding-window bundle adjustment
- No dense mapping
- No dynamic-object segmentation

## Data Policy

Large datasets, rosbag files, generated reports, and slide decks are intentionally excluded from the source repository. Publish them separately as release assets if reproducibility or demonstration material is required.

## License

The `stereo_vo` package is MIT licensed. The `camera_models/` package is based on camodocal-style third-party code and may be subject to separate upstream licensing terms. Review upstream licenses before redistributing modified versions.
