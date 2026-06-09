# Ubuntu 20.04 + ROS Noetic Runbook

This runbook is for the Ubuntu runtime machine. Do not run ROS/Gazebo from the Windows editing environment.

## 1. Install System Dependencies

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git wget curl \
  python3-pip python3-rosdep python3-catkin-tools python3-vcstool \
  libopencv-dev libeigen3-dev libboost-all-dev libsuitesparse-dev \
  ros-noetic-desktop-full \
  ros-noetic-navigation ros-noetic-move-base ros-noetic-dwa-local-planner \
  ros-noetic-octomap ros-noetic-octomap-server ros-noetic-octomap-ros ros-noetic-octomap-rviz-plugins \
  ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs ros-noetic-message-filters \
  ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-image-geometry \
  ros-noetic-gazebo-ros ros-noetic-gazebo-plugins ros-noetic-robot-state-publisher \
  ros-noetic-xacro ros-noetic-rviz
```

```bash
sudo rosdep init || true
rosdep update
```

## 2. Clone and Build

```bash
mkdir -p ~/catkin_hnbv/src
cd ~/catkin_hnbv/src
git clone https://github.com/xgmszlx/NBV-slam-.git hnbv_active_slam
vcs import . < hnbv_active_slam/third_party.repos

cd ~/catkin_hnbv
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install --user -r src/hnbv_active_slam/requirements.txt

source /opt/ros/noetic/setup.bash
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash
```

## 3. Prepare Models

```bash
cd ~/catkin_hnbv/src/hnbv_active_slam
mkdir -p models/yolo models/orb
python3 - <<'PY'
from ultralytics import YOLO
YOLO("yolov8s-seg.pt")
PY
```

Copy ORB-SLAM2 vocabulary to:

```text
~/catkin_hnbv/src/hnbv_active_slam/models/orb/ORBvoc.txt
```

## 4. Launch Env 1

Terminal 1:

```bash
source ~/catkin_hnbv/devel/setup.bash
roslaunch hnbv_bringup sim_env1.launch
```

Terminal 2:

```bash
source ~/catkin_hnbv/devel/setup.bash
roslaunch hnbv_bringup hnbv_system.launch env:=env1 mode:=full_hierarchical
```

Terminal 3:

```bash
source ~/catkin_hnbv/devel/setup.bash
rosservice call /hnbv/start "{}"
```

## 5. Launch Env 2

Terminal 1:

```bash
source ~/catkin_hnbv/devel/setup.bash
roslaunch hnbv_bringup sim_env2.launch
```

Terminal 2:

```bash
source ~/catkin_hnbv/devel/setup.bash
roslaunch hnbv_bringup hnbv_system.launch env:=env2 mode:=full_hierarchical
```

Terminal 3:

```bash
source ~/catkin_hnbv/devel/setup.bash
rosservice call /hnbv/start "{}"
```

## 6. Record a Run

```bash
mkdir -p ~/hnbv_runs/env1_full/trial_001
rosbag record -O ~/hnbv_runs/env1_full/trial_001/run.bag \
  /tf /tf_static \
  /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /camera/camera_info \
  /hnbv/semantics/tracks /hnbv/semantics/instance_mask \
  /hnbv/slam/pose /hnbv/slam/status /hnbv/slam/features \
  /hnbv/map/occupancy /hnbv/planner/global_goal /hnbv/planner/local_cmd_vel \
  /cmd_vel /gazebo/model_states
```

## 7. Compute Metrics and Figures

```bash
python3 ~/catkin_hnbv/src/hnbv_active_slam/hnbv_eval/scripts/compute_metrics.py \
  --run_dir ~/hnbv_runs/env1/full_hierarchical/trial_001

python3 ~/catkin_hnbv/src/hnbv_active_slam/hnbv_eval/scripts/aggregate_trials.py \
  --root ~/hnbv_runs --env env1 --modes global_only full_hierarchical

python3 ~/catkin_hnbv/src/hnbv_active_slam/hnbv_eval/scripts/make_figures.py \
  --root ~/hnbv_runs --env env1 --out ~/hnbv_runs/env1/figures

python3 ~/catkin_hnbv/src/hnbv_active_slam/hnbv_eval/scripts/make_report.py \
  --root ~/hnbv_runs --env env1 --out ~/hnbv_runs/env1/aggregate_report.md
```

## 8. Current Implementation Status

The first committed version is a ROS/catkin scaffold with stable interfaces and offline metric utilities. The remaining implementation order is:

1. Replace `slam_state_publisher` scaffold with ORB-SLAM2 RGB-D wrapper.
2. Replace semantic tracker scaffold with YOLOv8s-seg + BoT-SORT inference.
3. Implement dynamic feature masking and masked point cloud generation.
4. Implement full global NBV frontier/DBSCAN/raycasting/entropy scoring.
5. Implement local FPM prediction and DWA candidate scoring.
6. Add deterministic robot and walking-person Gazebo models.
7. Run multi-trial experiments and generate reports.

