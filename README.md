# Hierarchical NBV Active Semantic Visual SLAM

This repository scaffolds a ROS Noetic reproduction of **Mutual information-based hierarchical NBV decision for active semantic visual SLAM under dynamic environments**.

Current state:

- Windows is used only for editing and version control.
- Runtime target is Ubuntu 20.04 with ROS Noetic and Gazebo.
- The project is organized as a catkin source repository containing ROS packages for messages, SLAM wrapping, semantics, mapping, planning, simulation, bringup, and evaluation.
- The implementation plan is in `docs/reproduction_implementation_plan.md`.

## Package Layout

```text
hnbv_msgs        Custom ROS messages.
hnbv_slam        ORB-SLAM2 RGB-D wrapper and dynamic feature-mask integration points.
hnbv_semantics   YOLOv8s-seg + BoT-SORT semantic tracking node.
hnbv_mapping     Dynamic-mask-aware point cloud and occupancy projection nodes.
hnbv_planner     Global mutual-information NBV and local FPM/DWA NBV planners.
hnbv_sim         Gazebo worlds, robot model, and walking-person placeholders.
hnbv_bringup     Launch files and shared configs.
hnbv_eval        Offline metrics, aggregation, figures, and reports.
```

## Ubuntu 20.04 Quick Start

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git python3-pip python3-rosdep python3-catkin-tools python3-vcstool \
  ros-noetic-desktop-full ros-noetic-navigation ros-noetic-move-base ros-noetic-dwa-local-planner \
  ros-noetic-octomap ros-noetic-octomap-server ros-noetic-cv-bridge ros-noetic-image-transport \
  ros-noetic-gazebo-ros ros-noetic-gazebo-plugins ros-noetic-xacro ros-noetic-rviz

sudo rosdep init || true
rosdep update

mkdir -p ~/catkin_hnbv/src
cd ~/catkin_hnbv/src
git clone https://github.com/xgmszlx/NBV-slam-.git hnbv_active_slam
cd ~/catkin_hnbv/src
vcs import . < hnbv_active_slam/third_party.repos

cd ~/catkin_hnbv
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install --user -r src/hnbv_active_slam/requirements.txt

source /opt/ros/noetic/setup.bash
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash
```

## Run in Gazebo

Env 1:

```bash
source ~/catkin_hnbv/devel/setup.bash
roslaunch hnbv_bringup sim_env1.launch
```

In a second terminal:

```bash
source ~/catkin_hnbv/devel/setup.bash
roslaunch hnbv_bringup hnbv_system.launch env:=env1 mode:=full_hierarchical
rosservice call /hnbv/start "{}"
```

Env 2:

```bash
source ~/catkin_hnbv/devel/setup.bash
roslaunch hnbv_bringup sim_env2.launch
```

In a second terminal:

```bash
source ~/catkin_hnbv/devel/setup.bash
roslaunch hnbv_bringup hnbv_system.launch env:=env2 mode:=full_hierarchical
rosservice call /hnbv/start "{}"
```

## Evaluate Runs

```bash
python3 src/hnbv_active_slam/hnbv_eval/scripts/run_experiment.py \
  --env env1 --mode full_hierarchical --trials 5 --record-bag

python3 src/hnbv_active_slam/hnbv_eval/scripts/compute_metrics.py \
  --run_dir ~/hnbv_runs/env1/full_hierarchical/trial_001

python3 src/hnbv_active_slam/hnbv_eval/scripts/aggregate_trials.py \
  --root ~/hnbv_runs --env env1 --modes global_only full_hierarchical

python3 src/hnbv_active_slam/hnbv_eval/scripts/make_figures.py \
  --root ~/hnbv_runs --env env1 --out ~/hnbv_runs/env1/figures
```

## Notes

- ORB-SLAM2 vocabulary and YOLO weights are not committed. Place `ORBvoc.txt` under `models/orb/` and download `yolov8s-seg.pt` through Ultralytics on the Ubuntu machine.
- The current ROS nodes are scaffolding and integration points. The detailed task order is documented in `docs/reproduction_implementation_plan.md`.
- Do not use Windows as the runtime target for ROS/Gazebo.

