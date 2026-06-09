# Mutual Information Hierarchical NBV Active Semantic Visual SLAM 复现项目

本仓库用于复现论文 **Mutual information-based hierarchical NBV decision for active semantic visual SLAM under dynamic environments** 中提出的动态环境主动语义视觉 SLAM 方法。

论文核心思想是：在 ORB-SLAM2 RGB-D 视觉 SLAM 基础上，引入语义动态目标剔除、全局 mutual information NBV 探索、局部 Feature Probability Map（FPM）和 DWA 可达视角选择，使机器人在存在行人等动态目标的场景中仍能稳定定位、高效建图并主动避开动态遮挡。

> 重要说明：当前 Windows 环境只用于代码编写、文档维护和 Git 提交，不作为 ROS/Gazebo 运行环境。实际运行目标是 **Ubuntu 20.04 + ROS Noetic + Gazebo**。

## 当前状态

当前提交是项目第一版工程骨架，已经包含：

- ROS Noetic / catkin 多包工程结构；
- 自定义消息包 `hnbv_msgs`；
- SLAM、语义跟踪、建图、全局 NBV、局部 NBV、仿真、启动和评估模块入口；
- Ubuntu 20.04 运行说明；
- 论文复现技术实施方案；
- 离线评估指标的纯 Python 基础实现和测试样例；
- Gazebo world 占位、launch 文件和参数配置。

当前提交尚未完成完整论文算法实现，以下模块仍是 scaffold / integration point：

- ORB-SLAM2 RGB-D wrapper 与动态特征 mask 深度改造；
- YOLOv8s-seg + BoT-SORT 真实推理与跟踪；
- masked RGB-D point cloud 回投与 OctoMap 投影；
- frontier + DBSCAN + ray casting + entropy 的完整全局 NBV；
- FPM 构建、下一视角预测和 DWA 局部 NBV；
- 确定性行人轨迹和完整差速机器人模型。

后续应按 `docs/reproduction_implementation_plan.md` 中的 M1-M6 里程碑逐步补完。

## 仓库结构

```text
.
├── README.md
├── requirements.txt
├── third_party.repos
├── docs/
│   ├── reproduction_implementation_plan.md
│   └── ubuntu20_ros_noetic_runbook.md
├── assets/
│   └── 论文转换图片
├── hnbv_msgs/
│   └── 自定义 ROS 消息
├── hnbv_slam/
│   └── ORB-SLAM2 RGB-D wrapper 与动态特征剔除入口
├── hnbv_semantics/
│   └── YOLOv8s-seg + BoT-SORT 语义跟踪节点入口
├── hnbv_mapping/
│   └── masked point cloud 与 occupancy projection 节点入口
├── hnbv_planner/
│   └── 全局 NBV 与局部 FPM/DWA NBV 节点入口
├── hnbv_sim/
│   └── Gazebo world、模型和仿真启动文件
├── hnbv_bringup/
│   └── 系统 launch、参数配置和探索 start/stop 服务
└── hnbv_eval/
    └── 离线指标计算、聚合、图表和报告模块
```

## 模块说明

### `hnbv_msgs`

定义系统内部通信消息：

- `TrackedObject.msg`：动态目标 track id、类别、bbox、像素速度、运动方向、动态确认状态；
- `TrackedObjectArray.msg`：一帧内所有跟踪目标；
- `FeaturePointArray.msg`：SLAM 前端输出的 ORB 特征点；
- `PlannerDebug.msg`：NBV 候选、得分和选中目标调试信息。

### `hnbv_slam`

目标是封装 ORB-SLAM2 RGB-D，并增加动态 mask 接口：

1. 同步 RGB、Depth、CameraInfo 和 instance mask；
2. 删除动态目标 mask 内的 ORB keypoints/descriptors；
3. 发布 `/hnbv/slam/pose`、`/hnbv/slam/features`、`/hnbv/slam/status`；
4. 为评估模块输出 tracking loss 和特征数量。

当前节点 `slam_state_publisher` 是占位节点，用于固定 ROS 话题接口。

### `hnbv_semantics`

目标是封装 YOLOv8s-seg + BoT-SORT：

1. 使用 COCO 预训练 `yolov8s-seg.pt`；
2. 实验默认只将 `person` 类作为潜在动态目标；
3. 输出 `/hnbv/semantics/instance_mask` 和 `/hnbv/semantics/tracks`；
4. 后续加入对极几何验证，确认目标是否真的动态。

当前 `semantic_tracker_node.py` 是占位节点，后续替换为真实推理。

### `hnbv_mapping`

目标是从 RGB-D 和 SLAM pose 构建动态剔除后的地图：

1. 动态 mask 内深度点不进入 point cloud；
2. 深度最大有效距离为 `5.0 m`；
3. 使用 OctoMap 建 3D occupancy；
4. 投影为 `0.05 m/cell` 的 2D `nav_msgs/OccupancyGrid`。

当前 `occupancy_projection_node` 发布空白 unknown grid，用于调通接口。

### `hnbv_planner`

目标是实现论文的分层 NBV：

全局 NBV：

- frontier 提取；
- DBSCAN 聚类；
- 8 个 yaw 方向候选；
- `90 deg` FOV、`5.0 m` range 的 ray casting；
- Shannon entropy / mutual information 评分；
- 移动距离和转角代价；
- 输出 `/hnbv/planner/global_goal`。

局部 NBV：

- 从当前 ORB 特征构建 `M_f`；
- 从动态目标速度和 mask 构建各向异性高斯动态图 `M_t`；
- 融合得到 FPM；
- 预测候选下一视角 FPM；
- 通过 DWA 采样可执行速度；
- 输出 `/hnbv/planner/local_cmd_vel`。

当前节点是可编译的接口占位。

### `hnbv_eval`

用于离线评估主动 SLAM 结果，目标是从 rosbag 和导出文件中生成全面指标与图表。

已包含基础函数：

- travel distance；
- occupancy entropy；
- entropy reduction rate；
- coverage ratio；
- near collision event；
- 多 trial 均值、标准差和 95% CI。

规划中的完整指标体系包括：

- 探索效率：Travel Time、Travel Distance、Coverage、Exploration Rate、Path Efficiency；
- 建图质量：Entropy Reduction、ERR、Occupancy IoU、Unknown Ratio、Dynamic Residual；
- 定位鲁棒性：ATE、RPE、Tracking Loss Rate、Relocalization Time、Feature Count；
- 动态安全性：Near Collision Count、Minimum Dynamic Distance、Dynamic In-FOV Ratio；
- 局部 NBV 解释性：FPM Entropy、Local MI、Override Duration、Feature Recovery；
- 实时性：SLAM FPS、YOLO FPS、module runtime mean/P95/max、end-to-end latency。

## Ubuntu 20.04 部署

以下命令只在 Ubuntu 20.04 运行。

### 1. 安装系统依赖

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

初始化 rosdep：

```bash
sudo rosdep init || true
rosdep update
```

### 2. 拉取仓库和第三方代码

```bash
mkdir -p ~/catkin_hnbv/src
cd ~/catkin_hnbv/src
git clone https://github.com/xgmszlx/NBV-slam-.git hnbv_active_slam
vcs import . < hnbv_active_slam/third_party.repos
```

`third_party.repos` 当前会拉取：

- ORB-SLAM2；
- TARE planner；
- RNEX/RNE exploration。

TARE 和 RNE 主要用于后续基线对比，不是主系统运行的必要核心。

### 3. 安装 ROS 和 Python 依赖

```bash
cd ~/catkin_hnbv
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install --user -r src/hnbv_active_slam/requirements.txt
```

### 4. 构建 catkin workspace

```bash
source /opt/ros/noetic/setup.bash
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash
```

如果 ORB-SLAM2 在 Ubuntu 20.04 上出现 OpenCV4、Pangolin 或 C++ 标准相关错误，建议先单独编译 ORB-SLAM2，确认其 RGB-D 示例能跑通，再接入本项目 wrapper。

## 模型和权重准备

### YOLOv8s-seg

```bash
cd ~/catkin_hnbv/src/hnbv_active_slam
mkdir -p models/yolo models/orb
python3 - <<'PY'
from ultralytics import YOLO
YOLO("yolov8s-seg.pt")
PY
```

### ORB-SLAM2 词袋

将 ORB-SLAM2 的 `ORBvoc.txt` 放到：

```text
~/catkin_hnbv/src/hnbv_active_slam/models/orb/ORBvoc.txt
```

权重和词袋文件较大，默认不提交到 Git。

## 启动仿真

### Env 1：10 m x 10 m 开阔环境

终端 1：

```bash
source ~/catkin_hnbv/devel/setup.bash
roslaunch hnbv_bringup sim_env1.launch
```

终端 2：

```bash
source ~/catkin_hnbv/devel/setup.bash
roslaunch hnbv_bringup hnbv_system.launch env:=env1 mode:=full_hierarchical
```

终端 3：

```bash
source ~/catkin_hnbv/devel/setup.bash
rosservice call /hnbv/start "{}"
```

### Env 2：19 m x 22 m 复杂环境

终端 1：

```bash
source ~/catkin_hnbv/devel/setup.bash
roslaunch hnbv_bringup sim_env2.launch
```

终端 2：

```bash
source ~/catkin_hnbv/devel/setup.bash
roslaunch hnbv_bringup hnbv_system.launch env:=env2 mode:=full_hierarchical
```

终端 3：

```bash
source ~/catkin_hnbv/devel/setup.bash
rosservice call /hnbv/start "{}"
```

### 消融模式

只启用全局 NBV：

```bash
roslaunch hnbv_bringup hnbv_system.launch env:=env1 mode:=global_only
```

启用完整分层 NBV：

```bash
roslaunch hnbv_bringup hnbv_system.launch env:=env1 mode:=full_hierarchical
```

## 记录 rosbag

```bash
mkdir -p ~/hnbv_runs/env1/full_hierarchical/trial_001
rosbag record -O ~/hnbv_runs/env1/full_hierarchical/trial_001/run.bag \
  /tf /tf_static \
  /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /camera/camera_info \
  /hnbv/semantics/tracks /hnbv/semantics/instance_mask \
  /hnbv/slam/pose /hnbv/slam/status /hnbv/slam/features \
  /hnbv/map/occupancy /hnbv/planner/global_goal /hnbv/planner/local_cmd_vel \
  /cmd_vel /gazebo/model_states
```

每个 trial 建议保留：

```text
trial_001/
├── run.bag
├── manifest.yaml
├── trajectories/
├── maps/
├── planner/
├── metrics.json
├── figures/
└── report.md
```

## 离线评估和出图

单次 trial 指标：

```bash
python3 ~/catkin_hnbv/src/hnbv_active_slam/hnbv_eval/scripts/compute_metrics.py \
  --run_dir ~/hnbv_runs/env1/full_hierarchical/trial_001
```

多 trial 聚合：

```bash
python3 ~/catkin_hnbv/src/hnbv_active_slam/hnbv_eval/scripts/aggregate_trials.py \
  --root ~/hnbv_runs --env env1 --modes global_only full_hierarchical
```

生成图表：

```bash
python3 ~/catkin_hnbv/src/hnbv_active_slam/hnbv_eval/scripts/make_figures.py \
  --root ~/hnbv_runs --env env1 --out ~/hnbv_runs/env1/figures
```

生成报告：

```bash
python3 ~/catkin_hnbv/src/hnbv_active_slam/hnbv_eval/scripts/make_report.py \
  --root ~/hnbv_runs --env env1 --out ~/hnbv_runs/env1/aggregate_report.md
```

导出论文风格表格：

```bash
python3 ~/catkin_hnbv/src/hnbv_active_slam/hnbv_eval/scripts/export_tables.py \
  --run_dir ~/hnbv_runs/env1
```

## 复现实验建议

第一阶段先跑两个方法：

| 环境 | 方法 |
| --- | --- |
| Env 1 | `global_only` |
| Env 1 | `full_hierarchical` |
| Env 2 | `global_only` |
| Env 2 | `full_hierarchical` |

第二阶段再补基线：

| 环境 | 方法 |
| --- | --- |
| Env 1 / Env 2 | nearest frontier |
| Env 1 / Env 2 | RNE / RNEX |
| Env 1 / Env 2 | TARE |

每个方法每个环境建议至少跑 5 次 trial，并输出 mean、std、95% confidence interval 和相对改进比例。

## 主要评估指标

论文主指标：

- Travel Distance；
- Travel Time；
- ATE；
- RPE Trans.；
- RPE Rot.；
- ERR；
- Tracking Loss Rate；
- Near Collision Count。

主动 SLAM 扩展指标：

- Coverage Ratio；
- Time to Coverage；
- Distance to Coverage；
- Exploration Rate；
- Path Efficiency；
- Redundant Revisiting Ratio；
- Entropy per Meter；
- Gain Prediction Error。

动态环境指标：

- Minimum Dynamic Distance；
- Time Near Dynamic Objects；
- Dynamic Object In-FOV Ratio；
- Dynamic Mask Area Ratio；
- Dynamic Feature Rejection Rate；
- Static Feature Retention；
- Feature Recovery After Avoidance。

实时性指标：

- SLAM FPS；
- YOLO FPS；
- module runtime mean / P95 / max；
- end-to-end latency；
- CPU / GPU usage；
- dropped frame ratio。

## 可视化结果

目标输出图表包括：

- `trajectory_overlay.png`：估计轨迹、ground truth、NBV goal 和 near collision 标记；
- `final_map_overlay.png`：最终 occupancy map、轨迹和动态残留；
- `coverage_over_time.png`：覆盖率随时间变化；
- `entropy_over_time.png`：地图熵随时间或距离下降；
- `err_bar_ci.png`：ERR 均值和置信区间；
- `slam_error_boxplot.png`：ATE/RPE 分布；
- `tracking_state_timeline.png`：OK/LOST 状态时间轴；
- `dynamic_distance_timeline.png`：机器人到动态目标的最小距离；
- `fpm_snapshot_grid.png`：RGB、mask、feature map、dynamic map、FPM、local view；
- `global_nbv_candidates.png`：frontier candidates 和 NBV score；
- `runtime_breakdown.png`：各模块耗时对比。

## 开发路线

建议按以下顺序继续实现：

1. 在 Ubuntu 20.04 上确认当前骨架 `catkin build` 通过；
2. 接入 ORB-SLAM2 RGB-D，替换 `slam_state_publisher`；
3. 接入 YOLOv8s-seg + BoT-SORT，替换语义跟踪 scaffold；
4. 实现动态 mask 下的 ORB 特征剔除；
5. 实现 RGB-D masked point cloud 和 OctoMap / 2D occupancy 投影；
6. 实现全局 NBV 的 frontier、DBSCAN、ray casting、entropy score；
7. 实现局部 FPM、动态目标预测和 DWA candidate scoring；
8. 完善 Gazebo robot、RGB-D camera 和 walking person actor；
9. 完善 rosbag extractor、指标计算、图表和报告；
10. 跑消融实验和基线对比。

## 文档

详细复现方案：

```text
docs/reproduction_implementation_plan.md
```

Ubuntu 运行手册：

```text
docs/ubuntu20_ros_noetic_runbook.md
```

## 注意事项

- 不要在 Windows 环境运行 ROS、Gazebo、catkin build 或 CUDA 推理；
- 本项目运行目标是 Ubuntu 20.04 + ROS Noetic；
- ORB-SLAM2 为 GPL 许可，后续若深度集成或分发派生代码，需要保持许可证兼容；
- YOLO 权重、ORB-SLAM2 词袋和大 rosbag 不应提交到 Git；
- 当前 world 和节点是工程骨架，正式实验前必须补齐机器人模型、行人轨迹和真实算法实现；
- 所有实验结果必须记录 manifest，包括 git commit、配置 hash、模型 hash、world、随机种子和终止原因。

