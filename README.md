<p align="center">
  <img width=400 src="doc/bievr_final.svg">
</p>


# BIEVR-LIO: Robust LiDAR-Inertial Odometry through Bump-Image-Enhanced Voxel Maps

<p align="center">
<a href="https://patripfr.github.io/bievr-lio/"><img src="https://shieldcn.dev/badge/Project-Page-gray?size=xs" alt="Project Page" /></a>
<a href="https://arxiv.org/abs/2604.14421"><img src="https://shieldcn.dev/badge/arXiv-2604.14421-b31b1b?logo=arxiv&size=xs" alt="arXiv" /></a>
<a href="https://arxiv.org/pdf/2604.14421"><img src="https://shieldcn.dev/badge/Paper-PDF-black?size=xs" alt="Paper PDF" /></a>
<a href="LICENSE"><img src="https://shieldcn.dev/badge/License-BSD--3--Clause-green?size=xs" alt="License: BSD-3-Clause" /></a>
<a href="https://youtu.be/TsDJOdthhNk"><img src="https://shieldcn.dev/badge/YouTube-red?logo=youtube&size=xs" alt="YouTube" /></a>
</p>

<p align="center">
<a href="https://github.com/ethz-asl/BIEVR-LIO/actions/workflows/build_20_04.yaml"><img src="https://shieldcn.dev/github/ethz-asl/BIEVR-LIO/ci.svg?workflow=build_20_04.yaml&label=ROS1%20Noetic&size=xs&variant=outline&mode=light" alt="Ubuntu 20.04 + ROS Noetic Build" /></a>
</p>

<p align="center">
  <img width='100%' src="doc/tunnel_detail.png">
</p>

BIEVR-LIO is a robust LiDAR-Inertial Odometry framework that uses a high-resolution,
voxel-wise oriented height image map to exploit subtle geometric variations in
challenging, information-sparse environments.

<details>
<summary><b>Abstract</b></summary>
<br>
Reliable odometry is essential for mobile robots as they increasingly enter more challenging environments, which often contain little information to constrain point cloud registration, resulting in degraded LiDAR–Inertial Odometry (LIO) accuracy or even divergence. To address this, we present BIEVR-LIO, a novel approach designed specifically to exploit subtle variations in the available geometry for improved robustness. We propose a high-resolution map representation that stores surfaces as voxel-wise oriented height images. This representation can directly be used for registration without the calculation of intermediate geometric primitives while still supporting efficient updates. Since informative geometry is often sparsely distributed in the environment, we further propose a map-informed point sampling strategy to focus registration on geometrically informative regions, improving robustness in uninformative environments while reducing computational cost compared to global high-resolution sampling. Experiments across multiple sensors, platforms, and environments demonstrate state-of-the-art performance in well-constrained scenes and substantial improvements in challenging scenarios where baseline methods diverge. Additionally, we demonstrate that the fine-grained geometry captured by BIEVR-LIO can be used for downstream tasks such as elevation mapping for robot locomotion.
</details>

# Setup

The core estimator (`bievr_lio`) is a self-contained, ROS-independent library.
This repository provides a **ROS1 Noetic** interface (`bievr_lio_ros`) under
`interfaces/ros1`.

## Installation

### Dependencies

BIEVR-LIO is intentionally light on dependencies: the core estimator only needs
**[Eigen](https://eigen.tuxfamily.org)** and **[Ceres](http://ceres-solver.org)**.


### Build

Requires [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) and
`python3-catkin-tools` (`sudo apt install python3-catkin-tools`).

Create a catkin workspace and clone BIEVR-LIO into it:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel

cd ~/catkin_ws/src
git clone git@github.com:ethz-asl/BIEVR-LIO.git BIEVR-LIO
```

Make sure a compatible Ceres installation is available to CMake before building.

(Optional) **Livox support.** The Livox `CustomMsg` branches are only compiled if
the corresponding driver is found in the workspace at build time. Otherwise
BIEVR-LIO builds fine without them. If you need to process Livox data, clone and
build the matching driver into `~/catkin_ws/src` *before* building BIEVR-LIO
(each driver also needs its Livox-SDK installed system-wide):

- Livox gen1 (`livox_ros_driver`, enables `BIEVR_WITH_LIVOX`):
  [livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver) +
  [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK)
- Livox gen2 (`livox_ros_driver2`, enables `BIEVR_WITH_LIVOX2`):
  [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) +
  [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)

Build and source it:

```bash
cd ~/catkin_ws
catkin build bievr_lio_ros
source devel/setup.bash
```

## Run data

BIEVR-LIO provides two ROS1 entry points:

- **`process_topics`** runs online: it subscribes to the LiDAR and IMU topics and
  processes messages as they arrive. Use it with a live sensor or alongside
  `rosbag play`.
- **`process_bag`** reads a recorded bag directly and pushes its messages through
  the pipeline as fast as they can be processed (no real-time playback). This is the preferred choice for offline evaluation and reproducing results.

同一次采集被拆成多个 bag 时，可重复传入 `--bag`。程序只创建一次 LIO、关键帧
提取器和定位器，按命令行顺序连续处理，并在全部 bag 结束后统一保存；不要分别启动
三个进程，否则 IMU 状态、里程计和关键帧窗口会在边界处重置：

```bash
rosrun bievr_lio_ros process_bag --sensor_config_file config/sensor_configs/xiangyang.yaml \
  --params_file config/params.yaml \
  --bag /data/bag2.bag --bag /data/bag3.bag --bag /data/bag4.bag
```

在线连续播放多个 bag 时，保持同一个前端和 GNSS 记录节点运行，共用一个 ENU 原点及
点云归档。`gnss_optimize` 只读取已保存的 GNSS 文本、LIO 轨迹和点云归档，不读取 bag。

### Quick start（中文）

每次打开新终端后，先加载 ROS 和工作空间环境（请将路径替换为自己的
catkin 工作空间）：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

如果刚修改过代码，需要先重新编译：

```bash
cd ~/catkin_ws
catkin build bievr_lio_ros
source devel/setup.bash
```

建图时，先编辑 `interfaces/ros1/launch/mapping.launch` 中
`process_topics` 节点的 YAML 路径。默认使用 `$(find bievr_lio_ros)`
定位 ROS 包，因此更换工作空间时无需修改绝对路径：

```xml
args="--sensor_config_file $(find bievr_lio_ros)/../../config/sensor_configs/xiangyang.yaml --params_file $(find bievr_lio_ros)/../../config/params.yaml"
```

启动时不需要传入 YAML 或 bag 参数。终端 1 启动建图节点
（`roslaunch` 会在需要时自动启动 ROS master）：

```bash
roslaunch bievr_lio_ros mapping.launch
```

终端 2 播放 bag：

```bash
rosbag play /absolute/path/to/data.bag
```

这两个终端都需要先 source ROS 和工作空间。`mapping.launch` 默认启动
RViz；不需要时可使用 `roslaunch bievr_lio_ros mapping.launch rviz:=false`。
如果 bag 中记录了 `/clock` 且系统使用仿真时间，播放时添加 `--clock`。

等 `rosbag play` 播放完成后，在终端 1 按 `Ctrl+C` 正常结束建图节点；
请勿使用 `kill -9`。直接读取 bag 的 `process_bag` 模式则会在处理完成后自动结束。
结束时程序会等待尚未累计完成的点云，并创建 `Log` 目录，输出：

| File | Description |
|------|-------------|
| `Log/map.pcd` | 体素降采样后的累计点云地图，包含 `x y z intensity`；每个体素保留首个点及其强度 |
| `Log/gnss_optimization/trajectory.obj` | GNSS 优化后的 ENU 轨迹，格式同 `Log/trajectory.obj` |
| `Log/trajectory.obj` | OBJ 折线轨迹，可由 MeshLab、Blender 等软件打开 |
| `Log/odometry_tum.txt` | TUM 格式里程计，字段为 `timestamp tx ty tz qx qy qz qw` |
| `Log/localization_tum.txt` | Localization 模式输出的地图坐标系定位轨迹；只有成功定位后才生成 |
| `Log/poles/landmarks.txt` | 去重后的支柱坐标、柱轴、观测次数、拟合半径和 RMS |
| `Log/poles/extraction_config.yaml` | 提取参数快照，GNSS 优化时用于自动重建 |
| `Log/poles/gnss_optimization.yaml` | 自动重建使用的轨迹、点云归档和原结果备份路径 |
| `Log/poles/detections.txt` | 每次检测的柱轴、半径、RMS、内点率、角度覆盖率等拟合质量指标 |
| `Log/poles/landmarks.pcd` | 去重后的支柱中心点云，`intensity` 表示观测次数 |
| `Log/poles/rail_vector_map.txt` | 左右轨顶三维控制点、线路切向、轨距和提取质量 |
| `Log/poles/poles.pcd` | 所有关键帧通过细过滤的支柱点，结束时汇总保存，由 `save_keyframe_clouds` 控制 |
| `Log/poles/rails.pcd` | 所有关键帧中钢轨附近的实测激光点，保留输入分辨率；不插值、不作提取模块的体素降采样 |
| `Log/poles_localization/localization_corrections.txt` | 二次运行中通过验收的 `POLE/RAIL` 原始修正、LIO 累计里程、匹配数和 RMS |
| `Log/pole_localization/localization_tum.txt` | 不使用稠密地图时，由支柱修正与每帧 LIO 组合得到的完整轨迹 |
| `Log/pole_localization/filtered_corrections.txt` | 纯支柱定位每次滤波更新的原始/滤波修正、创新、马氏距离和协方差 |
| `Log/pole_localization/summary.yaml` | 稀疏地图定位状态、支柱/轨道接受数、轨迹帧数和末次创新 |

重定位运行仅保存定位模块的修正轨迹与统计，不再额外生成原始 LIO 的
`trajectory.obj` 和 `odometry_tum.txt`；这两个文件仍在建图时生成。

当前 `config/params.yaml` 给出了项目目录下的绝对路径默认值；使用覆盖参数时，
输出目录由最后一层 YAML 决定：

```bash
ls -lh /home/zsq/slam/bievr-slam/src/BIEVR-LIO/Log
```

保存点云要求 `config/params.yaml` 中的 `map_save.accumulate` 保持为
`True`。如果程序被 `kill -9` 强制终止，则退出保存逻辑无法执行。

### 铁路支柱提取

`config/params.yaml` 的 `pole_extraction` 段控制在线支柱提取。模块直接接收
SLAM 内部同一时刻的去畸变点云和位姿。行驶距离达到
`keyframe_distance_m` 且至少积累 `min_frames_per_keyframe` 帧时生成支柱关键帧；
低速或停车时由 `frames_per_keyframe` 限制最大帧数，相邻关键帧复用
`keyframe_overlap_frames` 帧以减少窗口边界漏检。后台提取先把体素点投影到该
关键帧的轨迹折线，按局部切向建立铁路走廊 ROI，再执行二维栅格高度连续性、
形态学连通域过滤和欧式聚类。闭运算补出的栅格只用于连通判断，不会把桥接
栅格中的杂点重新带入三维候选。

粗筛后参考 SE-LIO，通过 PCA 估计柱轴，将候选点投影到柱轴法平面，再用
RANSAC 和最小二乘拟合截面圆；轴倾角、拟合半径、全点径向 RMS、内点率和
角度覆盖率共同决定候选是否保留。PCA 与圆拟合及其验收门限保持不变。单次
观测的支柱坐标是拟合柱轴与 `trajectory_z + center_z_offset_m` 高程平面的
交点；跨关键帧合并时使用拟合质量加权融合柱轴，最后再计算支柱基点，并限制
同一关键帧中的观测与地标一对一匹配。它不会通过 ROS 重新同步点云和里程计。

运行时可在 RViz 中订阅：

- `/bievr_lio/poles/centers`：当前支柱关键帧的中心；
- `/bievr_lio/poles/points`：当前关键帧通过细过滤的支柱点；
- `/bievr_lio/poles/map`：跨关键帧合并后的支柱中心地图。
- `/bievr_lio/poles/markers`：累计支柱的拟合柱体、柱轴、基点和文字 ID。
- `/bievr_lio/poles/prior_map`：二次定位所加载的历史支柱地图（洋红色）。
- `/bievr_lio/poles/correction`：支柱估计的原始 `map <- odom` 修正坐标轴。
- `/bievr_lio/poles/correction_filtered`：纯支柱定位滤波后的正式修正。

随包提供的 RViz 配置已经启用支柱显示：青色点是实际被提取的点，半透明绿色
圆柱是拟合模型，黄色箭头是柱轴，红色球是输出坐标，文字包含支柱 ID、观测
次数、半径和 RMS。在线运行或希望边处理边看时使用：

```bash
roslaunch bievr_lio_ros mapping.launch
```

离线 `process_bag` 通常很快结束，处理完成后推荐直接查看保存结果：

```bash
python3 scripts/visualize_poles.py
```

查看器会加载 `Log/map.pcd` 和 `Log/poles/`。也可以导出一个带颜色的点云，供
CloudCompare 或 MeshLab 打开：

```bash
python3 scripts/visualize_poles.py --no-map --no-gui \
  --export Log/poles/poles_visualization.ply \
  --overview Log/poles/poles_overview.png
```

襄阳铁路 bag 使用 `mapping.launch` 中已写入的 `xiangyang.yaml`。
先启动建图节点，再在另一终端播放数据：

```bash
roslaunch bievr_lio_ros mapping.launch

# 另一终端
rosbag play "/mnt/e/Data/襄阳铁路数据-第二次数据/2026-01-13_襄阳金鹰重工_焦柳线_郜营-余家湖/20260113/bag20260113_1.bag"
```

默认参数以约 10 Hz 的 Livox 数据为基准：正常行驶每 30 m 生成关键帧，至少
积累 20 帧；低速时最多积累 50 帧，并保留 5 帧重叠。其他帧率应相应修改
`min_frames_per_keyframe`、`frames_per_keyframe` 和 `keyframe_overlap_frames`；
提取跟不上时会丢弃最老的待处理支柱关键帧，但不会阻塞里程计，最终统计会
报告丢弃数。

论文面向树干使用的 `0.05 m` 拟合误差在铁路格构柱、方柱和局部遮挡条件下
偏严；襄阳数据默认使用 `cylinder_inlier_threshold_m: 0.10`、
`cylinder_max_rms_m: 0.20` 和 `cylinder_min_inlier_ratio: 0.35`。铁路刚性支柱
默认 `cylinder_max_piecewise_depth: 0`；只有需要拟合弯曲柱体时才建议设为
`1`。跨关键帧地图会在共同高度比较两条拟合轴线，并同时检查轴线距离、柱轴
夹角和半径差；匹配后按点数、内点率、角度覆盖率和拟合残差进行质量加权，
减少斜柱基点外推误差和相邻目标误合并。

将保存支柱用于第二次运行时，`PoleExtractor` 会对最近 7 个关键帧做质量加权
融合，以轴线、轴角和半径门限生成候选，再用 Hungarian 完成一对一关联和 Huber
`SE(2)` 优化。验收后的 `x/y/yaw` 修正直接进入纯支柱定位滤波器，不再传给
GICP。参数、初值要求和完整设计参见
[`doc/pole_landmark_localization.md`](doc/pole_landmark_localization.md)。离线验证器
`scripts/validate_pole_localization.py` 可对两次提取结果注入可控位置/航向误差。

在线播放 bag 并随时停止时，`mapping.launch` 和 `process_topics.launch` 默认同时启动
`gnss_recorder`，订阅 `/gnss` 并逐条刷新文件。先启动前端，再播放 bag：

```bash
roslaunch bievr_lio_ros mapping.launch
rosbag play --clock data.bag
# 停止播放，再 Ctrl+C 停止前端以保存 LIO 轨迹，然后优化：
rosrun bievr_lio_ros gnss_optimize --gnss_enu Log/gnss/gnss_enu.txt \
  --lio_trajectory Log/odometry_tum.txt --output_directory Log/gnss_optimization
```

如需 ROS 使用 bag 时间，启动前设置 `rosparam set /use_sim_time true`。
记录器使用 `NavSatFix.header.stamp`；前端与 GNSS 必须使用同一时间基准。
在 `config/params.yaml` 配置 GNSS 话题和节点开关：

```yaml
gnss_recorder:
  enable: True
  gnss_topic: "/gnss"
```

`process_topics.launch` 的自定义 `params` 文件也会传给记录节点。
关闭时设置 `enable: False`，节点直接退出，不创建记录文件。
其余参数使用默认值：输出到项目的 `Log/gnss`，已有文件不覆盖；再次运行前请先移走上次记录。
直接运行前端可另开终端启动记录器，读取同一份配置：

```bash
roslaunch bievr_lio_ros gnss_recorder.launch
```

使用 GeographicLib `LocalCartesian`（WGS84）转换，`/gnss` 或 `/gnss_pose` 首条有效位置为两者共用的固定 ENU 原点；
也可设置记录节点私有参数 `origin_lla: [纬度, 经度, 椭球高]`，让多次记录共用原点。
编译依赖 GeographicLib 开发库（Ubuntu：`sudo apt install libgeographiclib-dev`）。输出：

- `gnss_raw.txt`：时间戳、经纬度、椭球高、status、service、协方差类型及完整 3×3 协方差；无效 Fix 也保留。
- `gnss_enu.txt`：`timestamp east north up variance_e variance_n variance_u`，单位 m / m²；
  含 `# origin_lla lat lon alt` 元数据，直接适配优化器 `--gnss_enu`。
- `gnss_enu_tum.txt`：`timestamp east north up qx qy qz qw`；GNSS 无姿态，四元数为占位 `0 0 0 1`，不能作为姿态真值。

记录节点还默认订阅 `/gnss_pose`（`geometry_msgs/PoseStamped`），按本数据集的
`position.x=纬度(°)、y=经度(°)、z=椭球高(m)` 提取位置；不是普通米制 PoseStamped。
同目录输出 `gnss_pose_raw.txt`、`gnss_pose_enu.txt`、`gnss_pose_enu_tum.txt`。
原始文件保存时间戳、LLA 和消息中的四元数；ENU 文件使用与 `/gnss` 完全相同的原点，
协方差列为默认 `[1, 1, 4] m²`（消息没有协方差，不是接收机报告的精度）。
TUM 文件仅作为位置真值，姿态填 `0 0 0 1`，不将接收机原始姿态当作 ENU 姿态真值。
无效经纬度仅保留原始行；只播放 `/gnss_pose` 时也能独立建立原点并记录。
如需改话题可用 ROS remap `/gnss_pose:=/your/pose`，无需新增配置项。

仅有效 Fix 写入 ENU，未知或非法协方差使用与原优化器一致的 `[1, 1, 4] m²`。
暂停期间节点保持等待，已接收记录已刷新到文件；跳回旧时间或重新播放时应新建记录，避免重复时间段。
前端复用 `gnss_recorder.enable` 开关，同时将每帧**已去畸变的 IMU/body 坐标系点云**、
强度和纳秒时间戳保存到 `Log/gnss/scans.bin`（随 `map_save.path` 所在目录保存）。
后台写盘队列有内存上限，磁盘跟不上时等待，不静默丢帧；正常退出会写完排队数据。
已有 `scans.bin` 不覆盖，每次采集前将上一轮 GNSS 和点云归档一起移走。

优化器只使用与 LIO 时间范围重叠的 GNSS，默认从 GNSS 文本同目录读取 `scans.bin`，
按优化后的机体位姿变换逐帧点云，输出 `gnss_optimized_map.pcd`，保留强度并做体素降采样。
点云已应用 LiDAR→IMU 外参和前端去畸变，不再重复处理。
归档不在默认位置时指定 `--scans /path/to/scans.bin`。
当前旧记录如果没有逐帧归档，需要重新播放采集；仅有累计 `map.pcd` 不能恢复逐帧轨迹修正。
只优化轨迹时指定 `--skip-output-map`，launch 对应 `skip_output_map:=true`。

通过 launch 加载现有优化参数：

```bash
roslaunch bievr_lio_ros gnss_optimize.launch \
  gnss_enu:=/home/zsq/slam/bievr-slam/src/BIEVR-LIO/Log/gnss/gnss_enu.txt \
  lio_trajectory:=/home/zsq/slam/bievr-slam/src/BIEVR-LIO/Log/odometry_tum.txt
```

`gnss_optimize` 是离线批优化：读取已保存的 LIO 轨迹、GNSS 文本并执行 GTSAM LM，
再读取逐帧点云归档重建 ENU 地图。原 `--bag` 输入已删除。
GNSS 按时间插值约束相邻 LIO 位姿，并使用
`p_antenna = p_body + R_body * lever_arm` 杆臂模型。可在
`config/gnss_optimize.yaml` 配置点云地图、杆臂、协方差缩放/轴向标准差覆盖、
Huber/Cauchy 鲁棒核以及第一轮优化后的卡方异常值剔除。铁路配置将
`odom_neighbor_span` 设为 `3`，以加入参考 PGO 的多邻接 LIO 边；这些组合边与相邻
边相关，如果数据噪声较大或希望更保守，可改回 `1`。

为避免 GNSS 因子逐帧拉动 LIO 位姿并破坏原有点云配准结构，图中默认每 `0.5 s`
选取一个控制位姿；优化后对 `map<-odom` 修正做平滑插值，再作用到全部原始 LIO
位姿。将 `control_pose_interval_s` 设为 `0` 可恢复逐帧优化。

优化结束会在同一输出目录自动生成 GNSS 对比报告（`--skip-output-map` 时也生成）：

- `evaluation_metrics.txt` / `.csv` / `.json`：原始 LIO 与优化后 LIO 的 X/Y/Z、水平、
  三维位置误差 RMSE 和 MAX，单位 m；轴向 MAX 为最大绝对误差，水平/三维 MAX 为最大误差范数。
- `trajectory_xyz.png`：GNSS、LIO、优化后 LIO 的 X/Y/Z 随时间变化曲线。
- `position_error_xyz.png`：两组估计相对 GNSS 的 X/Y/Z 有符号误差。
- `evaluation_samples.csv`：同一批 GNSS 时间戳上的位置及逐轴误差，便于复核。
- `lio_aligned_enu_tum.txt`：原始 LIO 使用后端**优化前**的初始 ENU 变换转换后的轨迹。

以 GNSS 天线位置为参考，应用配置中的天线杆臂，按 GNSS 时间戳插值 LIO；
跨越大于 `2 * max_time_gap_s` 的轨迹缺口不插值。两组使用相同样本、单位尺度和原始时间，
优化结果不再重新对齐；全部匹配到的有效 GNSS 都参与报告，包括后端判为异常的 Fix。
这些数值是相对本次参与优化的 GNSS 的拟合误差。绘图依赖 `python3-numpy`、
`python3-matplotlib` 和 `python3-yaml`。

稠密地图默认体素分辨率 `0.15 m`，通过 `config/gnss_optimize.yaml` 中的 `output_map`、
`map_voxel_resolution_m` 和距离门限参数调整。使用 `--skip-output-map` 时只输出轨迹，
`summary.yaml` 中的 `optimized_map` 保持为空。

优化完成后可将局部 ENU 轨迹叠加到真实道路/铁路地图上：

```bash
python3 scripts/plot_gnss_trajectory_osm.py \
  --trajectory Log/gnss_control05_deskew_validation_20260909/gnss_optimized_tum.txt \
  --summary Log/gnss_control05_deskew_validation_20260909/summary.yaml \
  --output Log/gnss_control05_deskew_validation_20260909/gnss_optimized_trajectory_osm.png \
  --zoom 14
```

默认底图使用 OpenStreetMap 瓦片并显示版权标注；瓦片会缓存到本地。无网络时加
`--no-basemap`，仍会输出 ENU 轨迹图。OSM 标准瓦片适合少量人工查看，不建议批量
下载大范围离线地图；需要大规模离线处理时应改用自建或明确允许离线使用的地图服务。

GNSS 优化默认会从 `scans.bin` 和优化轨迹自动重建已有的 `Log/poles/`，
包括支柱地标、支柱点云、轨道矢量及实测轨道点云，不必再次播放 bag。
旧目录保留为 `poles.before_gnss_*`，新目录记录 `gnss_optimization.yaml`。
首次提取现在也会保存 `extraction_config.yaml` 供重建复用；旧结果没有快照时
回退到 `config/params.yaml`，可用 `--pole_params_file` 指定原提取参数。
默认支柱目录是 LIO 轨迹文件同级的 `poles/`，可用 `--poles_directory` 修改；
`--skip-poles` 跳过自动重建，`--skip-output-map` 只跳过稠密地图。
如果尚无支柱地图，可以将优化轨迹填入 `pole_extraction.mapping_trajectory_path`
后再首次提取。第二次支柱定位必须清空该项，运行时
只使用 LIO 和 `landmarks.txt`。节点还输出含完整初始四元数的
`initial_correction.yaml`，完整流程和实测结果见上述设计文档。

可选的 `rail_extraction` 会复用同一批支柱关键帧，按横断面显著性、软轨距和
连续性提取左右轨顶，将稀疏三维折线保存为 `rail_vector_map.txt`。定位时会把同一
段内相邻控制点连接成三维线段，将当前轨道连续投影到地图线段，而不是与离散采样
点直接做最近邻匹配。第二次运行加载
该矢量地图，以鲁棒线残差约束横向、`z/roll/pitch/yaw`，但明确不约束沿轨方向
平移；支柱继续提供绝对 `x/y/yaw` 锚点。为避免平行股道歧义，支柱尚未确认全局
位置时，轨道平面解不会写入支柱关联初值或 GTSAM 图。查询阶段仍不加载 GNSS 或
稠密点云。组合模式使用：

```bash
roslaunch bievr_lio_ros process_bag.launch sensor_config:=xiangyang \
  params:=params params_overlay:=params_pole_rail_localization \
  rosbag:="/absolute/path/to/bag.bag" rviz:=true
```

轨道提取还支持传感器安装位置先验：`rail_extraction.mount_prior_enable` 开启后，
由 `mount_height_m`、`mount_center_y_m` 和标准轨距预测本车左右钢轨的位置。
高度是轨顶相对 IMU/body 原点的向下距离，中心 Y 是轨道中心相对该原点的 Y 坐标；
当前襄阳配置的点云/IMU 外参为单位变换，估计值分别为 4.135 m 和 +0.49 m。
预测随插值的 body 姿态转换到轨迹横断面，支持反向行驶；候选须通过横向/高度
容差门限，并在显著性、轨距和连续性评分之外加入安装偏差惩罚。
这只约束候选选择，输出仍来自实测轨顶；超出容差时不生成替代轨道。
默认横向容差 0.25 m、高度容差 0.20 m，换安装位置或外参后须重新估计。
建图和定位共享 `params.yaml` 中这组提取参数；旧地图快照未记录这些键时仍使用
原算法，GNSS 重建可用 `--pole_params_file config/params.yaml` 显式采用新配置。

轨道观测和地图分别发布到 `/bievr_lio/rails/observed` 与
`/bievr_lio/rails/map`。本数据的同源验证中，垂直 RMSE 从纯支柱的 `0.960 m`
降至 `0.076 m`，pitch RMSE 从 `0.246 deg` 降至 `0.039 deg`；详细限制和指标见
设计文档。

#### 推荐的铁路数据完整流程

以下命令以襄阳铁路数据为例。路径和输出目录可以替换。GNSS 优化会自动更新已有
支柱/轨道地图；手动首次提取时才需要填写 `mapping_trajectory_path`，重定位阶段必须清空。

```bash
source /opt/ros/noetic/setup.bash
source /home/zsq/slam/bievr-slam/devel_isolated/setup.bash

BAG="/mnt/e/Data/襄阳铁路数据-第二次数据/2026-01-13_襄阳金鹰重工_焦柳线_郜营-余家湖/20260113/bag20260113_1.bag"

# 终端 1：保持 ROS master 运行
roscore

# 终端 2：继续执行下面的命令
# 1. 先启动在线前端及 GNSS 记录器（另开终端播放，任意时刻停止后关闭前端）
roslaunch bievr_lio_ros mapping.launch
# 另一个终端：rosbag play --clock "$BAG"

# 2. 从本次保存的 GNSS、LIO 和逐帧点云建立优化轨迹、ENU 地图和初始变换
rosrun bievr_lio_ros gnss_optimize --gnss_enu Log/gnss/gnss_enu.txt \
  --lio_trajectory Log/odometry_tum.txt \
  --output_directory Log/gnss_optimization
```

GNSS 初始对齐使用首个有效 GNSS/LIO 匹配点开始的前 45 秒，估计统一的 yaw 和
XYZ 平移，再应用于整条轨迹；Z 平移取这段天线高度差的中位数。全程 GNSS 仍参与
后续因子图优化。通过 `config/gnss_optimize.yaml` 中的
`initial_alignment_duration_s` 和 `initial_alignment_min_baseline_m` 调整窗口和
最小水平基线（默认 5 m）。该 YAML 由 `gnss_optimize.launch` 加载；直接运行上面的
`rosrun` 命令使用内置默认值。窗口内少于三个匹配点或水平运动不足时会报错，应调整
窗口或检查数据。`summary.yaml` 记录实际对齐点数、相对 LIO 起点的时间偏移和跨度；
`initial_3d_rmse_m` 仍是初始对齐后的全程残差，并非起点误差。

`initial_outlier_gate_m` 只对上述初始对齐窗口内的 GNSS 生效，设为 `0` 可关闭。
窗口之后的 GNSS 不再因初始 LIO 的累计漂移被提前删除，而是进入鲁棒优化，再由
`postfit_outlier_chi2` 检查优化后的残差。`summary.yaml` 中的
`initial_outlier_gate_scope: alignment_window` 表示采用此规则。

如果 GNSS 优化前已经生成 `Log/poles/`，上一步会自动更新其中结果，无需重复回放。
只有首次运行时未开启提取、尚无支柱地图，才需要在 `config/params.yaml` 中设置：

```yaml
pole_extraction:
  enable: true
  localization_enable: false
  mapping_trajectory_path: "/absolute/path/to/Log/gnss_optimization/gnss_optimized_tum.txt"

rail_extraction:
  enable: true
```

然后再次运行 `process_bag`。输出目录中会同时生成
`landmarks.txt` 和 `rail_vector_map.txt`。建图阶段可以使用累计点云，也可以将
`map_save.accumulate` 设为 `false`；轨道/支柱地图本身不依赖稠密 PCD。

第二次运行前，编辑
`config/params_pole_rail_localization.yaml` 中的 `localization_map_path`、
`rail_extraction.map_path` 和输出目录，然后运行：

```bash
roslaunch bievr_lio_ros process_bag.launch \
  sensor_config:=xiangyang \
  params:=params params_overlay:=params_pole_rail_localization \
  rosbag:="$BAG" rviz:=true
```

查询阶段只读取 `/livox/lidar` 和 `/imu/data`（或传感器配置中的 LiDAR/IMU 话题），
不读取 `/gnss`，不加载 `map.pcd`，也不运行 GICP。若从中途开始运行，需要将
`gnss_optimize` 输出的 `initial_correction.yaml` 内容作为最后一层参数，或者通过
RViz `/initialpose` 提供初始位姿。

轨道滤波在首次有效观测时允许建立大范围高程初值；正常跟踪仍使用独立的
`max_innovation_z_m`、`max_innovation_tilt_deg` 和 `rail_mahalanobis_gate` 门限。若超过
`pole_localization.rail_rebootstrap_gap_m`（默认 100 m）没有接受轨道约束，下一条
通过匹配数、基线和 RMS 检验的轨道观测会重新捕获竖直状态。姿态修正绕当前轨道
窗口锚点实施，并同步补偿平移，避免在长距离 odom 坐标上由小 pitch 产生水平跳变。

连续 bag2–bag4 的同源闭环验证共 15946 个 LIO 位姿、约 1594.4 s。GNSS 图使用
6378 个位置因子；稀疏地图含 466 根支柱和 32697 个轨道采样点。第二次运行明确为
`POLE_MAP_ONLY (no dense map or GICP)`，最终支柱滤波接受 408/408，轨道滤波接受
401/413；对 GNSS 优化参考的水平 RMSE 为 0.588 m、竖直 RMSE 为 0.947 m、yaw
RMSE 为 0.158 deg。由于建图、定位和评估使用同一组数据，这些数值表示流程一致性，
不能替代独立往返数据或测量控制点的绝对精度评估。

另外使用一个常驻 `process_topics` 节点和一次 `rosbag play --clock --rate=1.0`
连续播放 bag2–bag4，完成了 1594.8 s 的在线回调测试。节点收到 15946 帧，处理 545
个关键帧且 `dropped_keyframes: 0`；在线生成的 12522 个已定位位姿与上述离线连续
处理结果逐点一致。测试输出、统计和误差图保存在
`Log/online_bag2_bag4_20260910/`。

另一次论文用在线实验从 bag1 的 150 s 冷启动，以一个常驻 `process_topics` 和
一次 `rosbag play --clock --rate=1.0` 连续播放 bag1--bag3，共处理 1493.3 s、
12622.5 m。建图得到 472 根支柱和 32141 个轨道矢量采样，522 个关键帧无丢弃；
查询阶段只加载这两种稀疏地图，不读取 GNSS、不加载稠密 PCD、不运行 GICP。
相对同批数据的 GNSS--LIO GTSAM 参考轨迹，组合定位水平 RMSE 为 0.269 m、垂向
RMSE 为 0.396 m、yaw RMSE 为 0.087 deg；完整报告、分 bag 指标和论文 PDF/PNG
图位于 `Log/paper_online_bag1_150_bag3_20260910/report/`。该结果属于同源闭环验证，
不是独立数据或测量真值实验。

GNSS 后端使用 GTSAM 优化 LIO 轨迹。稀疏定位前端仍使用 Hungarian + Huber
`SE(2)` 完成支柱关联，并用鲁棒小量最小二乘拟合轨道；通过几何和创新门控后，每个
提取关键帧建立一个 GTSAM `Pose3`，相邻节点由 LIO `BetweenFactor` 连接。支柱因子
只约束 `x/y/yaw`；轨道自定义 5DoF 因子约束横向、`z/roll/pitch/yaw`，其残差空间
不含沿轨平移。优化后的关键帧修正在稠密 LIO 轨迹上插值。轨道不是稠密点云匹配的
替代品，而是稀疏支柱地图模式中的连续横向、高程和姿态约束。

#### 评估和可视化

论文风格的轨迹、误差、稀疏地图、匹配质量和门限消融图可统一生成：

```bash
python3 scripts/generate_sparse_localization_paper_figures.py \
  --lio Log/experiment/localization/odometry_tum.txt \
  --localized Log/experiment/localization/localization_tum.txt \
  --baseline-localized Log/experiment/localization_baseline/localization_tum.txt \
  --reference Log/experiment/gnss/gnss_optimized_tum.txt \
  --poles Log/experiment/mapping/sparse_map/landmarks.txt \
  --rails Log/experiment/mapping/sparse_map/rail_vector_map.txt \
  --corrections Log/experiment/localization/observations/localization_corrections.txt \
  --output-directory Log/experiment/report/figures
```

每张图同时输出矢量 PDF 和 300 dpi PNG，数值保存在
`comparison_metrics.json`；可重复提供 `--boundary BAG_END_TIMESTAMP` 计算并标记
各 bag 区间。

相对于参考 TUM 轨迹计算水平、垂直、三维、yaw、roll 和 pitch 误差：

```bash
python3 scripts/evaluate_tum_against_reference.py \
  --estimate Log/pole_rail_localization/localization_tum.txt \
  --reference Log/gnss_optimization/gnss_optimized_tum.txt \
  --output Log/pole_rail_localization/metrics.json
```

绘制沿线水平/高程误差并标记分包边界：

```bash
python3 scripts/plot_localization_errors.py \
  --estimate Log/pole_rail_localization/localization_tum.txt \
  --reference Log/gnss_optimization/gnss_optimized_tum.txt \
  --boundary 1768270776.931093 --boundary 1768271314.443376 \
  --output Log/pole_rail_localization/localization_errors.png
```

查看支柱和轨道矢量地图，并导出 CloudCompare/MeshLab 可读的 PLY：

```bash
python3 scripts/visualize_poles.py --no-map --no-gui \
  --poles Log/poles \
  --overview Log/poles/rail_overview.png \
  --export Log/poles/rail_map_colored.ply
```

`visualize_poles.py` 检测到 `rail_vector_map.txt` 后会自动绘制左右轨；没有该文件
时仍可只查看支柱地图。

稠密点云重定位叠加 `params_localization.yaml`：

```bash
roslaunch bievr_lio_ros process_bag.launch sensor_config:=xiangyang \
  params:=params params_overlay:=params_localization \
  rosbag:="/absolute/path/to/bag.bag" rviz:=true
```

该模式关闭全部支柱模块，只加载稠密地图并运行 GICP。完全不加载 `map.pcd`、
只使用 LIO 和 `landmarks.txt` 时，使用独立的纯支柱定位模式：

```bash
roslaunch bievr_lio_ros pole_localization.launch
```

离线复放使用：

```bash
roslaunch bievr_lio_ros process_bag.launch sensor_config:=xiangyang \
  params:=params params_overlay:=params_pole_localization \
  rosbag:="/absolute/path/to/bag.bag" rviz:=true
```

纯支柱模式发布与稠密定位相同的 `/bievr_lio/loc/odom`、`/bievr_lio/loc/path`、
`map -> odom` TF 和 `/bievr_lio/loc/status`。默认等待 `/initialpose`；已知两次 LIO
共原点时也可临时启用 `pole_extraction.localization_use_initial_guess`。支柱失配后
状态按行驶距离从 `TRACKING` 变为 `DEGRADED/LOST`，但最后有效修正会保留，LIO
局部轨迹不会被打断。后台提取得到的修正按关键帧传感器时间和累计里程进入滤波，
离线高速回放时会回填修正对应的轨迹尾段，不受后台线程完成时刻影响。

launch 文件只负责选择配置文件、数据和 RViz，不会覆盖算法参数。运行模式
统一在参数文件中设置；两个重定位模式严格互斥，误将它们同时开启会直接退出：

- 纯里程计：`loop_closure.enable: False`，`localization.enable: False`
- 回环建图：`loop_closure.enable: True`，`localization.enable: False`；地图包
  默认输出到 `Log/bievr_map_bundle`
- 稠密点云定位：`loop_closure.enable: False`，`localization.enable: True`，
  `pole_extraction.enable: False`，`pole_localization.enable: False`；默认加载
  `Log/bievr_map_bundle`
- 纯支柱定位：`loop_closure.enable: False`，`localization.enable: False`，
  `pole_extraction.localization_enable: True`，`pole_localization.enable: True`
- 支柱 + 轨道矢量定位：在纯支柱模式基础上设置 `rail_extraction.enable: True`，
  或叠加 `params_pole_rail_localization.yaml`

For topic-based mapping, `mapping.launch` contains the sensor-config and
parameter-file paths relative to `$(find bievr_lio_ros)`. Edit those paths in the
launch file; the bag path is intentionally not stored there.

Algorithm settings are not overridden by launch files. Use `config/params.yaml`
as the base and select exactly one localization overlay when relocalizing:

- Odometry: `loop_closure.enable: False`, `localization.enable: False`
- Mapping: `loop_closure.enable: True`, `localization.enable: False`; the default
  bundle output is `Log/bievr_map_bundle`
- Dense-map localization (`config/params_localization.yaml`): `localization.enable: True`,
  `pole_extraction.enable: False`, `pole_localization.enable: False`, and load
  `Log/bievr_map_bundle` through `localization.map_path`
- Pole-map localization (`config/params_pole_localization.yaml`): `localization.enable: False`,
  `pole_extraction.localization_enable: True`, `pole_localization.enable: True`

Start topic-based mapping (also used with `rosbag play`):

```bash
roslaunch bievr_lio_ros mapping.launch
```

In a second sourced terminal, replay the rosbag:

```bash
rosbag play /absolute/path/to/data.bag
```

## Configuration

The configuration is split in two files:

- **`config/params.yaml`**: Algorithm parameters (map resolution, sampling,
  optimization, IMU window, ...). These are dataset-independent and **typically do
  not need to be adjusted**: the defaults have been validated across a wide range
  of sensors, platforms, and environments.
- **`config/sensor_configs/<name>.yaml`**: Per-dataset / per-sensor settings:
  the LiDAR and IMU topic names, the LiDAR→IMU extrinsic calibration, and the
  LiDAR min/max range.

Set the `--sensor_config_file` and `--params_file` paths directly in
`interfaces/ros1/launch/mapping.launch`, `localization.launch`, and
`pole_localization.launch`. They use `$(find bievr_lio_ros)` so the launch files
do not depend on the workspace's absolute path. Other specialized launch files
may still expose configuration names as launch arguments for offline evaluation.

<details>
<summary><b>Provided datasets</b></summary>
<br>

We provide ready-to-use sensor configs for the following public datasets:

| Config | Dataset |
|--------|---------|
| `enwide` | [ENWIDE](https://projects.asl.ethz.ch/datasets/enwide/) |
| `ncd` | [Newer College Dataset](https://drive.google.com/drive/u/0/folders/1uR476FzjN3PfAiCknVKtuZi3_QfVvSdA) |
| `gamma` | [GEODE](https://thisparticle.github.io/geode) |
| `mars` | [MARS-LVIG](https://mars.hku.hk/dataset.html) |
| `grandtour` | [GrandTour](https://grand-tour.leggedrobotics.com/) |
</details>

<details>
<summary><b>Running on your own data</b></summary>
<br>

To run BIEVR-LIO on a new sensor or dataset, copy one of the provided sensor
configs to `config/sensor_configs/<your_name>.yaml` and adjust:

- `topics.pointcloud` / `topics.imu` : The topic names in your data.
- `calibration` : the `T_IMU_LIDAR` extrinsic (LiDAR → IMU) rotation and
  translation for your setup.
- `lidar.min_range_m` / `lidar.max_range_m` : the usable range of your LiDAR.

The algorithm parameters in `params.yaml` can usually be left at their defaults.
</details>

# Acknowledgements
For a full SLAM integration including loop-closure detection and pose-graph optimization check out [BIEVR-LIO-SLAM](https://github.com/S0UL4/BIEVR-LIO-SLAM), contributed by [SOUL4](https://github.com/S0UL4)

We thank the authors of [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry), [Wavemap](https://github.com/ethz-asl/wavemap) and [UGPM](https://github.com/UTS-RI/ugpm) for open-sourcing their works that served as an inspiration for us.
We used [ascii-image-converter](https://github.com/TheZoraiz/ascii-image-converter) for our ascii art.

# Citation

Please cite our work if you are using BIEVR-LIO in your research.
  ```bibtex
@article{pfreundschuh2026bievr,
  title        = {BIEVR-LIO: Robust LiDAR-Inertial Odometry through Bump-Image-Enhanced Voxel Maps},
  author       = {Pfreundschuh, Patrick and Tuna, Turcan and {Le Gentil}, Cedric and Siegwart, Roland and Cadena, Cesar and Oleynikova, Helen},
  year         = 2026,
  journal      = {Robotics: Science and Systems},
}
  ```
