# `solohao/robot` 实验二复现会话交接文档

更新时间：2026-07-11
原 Devin 会话：https://app.devin.ai/sessions/9a031004ab79495992670538ebea6a02

## 1. 新会话如何使用本文档

在新会话中上传本文档，并明确：

> 请从“当前现场状态”和“下一步”继续指导我复现实验二。每次只进行一个阶段，
> 等我反馈后再继续；不要让我同时启动多套仿真。

新会话应先确认当前主仿真是否仍在运行，再给下一阶段命令，不能从头重新安装或
重复已经完成的排障。

## 2. 用户最终目标

在 Windows 主机的 VMware Workstation 17 Pro 中，从零复现
`solohao/robot` 仓库实验二：

1. Ubuntu 22.04、ROS 2 Humble 和 TurtleBot4 Ignition Gazebo 仿真；
2. ROS 2 基础操作；
3. SLAM Toolbox 增量建图；
4. 保存有效地图；
5. AMCL 自定位；
6. 仓库自定义 A* Nav2 Global Planner；
7. RViz 设置初始位姿和导航目标；
8. 机器人绕过 `shelf_7` 并到达目标；
9. 验证 `/clock`、里程计、激光、规划路径和导航终态。

目前还没有完成最终 AMCL、A* 和 Nav2 现场复现；当前正在进一步降低
Create 3 辅助 GPU lidar 负载，解决用户虚拟机长时间运行后 `/clock` 停止。

## 3. 沟通和执行要求

- 使用中文，解释简短、明确；
- 每次只给一个阶段，用户反馈后再继续；
- 明确哪些终端必须保持运行、何时按 `Ctrl+C`；
- 不允许同时启动两套 Gazebo/ROS 仿真；
- 用户报告错误时，先根据日志分析，再给下一步；
- 不把性能受限、地图无效或仿真停滞描述成成功；
- 不要求完整探索整个 warehouse；
- 不修改 `/opt/ros/humble`；
- 不用零散 `cp`、`sed`、临时环境变量继续手工修补，优先使用仓库的一键预设；
- 代码修改必须走新分支和 PR，不能直接推送目标分支；
- 当前指导重点是用户虚拟机现场复现，不是重新做仓库架构分析。

## 4. 环境信息

### Windows 主机

- Intel Core i7-13650HX；
- 14 物理核、20 逻辑处理器；
- 16 GB RAM；
- Intel UHD Graphics；
- NVIDIA GeForce RTX 4060 Laptop GPU，8 GB；
- VMware Workstation 17 Pro。

### Ubuntu 虚拟机

- Ubuntu 22.04.5 LTS；
- 6 vCPU；
- 约 7.7 GiB RAM；
- 80 GB 虚拟磁盘，曾剩余约 59 GiB；
- NAT 网络；
- VMware 3D 加速已启用；
- guest OpenGL renderer 为 VMware `SVGA3D`，不是 RTX 4060 直通；
- swap 已从 2 GiB 扩大到 8 GiB。

VMware 可在 Windows 主机侧使用 RTX 4060，但 Ubuntu guest 通常仍显示
`SVGA3D`。本实验现阶段不要求配置 RTX，先用已经验证的兼容预设完成实验。

## 5. 仓库和路径

- GitHub：`https://github.com/solohao/robot`
- 用户仓库：`~/robot`
- 实验目录：`~/robot/experiment2`
- ROS 工作空间：`~/robot/experiment2/ros2_ws`
- 目标分支：`devin/1783302960-experiment1`
- 仓库一键命令：

  ```bash
  ros2 run tb4_experiment_bringup vmware_simulation
  ```

### 重要文件

- `experiment2/README.md`
- `experiment2/SUBMISSION.md`
- `experiment2/tb4-test-plan.md`
- `experiment2/maps/lab_map.yaml`
- `experiment2/maps/lab_map.pgm`
- `experiment2/ros2_ws/src/tb4_experiment_bringup/launch/simulation.launch.py`
- `experiment2/ros2_ws/src/tb4_experiment_bringup/scripts/vmware_simulation`
- `experiment2/ros2_ws/src/tb4_experiment_bringup/scripts/validate_vmware_simulation.py`
- `experiment2/ros2_ws/src/tb4_experiment_bringup/config/nav2_astar.yaml`
- `experiment2/ros2_ws/src/tb4_astar_planner/`

## 6. 已合并的相关 PR

- PR #15：显式支持实验二自定义 Gazebo 仿真参数
  https://github.com/solohao/robot/pull/15
- PR #16：修复重复 SLAM 节点并固定 RViz 视图
  https://github.com/solohao/robot/pull/16
- PR #17：隔离仿真子启动参数作用域
  https://github.com/solohao/robot/pull/17
- PR #18：完善实验二仿真报告与提交资料包
  https://github.com/solohao/robot/pull/18
- PR #19：新增实验二 VMware 一键低负载仿真预设
  https://github.com/solohao/robot/pull/19
- PR #20：修复 VMware 预设底盘控制器延迟启动
  https://github.com/solohao/robot/pull/20

PR #19 和 #20 均已合并到 `devin/1783302960-experiment1`。

## 7. 一键 VMware 预设的行为

建图启动：

```bash
cd ~/robot/experiment2/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run tb4_experiment_bringup vmware_simulation slam:=true
```

该脚本：

1. 在 `~/.cache/tb4_experiment_bringup/vmware/` 自动建立运行资源；
2. 从 `/opt/ros/humble` 复制官方 world 和 description 到用户缓存；
3. 不修改 `/opt/ros/humble`；
4. warehouse `max_step_size: 0.003 -> 0.1`；
5. OAK-D `always_on=0`、`update_rate=1`、关闭可视化；
6. RPLIDAR 保持启用，降为 10 Hz，关闭射线可视化；
7. Create 3 cliff/IR 辅助 GPU lidar 从 62 Hz 降为 1 Hz；
8. Gazebo 使用 server-only；
9. 仅 Gazebo 进程使用 `LIBGL_ALWAYS_SOFTWARE=1`；
10. RViz 不继承软件渲染变量，继续使用 VMware SVGA3D；
11. `slam:=true` 时默认不启动不需要的 Nav2 服务器；
12. 自动验证 `/clock` 和 `/scan`。

验证通过日志类似：

```text
VMware preset validated: /clock is active and /scan contains environment returns ...
```

## 8. 当前现场状态

用户已经：

1. 拉取 PR #19 和 PR #20；
2. 成功编译 `tb4_experiment_bringup`；
3. 确认安装的可执行文件：

   ```text
   tb4_experiment_bringup validate_vmware_simulation
   tb4_experiment_bringup vmware_simulation
   ```

4. 使用以下命令成功启动修复后的 SLAM 仿真：

   ```bash
   ros2 run tb4_experiment_bringup vmware_simulation slam:=true
   ```

5. RViz 已显示机器人、有效 LaserScan 和正在增量扩展的地图；
6. 用户尝试 teleop 时机器人短暂移动，随后 `/clock` 停止；主 launch 已要求
   用户按 `Ctrl+C` 退出，不应继续使用该旧进程。

用户最新测得：

```text
/clock ≈ 1.91 Hz
/odom  ≈ 1.91 Hz
/scan  ≈ 5.70 Hz
```

当时的结论：

- `/clock` 持续推进；
- 里程计持续发布；
- 激光持续发布；
- 启动后的基础链路正常，但尚不能保证用户机器长时间持续运行；
- 软件渲染下墙钟时间会比仿真时间更长，不改变基于 `use_sim_time` 的算法逻辑。

RViz/终端中出现过：

```text
TF_OLD_DATA ignoring data from the past
```

这是低速仿真和较大物理步长下的消息顺序警告。当前三项话题持续更新、地图和
激光正常，因此暂时不按致命错误处理。若之后 `/clock` 停止，则必须优先处理，
不能继续遥控或导航。

随后用户实际遥控时，日志中的多个 `TF_OLD_DATA` 固定在同一仿真时间
`848.2`，再次检查 `/clock` 完全无输出。进一步定位发现官方 Create 3 仍包含
4 组 cliff 和 7 组 IR intensity `gpu_lidar`，默认均约 62 Hz；在 VMware
llvmpipe 中持续计算会造成额外负载。

当前待合并 PR 已扩展 `vmware_simulation`：

- 同时复制并覆盖 `irobot_create_description`；
- 将 Create 3 cliff/IR GPU lidar 从 62 Hz 降为 1 Hz；
- 保留 RPLIDAR 10 Hz、里程计、TF、底盘控制和传感器几何。

开发环境最新验证：

```text
/clock ≈ 4.60 Hz
/scan  ≈ 4.63 Hz
```

持续运行后 `/clock` 仍在推进；发布 `0.15 m/s` 速度约 8 秒墙钟后，里程计从：

```text
x: 1.865e-09
y: 2.058e-22
```

变化为：

```text
x: 0.06213
y: -0.01394
```

说明降载版本中底盘运动链路有效。用户虚拟机尚未拉取并复测该最新 PR。

## 9. 下一步：更新最新降载 PR 并复测

新会话应先确认降低 Create 3 cliff/IR 频率的 PR 已合并，然后让用户在没有旧
Gazebo/ROS 进程的情况下执行：

```bash
cd ~/robot
git pull
cd experiment2/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select tb4_experiment_bringup
source install/setup.bash
ros2 run tb4_experiment_bringup vmware_simulation slam:=true
```

启动输出应额外包含：

```text
Create 3 cliff/IR sensors: 1 Hz
```

等待自动验证通过后，先观察 `/clock` 至少 2–3 分钟，再进行遥控：

```bash
source /opt/ros/humble/setup.bash
source ~/robot/experiment2/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

执行要点：

1. 鼠标点击 teleop 终端；
2. 输入法切为 English；
3. 先按 3 次 `x` 降低速度；
4. 短按 `i` 前进，然后按 `k` 停止；
5. 软件渲染下动作反馈可能延迟，不要长按；
6. 观察 RViz 中机器人位置和地图是否发生小幅变化；
7. 如果按键后没有运动，先检查：

   ```bash
   ros2 topic echo /cmd_vel --once
   ros2 topic hz /clock
   ```

   `/cmd_vel` 有正速度但 `/clock` 无输出，表示 Gazebo 停滞，不是 teleop 或
   A* 错误。

完成运动验证后，只在 teleop 终端按 `Ctrl+C`；主 launch 和 RViz 继续运行。

## 10. 后续阶段顺序

必须按以下顺序逐阶段完成，不要一次把全部命令发给用户。

### 阶段 A：有效 SLAM 证据

- 遥控原地转向并前进少量距离；
- 不要求完整探索 warehouse；
- 只需证明有效激光使地图随运动增量扩展；
- 地图应出现墙体、货架或通道边界；
- 可以截图作为 SLAM 证据。

### 阶段 B：保存本地复现地图

建议保存到：

```bash
mkdir -p ~/robot/experiment2/maps/reproduction
ros2 run nav2_map_server map_saver_cli \
  -f ~/robot/experiment2/maps/reproduction/lab_map
```

保存后检查：

```bash
ls -lh ~/robot/experiment2/maps/reproduction/lab_map.*
cat ~/robot/experiment2/maps/reproduction/lab_map.yaml
ros2 topic echo /map --once --field info
```

有效性要求：

- `/scan` 必须包含不同距离值或数字与 `inf` 混合；
- 不能全部是 `0.164 m`；
- `8 × 23` 的极小地图不能用于 AMCL/Nav2；
- 不把“Map saved”本身等同于地图有效。

本地短程建图主要用于证明 SLAM 流程。后续 AMCL/A*/Nav2 可优先使用仓库中已经
验证的完整地图：

```text
~/robot/experiment2/maps/lab_map.yaml
```

### 阶段 C：停止 SLAM

- 先停止 teleop；
- 在主 launch 终端按 `Ctrl+C`；
- 等所有进程完全退出并回到 shell 提示符；
- 不在旧节点退出过程中立即启动下一套仿真。

### 阶段 D：一键启动定位和导航

```bash
cd ~/robot/experiment2/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run tb4_experiment_bringup vmware_simulation \
  map:=$HOME/robot/experiment2/maps/lab_map.yaml
```

该模式默认：

- 不运行 SLAM；
- 运行 AMCL；
- 运行 Nav2；
- 加载自定义 A* 配置；
- 继续使用 VMware 低负载预设。

### 阶段 E：AMCL 初始化

在 RViz 中使用 `2D Pose Estimate`：

1. 在地图上的机器人真实起点按下；
2. 沿机器人正前方拖出短箭头；
3. 等待粒子云集中；
4. 等 LaserScan 与地图边缘基本对齐；
5. 验证：

   ```bash
   ros2 run tf2_ros tf2_echo map odom --once
   ```

### 阶段 F：确认自定义 A* 已加载

```bash
ros2 param get /planner_server planner_plugins
ros2 param get /planner_server GridBased.plugin
ros2 lifecycle get /planner_server
```

关键预期：

```text
GridBased
tb4_astar_planner/AStarPlanner
active [3]
```

### 阶段 G：RViz 发送导航目标

- 使用 RViz `Nav2 Goal`；
- 目标应位于 `shelf_7` 另一侧；
- 观察红色全局路径绕过货架；
- 不要求实时率达到 1.0；
- 等待同一导航 goal 从 `EXECUTING (2)` 到 `SUCCEEDED (4)`；
- 最终确认机器人到达目标而不是只开始移动。

## 11. 关键问题及结论

### `gz_args` 会完整覆盖默认参数

错误示例：

```bash
gz_args:="-r -v 2"
```

会丢失 world 路径并进入 Gazebo Quick Start。自定义 `gz_args` 必须包含完整
world 路径。一键脚本已经自动处理，现场复现不再手工拼接。

### 全部为 `0.164 m` 的扫描无效

`0.164 m` 是 RPLIDAR 最小量程。Standard 和 Lite 模型都曾在
`SVGA3D + OGRE2` 下返回全 `0.164`，说明是 VMware GPU lidar 兼容问题，
不是探索不足或模型选择错误。

软件渲染恢复过真实数据，例如：

```text
1.72
2.42
7.18
11.98
inf
```

一键预设会自动检查扫描中是否包含有效环境返回和距离变化/`inf`。

### 地图 `8 × 23` 不能用于导航

曾经保存成功但日志仅为：

```text
Received a 8 X 23 map @ 0.05 m/pix
```

该地图是在无效扫描下生成，只覆盖机器人附近，不能用于 AMCL/Nav2。

### `/cmd_vel` 有数据但机器人不动

曾测得：

```yaml
linear:
  x: 0.475495...
```

但当时 `/clock` 无输出。结论是 Gazebo 仿真时间停止，teleop 本身正常。
排障优先级始终是：

```bash
ros2 topic hz /clock
ros2 topic hz /odom
ros2 topic hz /scan
```

### namespace 延迟控制器错误

首次运行 PR #19 时，用户遇到：

```text
launch configuration 'namespace' does not exist
```

原因是官方 `irobot_create_control` 用 `OnProcessExit` 延迟启动
`diffdrive_controller`，部分 Humble 版本在 scoped group 退出后才解析
`namespace`。PR #20 已让父 launch 保留并显式传递 `namespace`，并新增回归测试。

修复后用户已经成功运行。

## 12. 仓库侧验证记录

PR #19 在开发环境验证：

- 完整工作空间构建成功；
- 49 项测试通过；
- `/scan` 有效范围约 `0.221–11.989 m`；
- Gazebo 进程有 `LIBGL_ALWAYS_SOFTWARE=1`；
- RViz 进程没有该变量；
- `/clock` 持续发布；
- 发布速度命令后 `/odom` 位置发生变化。

PR #20 验证：

- 完整工作空间 53 项测试通过；
- `joint_state_broadcaster` 成功激活；
- 延迟的 `diffdrive_controller` 成功激活；
- 启动检查器通过；
- 有效扫描约 `0.222–11.989 m`。

## 13. 不应误判的日志

以下通常不是 A* 代码错误：

```text
Timed out waiting for transform from base_link to odom
Could not find a connection between 'odom' and 'base_link'
TF has two or more unconnected trees.
TF_OLD_DATA ignoring data from the past
```

应先确认 Gazebo 是否已经生成机器人，以及 `/clock`、`/odom`、`/scan` 是否
持续更新。只有基础链路正常后，才检查 AMCL、Planner Server 或 A*。

## 14. 旧手工 overlay

用户机器上可能仍有：

```text
~/tb4-test-runtime/
```

以及当前旧终端环境中的：

```text
AMENT_PREFIX_PATH=/home/liujunhao/tb4-test-runtime/overlay:...
```

编译时出现过：

```text
The path '/home/liujunhao/tb4-test-runtime/overlay' ... doesn't contain any
'local_setup.*' files.
```

该警告不是编译失败。一键脚本会把新的缓存 overlay 放在搜索路径最前面，并显式
取消父终端的 `LIBGL_ALWAYS_SOFTWARE` 后只对 Gazebo 重新设置。新终端通常不会
保留旧的 `export`；无需删除 `/opt/ros` 或重新安装 ROS。

## 15. 当前成功标准

当前阶段不是“整个实验完成”，而是：

- PR #19/#20 的一键入口与 controller 修复已合并；
- 用户机器曾在持续运行时再次发生 `/clock` 停止；
- 新降载修改已将 Create 3 cliff/IR GPU lidar 从 62 Hz 降为 1 Hz；
- 开发环境已验证持续 `/clock`、有效 `/scan` 和 `/odom` 位移；
- 下一步应合并最新 PR，让用户拉取后复测 2–3 分钟时钟和短程遥控运动；
- 之后再保存地图、切换 AMCL/A*/Nav2，并验证绕过 `shelf_7` 到达目标。
