# 实验二：TurtleBot4 移动机器人自主导航与避障

本目录包含 ROS 2 Humble 下的 TurtleBot4 仿真、SLAM、定位、自主导航，以及基于
Nav2 `GlobalPlanner` 接口实现的代价感知 A* 全局路径规划器。

## 目录

- `ros2_ws/src/tb4_astar_planner/`：A* 规划器插件与单元测试
- `ros2_ws/src/tb4_experiment_bringup/`：仿真和实机启动文件、Nav2 参数
- `maps/`：SLAM 保存的 `lab_map.yaml` 与 `lab_map.pgm`
- `report/`：LaTeX 源文件、Word/PDF 实验报告、图表和客观测试数据
- `video/`：SLAM 增量建图与自主导航录屏
- `SUBMISSION.md`：课程提交资料包清单与验收结果

## 安装

推荐系统为 Ubuntu 22.04 和 ROS 2 Humble。安装 ROS 2 后执行：

```bash
sudo apt update
sudo apt install \
  ros-humble-turtlebot4-simulator \
  ros-humble-turtlebot4-desktop \
  ros-humble-irobot-create-nodes \
  ros-humble-teleop-twist-keyboard \
  ros-dev-tools
```

## 编译与测试

```bash
source /opt/ros/humble/setup.bash
cd experiment2/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

## 仿真导航

使用 TurtleBot4 自带的 `warehouse` 地图进行定位和导航：

```bash
source /opt/ros/humble/setup.bash
source experiment2/ros2_ws/install/setup.bash
ros2 launch tb4_experiment_bringup simulation.launch.py
```

Ignition Gazebo、AMCL、Nav2 和 RViz2 会依次启动。在 RViz2 中先用
`2D Pose Estimate` 设置初始位姿，再用 `Nav2 Goal` 设置目标。
项目使用固定俯视角的 `experiment.rviz`，每次启动都会恢复相同地图朝向。

如需覆盖传给 Gazebo Sim 的完整参数（例如使用自定义 SDF），可传入
`gz_args`：

```bash
ros2 launch tb4_experiment_bringup simulation.launch.py \
  gz_args:="/absolute/path/to/custom-world.sdf -r -s -v 2"
```

切换其他官方场景时，地图必须与场景一致：

```bash
ros2 launch tb4_experiment_bringup simulation.launch.py \
  world:=maze \
  map:=/opt/ros/humble/share/turtlebot4_navigation/maps/maze.yaml
```

## 仿真建图

```bash
ros2 launch tb4_experiment_bringup simulation.launch.py slam:=true
```

另开终端遥控：

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

建图完成后保存：

```bash
cd experiment2/maps
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli -f lab_map
```

## VMware 低负载一键预设

VMware Workstation 的 `SVGA3D + OGRE2` 组合可能使 Ignition
`gpu_lidar` 的每束距离都错误地等于最小量程 `0.164 m`。不要用这种扫描继续
建图；生成的地图通常只有机器人附近几格。例如 `8 × 23` 的地图不能用于后续
AMCL 或 Nav2。

完成工作空间编译并加载 `install/setup.bash` 后，使用一键预设启动建图：

```bash
ros2 run tb4_experiment_bringup vmware_simulation slam:=true
```

该命令会在 `~/.cache/tb4_experiment_bringup/vmware/` 自动创建用户级临时
overlay，不修改仓库或 `/opt/ros`，并执行以下低负载设置：

- Gazebo server-only，使用 Mesa 软件渲染保证 GPU lidar 数据正确；
- RViz 不继承软件渲染变量，继续使用 VMware 3D 加速；
- warehouse `max_step_size` 设为 `0.01`、墙钟更新率设为 `20 Hz`，
  将 `real_time_factor` 限制为 `0.2`；
- Create 3 controller 更新率由 `1000 Hz` 降为与物理步长匹配的 `100 Hz`，
  里程计发布率降为 `20 Hz`；
- OAK-D 停用，RPLIDAR 保持启用并降为 `10 Hz`；
- 禁用导航不依赖的 Create 3 cliff/IR GPU lidar 及其 ROS 桥接，保留传感器几何；
- 不生成充电底座，并停用 TurtleBot 4 HMI/传感器生命周期节点，避免离开底座时
  启停 OAK-D/RPLIDAR；Create 3 运动控制、里程计和 TF 保持启用；
- 关闭 RPLIDAR 射线可视化；
- SLAM 模式默认不启动不需要的 Nav2 规划和控制服务器；
- 启动后自动观察 `/clock` 120 秒并检查 `/scan`；通过后 watchdog 在整个仿真
  期间继续运行，时钟连续 15 秒不推进即失败。有效扫描必须包含大于最小量程的
  环境返回。

验证成功时启动终端会显示：

```text
VMware preset validated: /clock advanced for 120 wall seconds and /scan contains ...
```

需要手动复核时，在独立终端依次运行以下命令；每条看到 `average rate` 后按
`Ctrl+C`：

```bash
ros2 topic hz /clock
ros2 topic hz /odom
ros2 topic hz /scan
```

SLAM 证据只需展示地图随有效扫描增量扩展，不要求完整探索 warehouse。应覆盖
起点、目标和二者之间的通道特征，再保存用于定位的地图。

保存地图后，用同一预设启动 AMCL、A* 和 Nav2：

```bash
ros2 run tb4_experiment_bringup vmware_simulation \
  map:=$HOME/robot/experiment2/maps/lab_map.yaml
```

如果只验证建图而不需要 RViz，可额外传入 `start_rviz:=false`。若需在 SLAM
期间同时启动 Nav2，可显式传入 `start_nav2:=true`。软件渲染下仿真可能慢于
现实时间，但只要 `/clock` 持续推进，就不会改变使用仿真时间的 SLAM/Nav2
算法逻辑。

## 实机运行

现场电脑需要 Ubuntu 22.04、ROS 2 Humble、`turtlebot4_desktop` 和本工作空间。
完成机器人网络及 Discovery Server 配置后：

```bash
source /opt/ros/humble/setup.bash
source experiment2/ros2_ws/install/setup.bash
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 launch tb4_experiment_bringup real_robot.launch.py \
  map:=/absolute/path/to/lab_map.yaml
```

实机重新建图时使用：

```bash
ros2 launch tb4_experiment_bringup real_robot.launch.py slam:=true
```

仿真使用 `use_sim_time=true`，实机启动文件会切换为系统时间。

## 仿真结果

已完成两个独立流程：

1. SLAM Toolbox 增量建图：机器人移动 `0.550 m`，已知栅格增加 `4,858`，
   保存地图分辨率为 `0.05 m/cell`。
2. AMCL + 自定义 A* + Nav2 导航：路径包含 `189` 个位姿，绕过 `shelf_7`；
   同一目标 UUID 从 `EXECUTING (2)` 进入 `SUCCEEDED (4)`，最终距目标
   `0.215 m`。

低配测试主机的导航流程出现过一次超过 10 秒的 `/clock` 监听间隙，随后时钟恢复并
完成导航。完整实验条件、低负载配置、客观数据和限制说明见
[`report/实验二-TurtleBot4自主导航仿真-实验报告.pdf`](report/实验二-TurtleBot4自主导航仿真-实验报告.pdf)。

## 重新生成报告

```bash
sudo apt install pandoc texlive-xetex texlive-lang-chinese python3-docx
cd experiment2/report
make all
```

Word 报告由 Pandoc 直接读取 `report.tex` 生成，再应用 `reference.docx` 的中文论文
样式；PDF 使用 XeLaTeX 编译。
