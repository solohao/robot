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
