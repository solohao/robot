# 实验二：TurtleBot4 移动机器人自主导航与避障

本目录包含 ROS 2 Humble 下的 TurtleBot4 仿真、SLAM、定位、自主导航，以及基于
Nav2 `GlobalPlanner` 接口实现的代价感知 A* 全局路径规划器。

## 目录

- `ros2_ws/src/tb4_astar_planner/`：A* 规划器插件与单元测试
- `ros2_ws/src/tb4_experiment_bringup/`：仿真和实机启动文件、Nav2 参数
- `maps/`：SLAM 保存的地图
- `report/`：实验报告与结果图
- `video/`：实验视频或视频说明

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
