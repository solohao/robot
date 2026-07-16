# TurtleBot4 实机实验现有记录

## 证据范围

- 平台：Ubuntu 22.04、ROS 2 Humble、TurtleBot4；
- 用户确认实机实验已录制连续视频并最终通过；
- 当前未提供视频文件、rosbag、完整 `/plan`、终态 TF、action result 或重复试验统计；
- 因此报告可以记录功能通过、故障现象和修复方法，但不能补写路径长度、终点误差、
  action 状态码、运行耗时或成功率。

## 地图与定位故障

在 Discovery Server 环境下，`lifecycle_manager_localization` 调用
`/amcl/change_state` 时可能超时，AMCL 或 `map_server` 停留在 `inactive [2]`。
AMCL 未 active 时不会广播 `map -> odom`，RViz 初始位姿请求也不会生效。

验证与手动激活命令：

```bash
sleep 5
ros2 lifecycle get /amcl
ros2 lifecycle get /map_server
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map base_link
```

`configure` 在节点已处于 inactive 时可能返回 `Unknown transition`；随后执行
`activate` 即可。地图、LaserScan 与 TF 正常后再发布 Nav2 Goal。

## 窄直角弯故障

现场路线由泡沫垫和纸箱形成约 `0.7 m` 的窄直角弯。原配置：

```yaml
inflation_layer:
  cost_scaling_factor: 4.0
  inflation_radius: 0.45
```

机器人半径约 `0.175 m`，原膨胀配置在窄弯两侧叠加后使通道近似封闭，机器人在
拐角附近停转、后退，`backup` 与 `spin` recovery 报 `Collision Ahead`。

实机通过时使用：

```yaml
inflation_layer:
  cost_scaling_factor: 2.5
  inflation_radius: 0.30
```

local/global costmap 均采用上述值。重新构建与启动命令：

```bash
cd ~/experiment2/ros2_ws
colcon build --symlink-install --packages-select tb4_experiment_bringup
source install/setup.bash
ros2 launch tb4_experiment_bringup real_robot.launch.py \
  map:=/home/lex/map/map.yaml
```
