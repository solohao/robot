# TurtleBot4 A* 自主导航端到端测试计划（低负载最终版）
## 目标
 
在 TurtleBot4 官方 warehouse 世界中，通过 RViz2 完成 AMCL 初始化和
Nav2 Goal 操作，证明 Humble 默认 `GridBased` 标识实际动态加载
`tb4_astar_planner/AStarPlanner`，生成绕开 `shelf_7` 的完整路径，并使
机器人到达目标、同一 action 进入 `SUCCEEDED (4)`。 


这是唯一的主流程；若任一关键断言无法客观验证，标记为 `failed` 或
`inconclusive`，不以“已开始运动”替代完整通过。


## 代码与 UI 路径依据

- `simulation.launch.py` 将 `world`、`model` 和 `gz_args` 传给官方
  TurtleBot4 Ignition 启动，并关闭重复 RViz。
- `simulation.launch.py:71-90` 使用项目内 `nav2_astar.yaml` 启动 Nav2，
  再启动官方 `view_robot.launch.py`。
- `nav2_astar.yaml:218-229` 将 `GridBased` 映射到
  `tb4_astar_planner/AStarPlanner`。
- 官方 `robot.rviz:275-300` 将红色全局路径显示绑定到 `/plan`；
  `robot.rviz:538-556` 配置 `SetInitialPose` 和 Nav2 `GoalTool`。
- 官方 `warehouse.sdf:113-118` 将 `shelf_7` 放在 `(0.4, -2.0)`。
- 官方 `oakd.urdf.xacro:123-139` 定义非导航 RGB-D 传感器；RPLIDAR 在
  独立的 `rplidar.urdf.xacro` 中。

  
## VMware 低负载测试环境


标准世界使用 `max_step_size=0.003`，当前 2 核 VM 无法在合理墙钟时间内
完成导航。仓库提供一键预设，只在用户缓存目录创建临时资源，不修改仓库或
`/opt/ros/humble`：

```bash
source /opt/ros/humble/setup.bash
source experiment2/ros2_ws/install/setup.bash
ros2 run tb4_experiment_bringup vmware_simulation
```

预设执行以下调整：

1. 缓存目录中的 `warehouse-low-resource.sdf` 将：

   ```xml
   <max_step_size>0.003</max_step_size>
   <real_time_factor>1.0</real_time_factor>
   ```

   改为：
   ```xml
   <max_step_size>0.01</max_step_size>
   <real_time_factor>0.2</real_time_factor>
   <real_time_update_rate>20</real_time_update_rate>
   ```
 
2. 临时 ament overlay 完整复制官方 `turtlebot4_description`，只将 OAK-D
   Gazebo sensor 设为：
 
   ```xml
   <always_on>0</always_on>
   <update_rate>1</update_rate>
   <visualize>false</visualize>
   ```
 
   OAK-D 链接、关节、碰撞体和 TF 均保留；展开后的标准机器人 URDF 必须仍有
   `rplidar` GPU lidar，且其 `always_on` 为 `true`。

3. 同一 overlay 复制官方 `irobot_create_control`、`irobot_create_description`
   和 `irobot_create_ignition_bringup`，关闭导航不依赖的 Create 3 cliff 和
   IR intensity GPU lidar 及其 ROS 桥接；传感器几何、里程计、TF、底盘控制和
   RPLIDAR 保持不变。

4. warehouse 使用 `0.01 s` 物理步长、`20 Hz` 墙钟更新率和 `0.2`
   real-time factor；Create 3 controller 使用匹配的 `100 Hz` 更新率和 `20 Hz`
   里程计发布率。将缓存 overlay 放在 ament 搜索路径首位，Gazebo 使用 Mesa
   软件渲染并以 server-only 模式启动；作用域外的 RViz 继续使用 VMware 3D 加速。

5. 启动检查器在 180 秒墙钟超时内持续观察 `/clock` 120 秒；时钟连续
   15 秒不推进即失败，同时要求 `/scan` 包含大于 `range_min + 0.05 m`
   的有限环境返回。通过后 watchdog 持续运行到仿真退出。SLAM 模式默认关闭
   Nav2 规划和控制服务器，导航模式保持启用。
 
此调整仍保留 warehouse/`shelf_7` 碰撞体、TurtleBot4 几何、轮式动力学、
里程计、TF、RPLIDAR、AMCL、全局/局部代价地图、Planner Server、DWB
Controller、行为树和自定义 A* pluginlib 动态加载。
 
## 执行前速度门槛
 
在录屏前采集 30 秒 `/clock`，用首末仿真时间差除以墙钟时间计算实时因子。
 
通过标准：
 
```text
RTF >= 0.20
```

这保证 180 秒墙钟内至少推进 36 秒仿真时间。若低于 `0.20`，停止本轮并重新
调整计划，不能开始一个注定超时的最终录屏。
## 证据采集
 
目标发送前启动并保存到
`/home/ubuntu/tb4-test-artifacts/final-run-5/`：
 
```bash
ros2 topic echo /navigate_to_pose/_action/status \
  --qos-profile action_status_default \
  --no-daemon --spin-time 10.0 \
  > navigation-status.yaml
```
```bash
ros2 topic echo /plan --full-length --once \
  --no-daemon --spin-time 10.0 \
  > plan.yaml
```
 
必须使用 `--full-length`；否则 ROS CLI 会在 128 个元素后加入 `...`。


同时保存：
 
```bash
ros2 topic info /plan -v > plan-topic-info.txt
```

通过标准：`/plan` 恰有一个发布者，节点名为 `planner_server`。
 ## 单一端到端 GUI 流程
 
### 1. AMCL 初始化
 
1. 最大化并聚焦 RViz2。
2. 开始录屏，添加 `setup`：
   `Warehouse loaded; initializing AMCL in RViz`。
3. 添加 `test_start`：
   `It should navigate around shelf_7 with custom AStarPlanner`。
4. 按 `p` 激活 `2D Pose Estimate`。
5. 在可见机器人模型中心按下并沿其正向轴拖动短箭头后释放。
6. 等待粒子云集中和 LaserScan 与地图边缘基本对齐。
7. 运行：
 
   ```bash
   timeout 120 ros2 run tf2_ros tf2_echo map odom --once
   ```
 
通过标准：
 
- `map -> odom` 返回含 Translation 和 Rotation 的有效 Transform；
- 粒子云集中在机器人附近；
- RViz Global Status 不再持续 Error；
- LaserScan 与已知地图边缘基本对齐。
 
断言注释：`AMCL published map-to-odom and aligned the laser scan`。
 
### 2. 自定义规划器加载
 
运行：
 
```bash
ros2 param get --no-daemon --spin-time 10.0 \
  /planner_server planner_plugins
ros2 param get --no-daemon --spin-time 10.0 \
  /planner_server GridBased.plugin
ros2 lifecycle get --no-daemon --spin-time 10.0 \
  /planner_server
```
 通过标准必须逐字包含：
 
```text
String values are: ['GridBased']
String value is: tb4_astar_planner/AStarPlanner
active [3]
```
 仿真日志不得包含：
 
```text
planner GridBased is not a valid planner
```
 断言注释：
`GridBased resolves to custom AStarPlanner; server active`。
 
### 3. RViz 发送目标并验证完整 A* 路径
 
1. 按 `g` 激活 `Nav2 Goal`。
2. 在地图坐标约 `(-0.14, -3.93)`、`shelf_7` 南侧按下，向南拖动短箭头
   并释放。
3. 等待 `/plan --full-length --once` 保存完成并解析全部 pose。
 
通过标准：
- pose 数量 `>= 3`；
- 首 pose 与机器人起点距离 `<= 0.30 m`；
- 末 pose 与目标 `(-0.14, -3.93)` 距离 `<= 0.35 m`；
- 在 `y=[-2.3,-1.7]` 的每个路径点均不位于
  `x=[-1.9,1.7]` 内；
- 全路径 `max(|x|) >= 2.0 m`；
- RViz 红色全局路径明显绕过 `shelf_7`，不穿过灰色占用区或粉色膨胀区；
- `/plan` 发布者为 `planner_server`。
 断言注释：`A* path detours outside shelf_7 collision band`。
 ### 4. 导航终态
 保持 RViz2 可见，目标发送后最多等待 180 秒墙钟。
 
通过标准：
 - 机器人沿红色全局路径和蓝色局部路径运动；
- 机器人不进入 `shelf_7` 占用区、不发生可见碰撞；
- 不持续原地旋转，不进入恢复失败；
- 最终机器人与目标距离 `<= 0.35 m`；
- 同一 goal UUID 从 `EXECUTING (2)` 进入 `SUCCEEDED (4)`；
- 同一 goal UUID 不出现 `ABORTED (6)`。
 断言注释：`Robot reached the RViz goal; action status SUCCEEDED`。
 若 180 秒后没有 `SUCCEEDED (4)`，本项标记为 `failed` 或
`inconclusive`。
 ## GUI 证据与报告
 
保存完整、未裁剪截图：
 
1. `screenshots/01-amcl-localized.png`；
2. `screenshots/02-astar-shelf-detour.png`；
3. `screenshots/03-goal-reached.png`。

停止录屏并创建：

```text
/home/ubuntu/tb4-test-artifacts/final-run-5/test-report.md
```

报告内嵌上述截图，逐项列出 `passed`、`failed`、`untested` 或
`inconclusive`，披露 `max_step_size=0.1` 和 OAK-D 临时停用，并明确真实
TurtleBot4、Discovery Server 和实验室网络未在当前环境测试。 
