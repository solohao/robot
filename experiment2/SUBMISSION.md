# 实验二仿真提交资料包

## 提交内容

| 内容 | 路径 |
|---|---|
| ROS 2 仿真源码 | `ros2_ws/src/` |
| 自定义 A* 规划器 | `ros2_ws/src/tb4_astar_planner/` |
| Ignition/SLAM/AMCL/Nav2/RViz 启动与配置 | `ros2_ws/src/tb4_experiment_bringup/` |
| VMware 一键低负载预设 | `ros2 run tb4_experiment_bringup vmware_simulation` |
| SLAM 保存地图 | `maps/lab_map.yaml`、`maps/lab_map.pgm` |
| LaTeX 报告源文件 | `report/report.tex` |
| Word 实验报告 | `report/实验二-TurtleBot4自主导航仿真-实验报告.docx` |
| PDF 实验报告 | `report/实验二-TurtleBot4自主导航仿真-实验报告.pdf` |
| 客观数据与图表 | `report/data/`、`report/figures/` |
| SLAM 与导航录屏 | `video/` |

## 功能验收摘要

- SLAM Toolbox 节点数：`1`；
- AMCL 在导航流程中发布 `map` frame 位姿；
- SLAM 里程计位移：`0.550 m`；
- 已知栅格增量：`4,858`；
- 地图分辨率：`0.05 m/cell`；
- 自定义规划器：`tb4_astar_planner/AStarPlanner`；
- A* 路径：`189` 个位姿，长度 `6.838 m`；
- `shelf_7` 占用带违规点：`0`；
- Nav2 同一目标状态：`EXECUTING (2) -> SUCCEEDED (4)`；
- 最终机器人到目标距离：`0.215 m`。

## 已知限制

低配测试主机约有 2 vCPU 且使用 llvmpipe 软件渲染。导航阶段的严格 `/clock`
连续性监视出现过一次超过 10 秒的监听间隙；时钟随后恢复并完成导航。该项在报告中
如实记录为性能限制，没有修改为通过。

证据采集使用低负载 world/overlay：Gazebo server-only、OAK-D 关闭、RPLIDAR
降至 10 Hz、Create 3 cliff/IR 辅助传感器降至 1 Hz、射线可视化关闭、
`max_step_size=0.1`。仓库提供
`vmware_simulation` 一键入口，在用户缓存目录生成 overlay，并将软件渲染仅限制
在 Gazebo 进程；RViz 仍使用 VMware 3D 加速。具备原生硬件 3D 加速的
Ubuntu 22.04 主机可直接使用默认 world 与传感器参数复测。

## 填写学生信息

Word 和 PDF 封面保留姓名、院系、专业、学号、指导老师和组号填写项。提交前请在
Word 中填写个人信息；若修改正文导致分页变化，应同步更新目录页码。
