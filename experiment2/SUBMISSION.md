# 实验二仿真提交资料包

## 提交内容

| 内容 | 路径 |
|---|---|
| ROS 2 仿真源码 | `ros2_ws/src/` |
| 自定义 A* 规划器 | `ros2_ws/src/tb4_astar_planner/` |
| Ignition/SLAM/AMCL/Nav2/RViz 启动与配置 | `ros2_ws/src/tb4_experiment_bringup/` |
| VMware 一键低负载预设 | `ros2 run tb4_experiment_bringup vmware_simulation` |
| SLAM 保存地图 | `maps/lab_map.*`、`maps/extended_lab_map.*` |
| 实地建图地图 | `maps/real_robot_map.pgm` |
| LaTeX 报告源文件 | `report/report.tex` |
| Word 实验报告 | `report/实验二-TurtleBot4自主导航与避障-完整实验报告.docx` |
| PDF 实验报告 | `report/实验二-TurtleBot4自主导航与避障-完整实验报告.pdf` |
| 客观数据与图表 | `report/data/`、`report/figures/` |
| 仿真 SLAM 与导航录屏 | `video/实验二-SLAM增量建图.mp4`、`video/实验二-自主导航.mp4` |
| 实地建图与导航录屏 | `video/实验二-实地导航.mp4` |
| 提交压缩包生成脚本 | `create_submission_archive.sh` |

`report/data/astar-plan.yaml` 与 `path.json` 保留改进前短路径基线；
`acceptance-summary.json` 汇总扩展 SLAM、AMCL/TF/LaserScan 稳定性、长距离
NavigateToPose action 和完整视频数据。自主导航录像和三张新导航截图来自同一
最终 VMware 复现实验。

在仓库根目录运行以下命令可生成只包含课程材料的
`experiment2-submission.zip`；历史排障交接文档、ROS 构建产物和 LaTeX 中间文件
不会进入压缩包：

```bash
./experiment2/create_submission_archive.sh
```

## 功能验收摘要

- SLAM Toolbox 节点数：`1`；
- AMCL 在导航流程中发布 `map` frame 位姿；
- 扩展 SLAM 累计行程：`5.00 m`（门槛 `5.0 m`）；
- 已知栅格增量：`58,506`（门槛 `15,000`）；
- 地图分辨率：`0.05 m/cell`；
- 静止位置/航向漂移：`0.000 m / 0.000 rad`；
- LaserScan/地图匹配率：`0.965`（门槛 `0.35`）；
- 自定义规划器：`tb4_astar_planner/AStarPlanner`；
- 长距离目标：`(-7.0, -10.0, 0.0)`；
- A* 路径：`370` 个位姿，长度 `13.459614 m`；
- Nav2 同一 action：`385.981 s -> SUCCEEDED (4)`；
- 完整无加速视频：`443.934 s`，`1600 × 1200`，`15 fps`；
- `/clock` 与 `/scan`：连续启动验证 `120 s` 通过，环境回波
  `1.339--11.997 m`。

## 实地实验说明

实地实验在实体 TurtleBot4 上完成了 SLAM 增量建图、地图保存与自主导航，整体进展
顺利。实地部分以保存的占据栅格地图（`maps/real_robot_map.pgm`）和连续录像
（`video/实验二-实地导航.mp4`）作为辅助佐证；由于未导出可复算的路径 YAML、rosbag
与 action 终态 JSON，报告不给出实地路径长度、终点误差等量化数值，量化验收以仿真
部分为主。

## 已知限制

测试主机约有 6 vCPU、7.7 GiB 内存且无独立 GPU。证据采集使用低负载
world/overlay：Gazebo server-only 和 Mesa llvmpipe、OAK-D 停用、RPLIDAR
降至 10 Hz、Create 3 cliff/IR GPU lidar 及桥接关闭、不生成充电底座、
`max_step_size=0.01`、`real_time_update_rate=20`、`real_time_factor=0.2`，
controller/里程计分别为 `100/20 Hz`。`vmware_simulation` 只在用户缓存目录生成
overlay，不修改仓库或 `/opt/ros/humble`；RViz 仍使用 VMware 3D 加速。

低资源运行中 Nav2 曾输出 Behavior Tree tick rate 超限警告，但路径显示、机器人运动
和 action 没有失败。完整录屏只使用连续原始分段的无损拼接文件，不使用 20.959 秒
标注摘要。资料包中的结果是仿真验证，不代表实体 TurtleBot4 的实时性能或安全验证。

## 学生信息

封面与首页已填入以下信息（由 `report/build_report_docx.py` 中的 `STUDENT_INFO`
统一注入，重新执行 `make all` 会自动套用）：

- 课程名称：机器人与智能系统综合实践
- 组员姓名：刘峻豪、莫移合、蓝恩先
- 学号依次对应：3230105220、3230102397、3230104940
- 指导老师：任沁源、姚近科
- 专业：控制自动化
- 院系：控制学院

组号与选课时间未提供，仍保留封面空栏。若修改 Word 正文导致分页变化，应同步更新
静态目录页码，并重新导出最终 PDF。
