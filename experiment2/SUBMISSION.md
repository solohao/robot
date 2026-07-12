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
| 提交压缩包生成脚本 | `create_submission_archive.sh` |

`report/data/astar-plan.yaml` 是本次最终运行保存的完整 ROS 2 `/plan` 原始消息；
`path.json` 是由其 189 个位姿计算得到的摘要。自主导航录像和三张导航截图均来自
同一最终 VMware 复现实验。

在仓库根目录运行以下命令可生成只包含课程材料的
`experiment2-submission.zip`；历史排障交接文档、ROS 构建产物和 LaTeX 中间文件
不会进入压缩包：

```bash
./experiment2/create_submission_archive.sh
```

## 功能验收摘要

- SLAM Toolbox 节点数：`1`；
- AMCL 在导航流程中发布 `map` frame 位姿；
- SLAM 里程计位移：`0.550 m`；
- 已知栅格增量：`4,858`；
- 地图分辨率：`0.05 m/cell`；
- 自定义规划器：`tb4_astar_planner/AStarPlanner`；
- A* 路径：`189` 个位姿，长度 `6.872 m`；
- 最大横向绕行：`2.425 m`；
- `shelf_7` 占用带违规点：`0`；
- Nav2 同一目标：`Goal accepted -> SUCCEEDED (4)`；
- Nav2 目标检查器：`0.25 m` 位置容差内通过；
- `/clock` 与 `/scan`：连续启动验证 `120 s` 通过，环境回波
  `1.339--11.997 m`。

## 已知限制

测试主机约有 6 vCPU、7.7 GiB 内存且无独立 GPU。证据采集使用低负载
world/overlay：Gazebo server-only 和 Mesa llvmpipe、OAK-D 停用、RPLIDAR
降至 10 Hz、Create 3 cliff/IR GPU lidar 及桥接关闭、不生成充电底座、
`max_step_size=0.01`、`real_time_update_rate=20`、`real_time_factor=0.2`，
controller/里程计分别为 `100/20 Hz`。`vmware_simulation` 只在用户缓存目录生成
overlay，不修改仓库或 `/opt/ros/humble`；RViz 仍使用 VMware 3D 加速。

录屏中 RViz 曾短暂灰显后自行恢复；路径显示、机器人运动和 action 没有失败。资料包
只把已保存的 120 秒启动检查记为严格连续性证据，不把动作成功冒充未单独采集的全程
时钟曲线。本次录屏未同步保存终态 TF 数值，因此报告使用
`SUCCEEDED + 0.25 m goal checker`，不沿用其他运行的精确距离。

## 填写学生信息

Word 和 PDF 封面保留姓名、院系、专业、学号、指导老师和组号填写项。提交前必须
填写这些信息；若修改 Word 正文导致分页变化，应同步更新静态目录页码，并重新导出
最终 PDF。
