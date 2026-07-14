# 地图

本目录包含仿真 SLAM Toolbox 流程保存的地图：

- `lab_map.yaml`：地图元数据，分辨率 `0.05 m/cell`；
- `lab_map.pgm`：改进前短程占据栅格图像；
- `extended_lab_map.yaml`：助教复验版本的地图元数据；
- `extended_lab_map.pgm`：`526 × 452`、`0.05 m/cell` 的扩展占据栅格图像。

生成命令：

```bash
ros2 run nav2_map_server map_saver_cli -t /map -f experiment2/maps/lab_map
ros2 run nav2_map_server map_saver_cli -t /map \
  -f experiment2/maps/extended_lab_map
```

该地图来自低负载 warehouse 仿真建图流程；导航终态验证使用与官方 warehouse 世界
匹配的静态地图，以避免 SLAM 建图轨迹有限造成的地图覆盖不足。

助教复验改用 `extended_lab_map.yaml` 和 `extended_lab_map.pgm`。该快照在
`validate_slam_coverage` 报告累计行程 `5.00 m`、已知栅格增量 `58,506` 后保存，
地图原点为 `[-14.9, -12.3, 0]`。

保存前必须确认 `/scan` 包含不同距离值或 `inf`，不能全部等于 RPLIDAR 最小量程
`0.164 m`。尺寸仅为 `8 × 23` 等极小地图不能用于 AMCL 或 Nav2。
