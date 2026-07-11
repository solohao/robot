# 地图

本目录包含仿真 SLAM Toolbox 流程保存的地图：

- `lab_map.yaml`：地图元数据，分辨率 `0.05 m/cell`；
- `lab_map.pgm`：占据栅格图像。

生成命令：

```bash
ros2 run nav2_map_server map_saver_cli -t /map -f experiment2/maps/lab_map
```

该地图来自低负载 warehouse 仿真建图流程；导航终态验证使用与官方 warehouse 世界
匹配的静态地图，以避免 SLAM 建图轨迹有限造成的地图覆盖不足。

保存前必须确认 `/scan` 包含不同距离值或 `inf`，不能全部等于 RPLIDAR 最小量程
`0.164 m`。尺寸仅为 `8 × 23` 等极小地图不能用于 AMCL 或 Nav2。
