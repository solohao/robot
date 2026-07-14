# 实验视频

本目录包含仿真部分录屏：

- `实验二-SLAM增量建图.mp4`：单一 SLAM Toolbox 节点、L 形运动、地图增长与保存；
- `实验二-自主导航.mp4`：助教复验版本的完整 443.934 秒录屏，从目标发送前连续展示
  RViz 中的地图、机器人、LaserScan、AMCL、13.460 m 全局路径、机器人运动和到达
  目标。对应 action 的 JSON 终态为 `SUCCEEDED`。

视频使用 H.264 MP4；自主导航视频为 `1600 × 1200`、`15 fps`。它由录屏工具保存的
四个连续原始 MKV 分段使用 FFmpeg concat demuxer 和 `-c copy` 无损拼接，未使用
加速、抽帧、trim 或其他视频滤镜。客观数值与截图见 `../report/data/` 和
`../report/figures/`。真实 TurtleBot4 不在本次仿真资料包范围内。
