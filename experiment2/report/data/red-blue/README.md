# 红点至蓝点仿真证据

该目录保存 2026-07-15 红点至蓝点双货架导航运行的原始证据。

| 文件 | 说明 |
|---|---|
| `red-blue-plan-20260715-024010.yaml` | `/plan` 的 787 个原始位姿 |
| `red-blue-navigation-20260715-024010.json` | 路径几何、门槛、墙钟时长和 action 终态 |
| `red-blue-navigation-20260715-024010.log` | 接受目标、接收路径和最终 `[PASS]` |
| `red-blue-nav2-preconditions.log` | 自定义 A* 插件和 Nav2 lifecycle 状态 |
| `red-blue-localization.log` | 定位漂移和 LaserScan/地图匹配结果 |
| `red-blue-clock-stability.log` | 世界加载后的四个 `/clock` 采样窗口 |
| `red-blue-video-ffprobe.json` | 完整连续视频的编码、帧率和时长 |
| `test-report.md` | 测试断言、限制和原始截图索引 |

`generate_figures.py` 会重新读取完整 YAML，并验证复算值与 JSON 一致；若路径位姿数、
路径长度、直线距离、绕行比或最大横向偏移不一致，报告构建立即失败。
