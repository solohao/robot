# 实验报告

本目录提供仿真部分的专业实验报告和可复现生成链路：

- `report.tex`：唯一报告内容源；
- `实验二-TurtleBot4自主导航仿真-实验报告.pdf`：XeLaTeX 排版 PDF；
- `实验二-TurtleBot4自主导航仿真-实验报告.docx`：由 Pandoc 从 LaTeX 转换的 Word；
- `reference.docx`：Word 中文字体、标题、页边距、页眉与页码样式；
- `generate_figures.py`：根据客观 JSON 数据生成系统架构、SLAM 指标和 A* 路径图；
- `make_reference_docx.py`、`polish_docx.py`：生成并完善 Word 样式、目录和图表编号；
- `data/astar-plan.yaml`、`data/path.json`：改进前短路径基线的原始消息与摘要；
- `data/acceptance-summary.json`：扩展 SLAM、定位稳定性、长距离 action 与视频验收数据；
- `data/action.json`、`data/final-distance.json`：目标 UUID、终态和目标容差证据；
- `data/navigation-clock.json`：120 秒 `/clock` 与有效 `/scan` 启动验证摘要；
- `data/slam-*.json`：独立 SLAM 增量建图流程数据；
- `figures/`：RViz 截图及报告图表。

生成命令：

```bash
sudo apt install pandoc texlive-xetex texlive-lang-chinese python3-docx
make all
```

Word 报告包含静态页码目录，适合直接编辑和提交。封面个人信息仍为占位符，提交前
必须填写；正文变化后需同步更新目录页码。最新版报告已整合 5.00 m 扩展 SLAM、
58,506 个已知栅格增量、0.965 激光地图匹配率、13.460 m 长路径及 443.934 s
无加速视频证据。
