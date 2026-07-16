# 实验二报告

本目录生成《TurtleBot4 移动机器人自主导航与避障》完整实验报告：

- `report.tex`：报告内容源，包含已完成的计算机仿真和实机问题、调参及视频结论；
- `template/实验报告模板.docx`：用户提供的浙江大学实验报告模板；
- `extract_template_assets.py`：通过 OpenXML 读取模板页面参数、字段标签和校名字；
- `build_report_docx.py`：以原模板 DOCX 为主文档，直接保留前两页原始 OpenXML、
  浮动对象、段落和宋体样式，再通过 `docxcompose` 拼接 Pandoc 正文；
- `generate_figures.py`：从原始路径、JSON、时钟和视频数据独立复算并生成图表；
- `data/red-blue/`：红点到蓝点运行的原始路径、日志和验收证据；
- `figures/red_blue_*.png`：红蓝点路线图、指标图和五张原始运行截图。

当前仿真结果为：

| 项目 | 结果 |
|---|---:|
| 起点 | `(-11.25, -10.50)` |
| 目标 | `(0.40, -10.50)` |
| 路径位姿 | `787` |
| 路径长度 | `24.489 m` |
| 绕行比 | `2.102` |
| 最大横向绕行 | `7.055 m` |
| 直线占用区段 | `4` |
| action 终态 | `SUCCEEDED (4)` |
| action 墙钟时长 | `1248.075 s` |

实机实验已录制视频并最终通过。报告记录了 Discovery Server 环境下 AMCL 与
map_server lifecycle 激活超时、手动激活流程，以及窄直角弯中将 local/global
costmap 的 `inflation_radius` 从 `0.45 m` 调整为 `0.30 m`、`cost_scaling_factor`
从 `4.0` 调整为 `2.5` 的处理。未提供的 rosbag、路径、终点误差、action 状态码和
重复成功率仍明确标记为待补，不以视频推断不存在的定量数据。

## 生成

Ubuntu 22.04：

```bash
sudo apt install pandoc libreoffice-writer poppler-utils python3-docx python3-lxml
python3 -m pip install --user -r requirements-report.txt
make docx
make pdf
```

输出文件：

```text
实验二-TurtleBot4自主导航与避障-完整实验报告.docx
实验二-TurtleBot4自主导航与避障-完整实验报告.pdf
```

封面和第二页团队信息保持空白占位符，提交前填写组号、姓名、院系、专业、学号、
指导老师、日期、地点和同组学生姓名。
