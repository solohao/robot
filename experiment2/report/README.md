# 实验二报告

本目录生成《TurtleBot4 移动机器人自主导航与避障》完整实验报告：

- `report.tex`：报告内容源，包含已完成的计算机仿真和待填写的实地实验模板；
- `template/实验报告模板.docx`：用户提供的浙江大学实验报告模板；
- `extract_template_assets.py`：通过 OpenXML 读取模板页面参数、字段标签和校名字；
- `build_report_docx.py`：保留模板前两页团队信息结构，并将 Pandoc 正文合并为 Word；
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

实地实验尚未执行。报告中的实体设备、现场路线、误差、成功率、照片和视频字段均
明确标记为“待实测填写”，没有使用仿真数据代替实测数据。

## 生成

Ubuntu 22.04：

```bash
sudo apt install pandoc libreoffice-writer poppler-utils python3-docx python3-lxml
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
