# 实验报告

本目录提供仿真部分的专业实验报告和可复现生成链路：

- `report.tex`：唯一报告内容源；
- `实验二-TurtleBot4自主导航仿真-实验报告.pdf`：XeLaTeX 排版 PDF；
- `实验二-TurtleBot4自主导航仿真-实验报告.docx`：由 Pandoc 从 LaTeX 转换的 Word；
- `reference.docx`：Word 中文字体、标题、页边距、页眉与页码样式；
- `generate_figures.py`：根据客观 JSON 数据生成系统架构、SLAM 指标和 A* 路径图；
- `make_reference_docx.py`、`polish_docx.py`：生成并完善 Word 样式、目录和图表编号；
- `data/`：SLAM、路径、Nav2 action、最终位姿与 `/clock` 客观数据；
- `figures/`：RViz 截图及报告图表。

生成命令：

```bash
sudo apt install pandoc texlive-xetex texlive-lang-chinese python3-docx
make all
```

Word 报告包含静态页码目录，适合直接编辑和提交。
