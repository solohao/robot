#!/usr/bin/env python3

from docx import Document
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT, WD_TABLE_ALIGNMENT
from docx.enum.text import (
    WD_ALIGN_PARAGRAPH,
    WD_BREAK,
    WD_TAB_ALIGNMENT,
    WD_TAB_LEADER,
)
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor


REPORT = "实验二-TurtleBot4自主导航仿真-实验报告.docx"

FIGURE_CAPTIONS = [
    "TurtleBot4 仿真自主导航系统架构",
    "SLAM 建图运动前的局部地图",
    "SLAM 建图运动后的增量地图",
    "增量建图客观指标",
    "SLAM 流程保存的占据栅格地图",
    "最终复现实验的 AMCL 初始定位与导航激活界面",
    "A* 全局路径与 shelf_7 绕障判定",
    "完整导航录屏中的 A* 全局路径与执行状态",
    "自主导航终态：NavigateToPose action 返回 SUCCEEDED",
]

TABLE_CAPTIONS = [
    "实验平台配置",
    "仿真代码包主要文件",
    "自定义 A* 与 Nav2 关键参数",
    "SLAM 流程量化结果",
    "全局路径量化结果",
    "仿真自主导航最终验收",
]

TABLE_DATA = [
    (
        [4.0, 11.0],
        [
            ["项目", "配置"],
            ["操作系统", "Ubuntu 22.04 LTS"],
            ["机器人中间件", "ROS 2 Humble Hawksbill"],
            ["仿真器", "Ignition Gazebo / Gazebo Sim，TurtleBot4 warehouse 世界"],
            ["可视化工具", "RViz2，实验专用固定俯视配置 experiment.rviz"],
            ["建图与定位", "SLAM Toolbox；Nav2 Map Server；AMCL"],
            ["导航框架", "Nav2 BT Navigator、Planner Server、DWB Controller、Velocity Smoother"],
            ["全局规划算法", "自研 tb4_astar_planner/AStarPlanner pluginlib 插件"],
            [
                "测试主机",
                "约 6 vCPU、7.7 GiB 内存、无独立 GPU；RViz 使用 VMware SVGA3D，"
                "Gazebo 使用 Mesa llvmpipe",
            ],
        ],
    ),
    (
        [6.3, 8.7],
        [
            ["路径", "功能"],
            ["tb4_astar_planner/src/astar_search.cpp", "与 ROS 解耦的八连通代价感知 A* 搜索核心。"],
            [
                "tb4_astar_planner/src/astar_planner.cpp",
                "Nav2 GlobalPlanner 适配、坐标转换、路径姿态生成和 pluginlib 导出。",
            ],
            ["tb4_astar_planner/global_planner_plugin.xml", "插件类型与基类注册信息。"],
            ["tb4_astar_planner/test/test_astar_search.cpp", "空地图、墙体缺口、无路、禁止穿角和高代价绕行单元测试。"],
            [
                "tb4_experiment_bringup/launch/simulation.launch.py",
                "Ignition、SLAM/AMCL、Nav2 与 RViz 的统一仿真入口。",
            ],
            [
                "tb4_experiment_bringup/config/nav2_astar.yaml",
                "Planner、Controller、Costmap、Smoother 与 BT Navigator 参数。",
            ],
            ["tb4_experiment_bringup/config/experiment.rviz", "固定 map frame、俯视方向、初始位姿工具和目标工具。"],
            ["maps/lab_map.yaml、maps/lab_map.pgm", "SLAM 流程保存的增量地图。"],
        ],
    ),
    (
        [4.5, 5.2, 5.3],
        [
            ["参数", "设置值", "作用"],
            ["planner_plugins", "[GridBased]", "规划器映射名称"],
            ["GridBased.plugin", "tb4_astar_planner/AStarPlanner", "自定义插件类型"],
            ["allow_unknown", "false", "不穿越未知栅格"],
            ["use_diagonal", "true", "使用八连通搜索"],
            ["cost_penalty", "2.0", "膨胀代价惩罚"],
            ["lethal_cost", "253", "致命障碍阈值"],
            ["robot_radius", "0.175 m", "代价地图机器人半径"],
            ["inflation_radius", "0.45 m", "障碍膨胀半径"],
            ["xy_goal_tolerance", "0.25 m", "目标位置容差"],
            ["smoothing_frequency", "20 Hz", "速度平滑频率"],
        ],
    ),
    (
        [5.2, 5.2, 4.6],
        [
            ["指标", "实测值", "判定"],
            ["SLAM Toolbox 节点数", "1", "通过"],
            ["AMCL 节点数", "0", "通过"],
            ["初始已知栅格数", "41,067", "—"],
            ["最终已知栅格数", "45,925", "—"],
            ["已知栅格增量", "4,858", "通过"],
            ["里程计位移", "0.550 m", "通过（阈值 0.50 m）"],
            ["地图分辨率", "0.05 m/cell", "通过"],
            ["地图文件", "YAML + PGM", "通过"],
            ["/clock 监听", "240.4 s 墙钟，状态 ok", "通过"],
        ],
    ),
    (
        [5.0, 5.4, 4.6],
        [
            ["指标", "实测值", "判定"],
            ["路径位姿数量", "189", "通过"],
            ["起点", "(0.000, 0.000) m", "与机器人初始位姿一致"],
            ["终点", "(-0.149, -3.924) m", "与目标一致"],
            ["路径长度", "6.872 m", "—"],
            ["最大横向绕行", "2.425 m", "通过（阈值 2.0 m）"],
            ["shelf 占用带违规点", "0", "通过"],
            ["/plan 发布者", "planner_server 唯一发布", "通过"],
        ],
    ),
    (
        [5.5, 6.1, 3.4],
        [
            ["验收项", "实测结果", "判定"],
            ["Ignition Gazebo 中机器人实际运动", "物理仿真、传感器与速度闭环运行", "通过"],
            ["AMCL 自定位", "map frame，TF 有效", "通过"],
            ["自定义全局规划器加载", "tb4_astar_planner/AStarPlanner", "通过"],
            ["货架绕障", "0 个 shelf 带违规点", "通过"],
            ["Nav2 动作终态", "同一 UUID：accepted → SUCCEEDED (4)", "通过"],
            ["目标位置容差", "0.25 m goal checker 已通过", "通过"],
            ["/clock 与 /scan 启动验证", "连续观察 120 s，watchdog 已启用", "通过"],
        ],
    ),
]


def add_page_break(paragraph):
    paragraph.add_run().add_break(WD_BREAK.PAGE)


def set_run_font(run, name, size=None, bold=None, color=None):
    run.font.name = name
    run._element.get_or_add_rPr().get_or_add_rFonts().set(qn("w:eastAsia"), name)
    if size is not None:
        run.font.size = Pt(size)
    if bold is not None:
        run.bold = bold
    if color is not None:
        run.font.color.rgb = RGBColor(*color)


def insert_paragraph_after(paragraph, text="", style=None):
    new_p = OxmlElement("w:p")
    paragraph._p.addnext(new_p)
    from docx.text.paragraph import Paragraph

    inserted = Paragraph(new_p, paragraph._parent)
    if text:
        inserted.add_run(text)
    if style:
        inserted.style = style
    return inserted


def enable_field_updates(document):
    settings = document.settings._element
    update = settings.find(qn("w:updateFields"))
    if update is None:
        update = OxmlElement("w:updateFields")
        settings.append(update)
    update.set(qn("w:val"), "true")


def style_cover(document):
    cover_values = {
        "组号：": "组号：________________",
        "课程名称：": "课程名称：机器人与智能系统综合实践",
        "姓名：": "姓名：________________",
        "院系：": "院系：________________",
        "专业：": "专业：________________",
        "学号：": "学号：________________",
        "指导老师：": "指导老师：________________",
        "实验平台：": "实验平台：ROS 2 Humble / TurtleBot4",
    }
    cover_title_map = {
        "本科实验报告": 26,
        "TurtleBot4 移动机器人自主导航与避障": 20,
        "Experiment 2: Autonomous Navigation of TurtleBot4": 14,
    }
    date_paragraph = None
    for paragraph in document.paragraphs:
        text = paragraph.text.strip()
        if text in cover_title_map:
            paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
            paragraph.paragraph_format.first_line_indent = Cm(0)
            paragraph.paragraph_format.space_after = Pt(12)
            for run in paragraph.runs:
                set_run_font(
                    run,
                    "黑体" if not text.startswith("Experiment") else "Times New Roman",
                    cover_title_map[text],
                    bold=True,
                )
        elif text.startswith(("组号：", "课程名称：", "姓", "院", "专", "学", "指导老师：", "实验平台：")):
            if text in cover_values:
                paragraph.text = cover_values[text]
            paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
            paragraph.paragraph_format.first_line_indent = Cm(0)
            paragraph.paragraph_format.space_after = Pt(6)
            for run in paragraph.runs:
                set_run_font(run, "宋体", 12)
        elif text == "2026 年 7 月 12 日":
            paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
            paragraph.paragraph_format.first_line_indent = Cm(0)
            for run in paragraph.runs:
                set_run_font(run, "宋体", 12)
            date_paragraph = paragraph
            break
    if date_paragraph is not None:
        add_page_break(date_paragraph)
    section = document.sections[0]
    section.different_first_page_header_footer = True
    section.first_page_header.paragraphs[0].text = ""
    section.first_page_footer.paragraphs[0].text = ""


def prepare_bibliography(document):
    paragraphs = document.paragraphs
    start = next(
        index
        for index, paragraph in enumerate(paragraphs)
        if paragraph.text.strip().startswith("99 Clearpath Robotics")
    )
    paragraphs[start].insert_paragraph_before("参考文献", "Heading 1")
    for number, paragraph in enumerate(paragraphs[start:start + 7], start=1):
        if number == 1:
            for run in paragraph.runs:
                if run.text.strip() == "99":
                    run.text = ""
        for run in paragraph.runs:
            if run.text:
                run.text = f"[{number}] {run.text}"
                break
        paragraph.paragraph_format.left_indent = Cm(0.74)
        paragraph.paragraph_format.first_line_indent = Cm(-0.74)


def number_headings(document):
    chapter = 0
    subsection = 0
    current = ""
    for paragraph in document.paragraphs:
        text = paragraph.text.strip()
        if paragraph.style.name == "Heading 1" and text not in {"摘要", "目录"}:
            subsection = 0
            if text == "仿真复现命令":
                current = "A"
                prefix = "附录 A"
            elif text == "报告生成方法":
                current = "B"
                prefix = "附录 B"
            else:
                chapter += 1
                current = str(chapter)
                prefix = current
            paragraph.text = f"{prefix}  {text}"
        elif paragraph.style.name == "Heading 2" and current:
            subsection += 1
            paragraph.text = f"{current}.{subsection}  {text}"


def insert_contents_page(document):
    keywords = next(
        paragraph
        for paragraph in document.paragraphs
        if paragraph.text.strip().startswith("关键词：")
    )
    add_page_break(keywords)
    title = insert_paragraph_after(keywords, "目录", "Heading 1")
    title.alignment = WD_ALIGN_PARAGRAPH.CENTER
    title.paragraph_format.first_line_indent = Cm(0)
    entries = [
        (1, "1  实验目的与要求", 4),
        (2, "1.1  实验目的", 4),
        (2, "1.2  仿真实验要求", 4),
        (2, "1.3  实验范围说明", 4),
        (1, "2  实验平台与 TurtleBot4 工作原理", 4),
        (2, "2.1  软硬件与仿真平台", 4),
        (2, "2.2  TurtleBot4 主要硬件构成", 5),
        (2, "2.3  远程主机连接原理", 5),
        (1, "3  自主导航系统设计", 6),
        (2, "3.1  总体架构", 6),
        (2, "3.2  SLAM Toolbox 增量建图", 6),
        (2, "3.3  AMCL 自定位", 6),
        (2, "3.4  代价感知 A* 全局路径规划", 7),
        (2, "3.5  Nav2 闭环执行", 7),
        (1, "4  软件实现", 7),
        (2, "4.1  代码包结构", 7),
        (2, "4.2  仿真启动模式", 8),
        (2, "4.3  关键导航参数", 8),
        (1, "5  实验步骤", 8),
        (2, "5.1  编译与静态验证", 8),
        (2, "5.2  SLAM 增量建图", 8),
        (2, "5.3  AMCL 初始化与规划器检查", 9),
        (2, "5.4  发送导航目标", 9),
        (1, "6  实验结果与分析", 10),
        (2, "6.1  SLAM 增量建图结果", 10),
        (2, "6.2  AMCL 自定位结果", 12),
        (2, "6.3  A* 路径与绕障结果", 13),
        (2, "6.4  Nav2 导航终态", 15),
        (2, "6.5  仿真时钟与低负载配置分析", 16),
        (1, "7  问题诊断与改进", 17),
        (2, "7.1  启动参数作用域", 17),
        (2, "7.2  RViz 视角确定性", 17),
        (2, "7.3  慢速主机的 lifecycle 启动顺序", 17),
        (2, "7.4  后续优化方向", 17),
        (1, "8  结论", 17),
        (1, "9  参考文献", 18),
        (1, "附录 A  仿真复现命令", 19),
        (1, "附录 B  报告生成方法", 19),
    ]
    previous = title
    for level, name, page in entries:
        entry = insert_paragraph_after(previous, f"{name}\t{page}")
        entry.paragraph_format.left_indent = Cm(0.74 if level == 2 else 0)
        entry.paragraph_format.first_line_indent = Cm(0)
        entry.paragraph_format.space_before = Pt(0)
        entry.paragraph_format.space_after = Pt(0)
        entry.paragraph_format.line_spacing = 1.0
        entry.paragraph_format.tab_stops.add_tab_stop(
            Cm(15.0), WD_TAB_ALIGNMENT.RIGHT, WD_TAB_LEADER.DOTS
        )
        for run in entry.runs:
            set_run_font(run, "宋体", 9.5, bold=level == 1)
        previous = entry
    add_page_break(previous)


def number_captions(document):
    for paragraph in document.paragraphs:
        text = paragraph.text.strip()
        if text in FIGURE_CAPTIONS:
            number = FIGURE_CAPTIONS.index(text) + 1
            paragraph.text = f"图 {number}  {text}"
            paragraph.style = "Caption"
            paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
            paragraph.paragraph_format.first_line_indent = Cm(0)
        elif text in TABLE_CAPTIONS:
            number = TABLE_CAPTIONS.index(text) + 1
            paragraph.text = f"表 {number}  {text}"
            paragraph.style = "Caption"
            paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
            paragraph.paragraph_format.first_line_indent = Cm(0)
            paragraph.paragraph_format.keep_with_next = True
            if number in {1, 4, 5, 6}:
                paragraph.paragraph_format.page_break_before = True


def set_cell_width(cell, width):
    cell.width = width
    properties = cell._tc.get_or_add_tcPr()
    tc_width = properties.find(qn("w:tcW"))
    if tc_width is None:
        tc_width = OxmlElement("w:tcW")
        properties.append(tc_width)
    tc_width.set(qn("w:w"), str(width.twips))
    tc_width.set(qn("w:type"), "dxa")


def replace_tables(document):
    old_tables = list(document.tables)
    if len(old_tables) != len(TABLE_DATA):
        raise RuntimeError(
            f"Expected {len(TABLE_DATA)} report tables, found {len(old_tables)}"
        )
    for old_table, (widths, rows) in zip(old_tables, TABLE_DATA):
        new_table = document.add_table(rows=len(rows), cols=len(widths))
        new_table.autofit = False
        for row_index, values in enumerate(rows):
            for column_index, value in enumerate(values):
                cell = new_table.cell(row_index, column_index)
                cell.text = value
                set_cell_width(cell, Cm(widths[column_index]))
        old_table._tbl.addprevious(new_table._tbl)
        old_table._element.getparent().remove(old_table._element)


def style_body(document):
    for paragraph in document.paragraphs:
        if paragraph.style.name == "Source Code":
            paragraph.paragraph_format.first_line_indent = Cm(0)
            paragraph.paragraph_format.space_before = Pt(4)
            paragraph.paragraph_format.space_after = Pt(4)
            for run in paragraph.runs:
                set_run_font(run, "Consolas", 9)
        elif paragraph.style.name.startswith("Heading"):
            paragraph.paragraph_format.first_line_indent = Cm(0)
        elif paragraph.style.name == "Caption":
            paragraph.paragraph_format.first_line_indent = Cm(0)
        elif paragraph.text.strip() and paragraph.text.strip() not in {
            "本科实验报告",
            "TurtleBot4 移动机器人自主导航与避障",
            "Experiment 2: Autonomous Navigation of TurtleBot4",
            "目录",
        }:
            if paragraph.alignment != WD_ALIGN_PARAGRAPH.CENTER:
                paragraph.paragraph_format.line_spacing = 1.5

    for shape in document.inline_shapes:
        parent = shape._inline.getparent().getparent()
        from docx.text.paragraph import Paragraph

        Paragraph(parent, document._body).alignment = WD_ALIGN_PARAGRAPH.CENTER


def style_tables(document):
    for table in document.tables:
        table.alignment = WD_TABLE_ALIGNMENT.CENTER
        table.style = "Table Grid"
        for row_index, row in enumerate(table.rows):
            tr_properties = row._tr.get_or_add_trPr()
            if tr_properties.find(qn("w:cantSplit")) is None:
                tr_properties.append(OxmlElement("w:cantSplit"))
            for cell in row.cells:
                cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
                for paragraph in cell.paragraphs:
                    paragraph.paragraph_format.first_line_indent = Cm(0)
                    paragraph.paragraph_format.space_after = Pt(0)
                    paragraph.paragraph_format.line_spacing = 1.15
                    paragraph.paragraph_format.keep_with_next = (
                        row_index < len(table.rows) - 1
                    )
                    if row_index == 0:
                        paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
                    for run in paragraph.runs:
                        set_run_font(run, "宋体", 8.5, bold=row_index == 0)


def main():
    document = Document(REPORT)
    prepare_bibliography(document)
    number_headings(document)
    style_cover(document)
    insert_contents_page(document)
    replace_tables(document)
    number_captions(document)
    style_body(document)
    style_tables(document)
    enable_field_updates(document)
    document.core_properties.title = "TurtleBot4 移动机器人自主导航与避障"
    document.core_properties.subject = "机器人与智能系统综合实践·实验二"
    document.core_properties.keywords = "TurtleBot4, ROS 2, Gazebo, SLAM, AMCL, Nav2, A*"
    document.save(REPORT)


if __name__ == "__main__":
    main()
