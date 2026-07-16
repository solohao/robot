#!/usr/bin/env python3

import argparse
from pathlib import Path

from docx import Document
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt
from docxcompose.composer import Composer


ROOT = Path(__file__).resolve().parent
TEMPLATE = ROOT / "template" / "实验报告模板.docx"
BODY_HEADING = "一、实验目的和要求（必填）"
TEMPLATE_BODY_HEADING = "一、实验目的和要求"


def element_text(element):
    text_tags = {qn("w:t"), qn("m:t")}
    return "".join(
        text.text or "" for text in element.iter() if text.tag in text_tags
    ).strip()


def normalized_text(element):
    return "".join(element_text(element).split()).replace("、", "")


def set_east_asia_font(run_or_style, font_name="宋体"):
    run_or_style.font.name = "Times New Roman"
    fonts = run_or_style._element.get_or_add_rPr().rFonts
    fonts.set(qn("w:ascii"), "Times New Roman")
    fonts.set(qn("w:hAnsi"), "Times New Roman")
    fonts.set(qn("w:eastAsia"), font_name)


def trim_template_to_front_matter(document):
    body = document._element.body
    for marker in body.iter(qn("w:lastRenderedPageBreak")):
        marker.getparent().remove(marker)

    page_break_paragraph = None
    for element in body:
        if element.tag != qn("w:p"):
            continue
        page_breaks = [
            page_break
            for page_break in element.iter(qn("w:br"))
            if page_break.get(qn("w:type")) == "page"
        ]
        if page_breaks:
            page_break_paragraph = element
            for page_break in page_breaks:
                page_break.getparent().remove(page_break)
            break
    if page_break_paragraph is None:
        raise ValueError("template page break not found")

    page_break_index = body.index(page_break_paragraph)
    if page_break_index == 0:
        raise ValueError("template page break has no preceding paragraph")
    break_run = OxmlElement("w:r")
    break_element = OxmlElement("w:br")
    break_element.set(qn("w:type"), "page")
    break_run.append(break_element)
    body[page_break_index - 1].append(break_run)

    target = "".join(TEMPLATE_BODY_HEADING.split()).replace("、", "")
    start = None
    for index, element in enumerate(body):
        if element.tag == qn("w:p") and normalized_text(element) == target:
            start = index
            break
    if start is None:
        raise ValueError(f"could not find template body heading {TEMPLATE_BODY_HEADING!r}")

    for element in list(body)[start:]:
        if element.tag != qn("w:sectPr"):
            body.remove(element)


def trim_to_report_body(document):
    body = document._element.body
    elements = list(body)
    start = None
    for index, element in enumerate(elements):
        if element.tag == qn("w:p") and element_text(element).startswith(BODY_HEADING):
            start = index
            break
    if start is None:
        raise ValueError(f"could not find report body heading {BODY_HEADING!r}")

    for element in elements[:start]:
        body.remove(element)


def repair_gridless_tables(document):
    section = document.sections[0]
    available_width = int(
        section.page_width.twips
        - section.left_margin.twips
        - section.right_margin.twips
    )
    fractions = {
        2: (0.34, 0.66),
        3: (0.36, 0.32, 0.32),
        4: (0.38, 0.21, 0.22, 0.19),
    }

    gridless_tables = [
        table
        for table in document._element.body.iter(qn("w:tbl"))
        if (
            table.find(qn("w:tblGrid")) is None
            or not len(table.find(qn("w:tblGrid")))
        )
    ]
    for table in gridless_tables:
        source_rows = table.findall(qn("w:tr"))
        source_cells = [
            [element_text(cell) for cell in row.findall(qn("w:tc"))]
            for row in source_rows
        ]
        column_count = max((len(row) for row in source_cells), default=0)
        if not source_cells or column_count == 0:
            table.getparent().remove(table)
            continue

        column_fractions = fractions.get(
            column_count,
            tuple(1.0 / column_count for _ in range(column_count)),
        )
        widths = [int(available_width * fraction) for fraction in column_fractions]
        widths[-1] += available_width - sum(widths)

        rebuilt = document.add_table(
            rows=len(source_cells),
            cols=column_count,
        )
        rebuilt.style = "Table Grid"
        rebuilt.alignment = WD_TABLE_ALIGNMENT.CENTER
        rebuilt.autofit = False
        for row_index, values in enumerate(source_cells):
            for column_index in range(column_count):
                cell = rebuilt.rows[row_index].cells[column_index]
                cell.width = Pt(widths[column_index] / 20.0)
                cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.TOP
                paragraph = cell.paragraphs[0]
                paragraph.paragraph_format.first_line_indent = Pt(0)
                paragraph.paragraph_format.space_after = Pt(0)
                run = paragraph.add_run(
                    values[column_index] if column_index < len(values) else ""
                )
                set_east_asia_font(run)
                run.font.size = Pt(9)
                run.font.bold = row_index == 0

        table.addprevious(rebuilt._tbl)
        table.getparent().remove(table)


def set_repeat_table_header(row):
    row_properties = row._tr.get_or_add_trPr()
    repeat = row_properties.find(qn("w:tblHeader"))
    if repeat is None:
        repeat = OxmlElement("w:tblHeader")
        row_properties.append(repeat)
    repeat.set(qn("w:val"), "true")


def configure_report_body(document):
    normal = document.styles["Normal"]
    set_east_asia_font(normal)
    normal.font.size = Pt(10.5)
    normal.paragraph_format.line_spacing = 1.5
    normal.paragraph_format.first_line_indent = Cm(0.74)
    normal.paragraph_format.space_after = Pt(0)

    for name, size in (
        ("Title", 18),
        ("Heading 1", 14),
        ("Heading 2", 12),
        ("Heading 3", 10.5),
    ):
        if name not in document.styles:
            continue
        style = document.styles[name]
        set_east_asia_font(style)
        style.font.size = Pt(size)
        style.font.bold = True
        style.paragraph_format.first_line_indent = Pt(0)
        style.paragraph_format.keep_with_next = True
        style.paragraph_format.space_before = Pt(8)
        style.paragraph_format.space_after = Pt(4)

    for style_name in ("Caption", "Image Caption", "Table Caption"):
        if style_name in document.styles:
            style = document.styles[style_name]
            set_east_asia_font(style)
            style.font.size = Pt(9)
            style.font.bold = False

    for paragraph in document.paragraphs:
        text = paragraph.text.strip()
        if text.startswith(("三、", "五、", "六、")):
            paragraph.paragraph_format.page_break_before = True
        if text.startswith("图 ") or text.startswith("表 "):
            paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
            paragraph.paragraph_format.first_line_indent = Pt(0)
        if text.startswith("附录 ") or text == "参考文献":
            paragraph.paragraph_format.keep_with_next = True
        for run in paragraph.runs:
            set_east_asia_font(run)

    for table in document.tables:
        table.alignment = WD_TABLE_ALIGNMENT.CENTER
        if table.rows:
            set_repeat_table_header(table.rows[0])
        for row_index, row in enumerate(table.rows):
            for cell in row.cells:
                cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.TOP
                for paragraph in cell.paragraphs:
                    paragraph.paragraph_format.first_line_indent = Pt(0)
                    paragraph.paragraph_format.space_after = Pt(0)
                    for run in paragraph.runs:
                        set_east_asia_font(run)
                        run.font.size = Pt(9)
                        if row_index == 0:
                            run.font.bold = True

    properties = document.core_properties
    properties.title = "实验二：TurtleBot4 移动机器人自主导航与避障"
    properties.subject = "计算机仿真报告与实地实验填写模板"
    properties.author = ""
    properties.keywords = "ROS 2, TurtleBot4, Nav2, A*, AMCL, SLAM"


def build(input_path, output_path):
    template_document = Document(TEMPLATE)
    trim_template_to_front_matter(template_document)

    report_document = Document(input_path)
    trim_to_report_body(report_document)
    repair_gridless_tables(report_document)
    configure_report_body(report_document)

    composer = Composer(template_document)
    composer.append(report_document)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    composer.save(output_path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    arguments = parser.parse_args()
    build(arguments.input, arguments.output)


if __name__ == "__main__":
    main()
