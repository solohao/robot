#!/usr/bin/env python3

import json
from pathlib import Path
from zipfile import ZipFile

from lxml import etree


ROOT = Path(__file__).resolve().parent
TEMPLATE = ROOT / "template" / "实验报告模板.docx"
FIGURES = ROOT / "figures"
METADATA = ROOT / "template" / "openxml-metadata.json"

NAMESPACES = {
    "a": "http://schemas.openxmlformats.org/drawingml/2006/main",
    "r": "http://schemas.openxmlformats.org/officeDocument/2006/relationships",
    "rel": "http://schemas.openxmlformats.org/package/2006/relationships",
    "w": "http://schemas.openxmlformats.org/wordprocessingml/2006/main",
}


def integer_attribute(element, name):
    value = element.get(f"{{{NAMESPACES['w']}}}{name}")
    return int(value) if value is not None else None


def main():
    FIGURES.mkdir(parents=True, exist_ok=True)
    with ZipFile(TEMPLATE) as package:
        document = etree.fromstring(package.read("word/document.xml"))
        relationships = etree.fromstring(
            package.read("word/_rels/document.xml.rels")
        )
        styles = etree.fromstring(package.read("word/styles.xml"))

        relationship_by_id = {
            relationship.get("Id"): relationship.get("Target")
            for relationship in relationships
        }
        first_image = document.find(".//a:blip", namespaces=NAMESPACES)
        if first_image is None:
            raise ValueError("template does not contain an embedded image")
        relationship_id = first_image.get(f"{{{NAMESPACES['r']}}}embed")
        image_target = relationship_by_id[relationship_id]
        image_path = f"word/{image_target}"
        (FIGURES / "zju_logo.png").write_bytes(package.read(image_path))

        section = document.find(".//w:sectPr", namespaces=NAMESPACES)
        page_size = section.find("w:pgSz", namespaces=NAMESPACES)
        page_margins = section.find("w:pgMar", namespaces=NAMESPACES)
        default_fonts = styles.find(
            ".//w:docDefaults/w:rPrDefault/w:rPr/w:rFonts",
            namespaces=NAMESPACES,
        )
        texts = [
            text.strip()
            for text in document.xpath(".//w:t/text()", namespaces=NAMESPACES)
            if text.strip()
        ]

        metadata = {
            "source": str(TEMPLATE.relative_to(ROOT)),
            "logo_relationship": relationship_id,
            "logo_package_path": image_path,
            "page_twips": {
                "width": integer_attribute(page_size, "w"),
                "height": integer_attribute(page_size, "h"),
            },
            "margins_twips": {
                key: integer_attribute(page_margins, key)
                for key in ("top", "right", "bottom", "left", "header", "footer")
            },
            "default_fonts": {
                key: default_fonts.get(f"{{{NAMESPACES['w']}}}{key}")
                for key in ("ascii", "eastAsia", "hAnsi", "cs")
                if default_fonts is not None
            },
            "preserved_field_labels": [
                label
                for label in (
                    "组号：",
                    "本科实验报告",
                    "课程名称：",
                    "姓    名：",
                    "院    系：",
                    "专    业：",
                    "学    号：",
                    "指导老师：",
                    "选课时间：",
                    "专业：",
                    "姓名：",
                    "学号：",
                    "日期：",
                    "地点：",
                    "实验报告",
                    "实验名称：",
                    "实验类型：",
                    "同组学生姓名：",
                )
                if any(label in text for text in texts)
            ],
        }
        METADATA.write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )


if __name__ == "__main__":
    main()
