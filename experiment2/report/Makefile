REPORT_BASE := 实验二-TurtleBot4自主导航与避障-完整实验报告
BUILD_DIR := .build
BODY_DOCX := $(BUILD_DIR)/report-body.docx

.PHONY: all assets figures pdf docx clean

all: docx pdf

assets:
	python3 extract_template_assets.py

figures:
	python3 generate_figures.py

reference.docx: make_reference_docx.py
	python3 make_reference_docx.py

docx: assets figures reference.docx
	mkdir -p "$(BUILD_DIR)"
	pandoc report.tex \
		--from=latex \
		--to=docx \
		--resource-path=. \
		--reference-doc=reference.docx \
		-o "$(BODY_DOCX)"
	python3 build_report_docx.py "$(BODY_DOCX)" "$(REPORT_BASE).docx"

pdf: docx
	libreoffice --headless --convert-to pdf --outdir . "$(REPORT_BASE).docx"

clean:
	rm -rf "$(BUILD_DIR)"
	rm -f *.aux *.log *.out *.toc *.lof *.lot *.xdv
