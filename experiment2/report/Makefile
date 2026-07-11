REPORT_BASE := 实验二-TurtleBot4自主导航仿真-实验报告

.PHONY: all figures pdf docx clean

all: figures pdf docx

figures:
	python3 generate_figures.py

reference.docx: make_reference_docx.py
	python3 make_reference_docx.py

pdf: figures
	xelatex -interaction=nonstopmode -halt-on-error -jobname="$(REPORT_BASE)" report.tex
	xelatex -interaction=nonstopmode -halt-on-error -jobname="$(REPORT_BASE)" report.tex

docx: figures reference.docx
	pandoc report.tex \
		--from=latex \
		--to=docx \
		--resource-path=. \
		--reference-doc=reference.docx \
		-o "$(REPORT_BASE).docx"
	python3 polish_docx.py

clean:
	rm -f *.aux *.log *.out *.toc *.lof *.lot *.xdv
