#!/usr/bin/env bash

set -euo pipefail

experiment_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repository_dir="$(dirname "${experiment_dir}")"
output="${1:-${repository_dir}/experiment2-submission.zip}"

if [[ "${output}" != /* ]]; then
  output="$(pwd)/${output}"
fi

required_files=(
  "README.md"
  "SUBMISSION.md"
  "tb4-test-plan.md"
  "maps/lab_map.yaml"
  "maps/lab_map.pgm"
  "report/report.tex"
  "report/实验二-TurtleBot4自主导航仿真-实验报告.docx"
  "report/实验二-TurtleBot4自主导航仿真-实验报告.pdf"
  "report/data/astar-plan.yaml"
  "video/实验二-SLAM增量建图.mp4"
  "video/实验二-自主导航.mp4"
)

for path in "${required_files[@]}"; do
  if [[ ! -f "${experiment_dir}/${path}" ]]; then
    echo "Missing required submission file: ${path}" >&2
    exit 1
  fi
done

rm -f "${output}"
(
  cd "${repository_dir}"
  zip -q -r "${output}" \
    experiment2/README.md \
    experiment2/SUBMISSION.md \
    experiment2/tb4-test-plan.md \
    experiment2/maps \
    experiment2/report \
    experiment2/ros2_ws/src \
    experiment2/video \
    -x '*/__pycache__/*' '*.pyc' \
       'experiment2/report/*.aux' \
       'experiment2/report/*.log' \
       'experiment2/report/*.out'
)

echo "Created ${output}"
if rg -q "请填写" "${experiment_dir}/report/report.tex"; then
  echo "Warning: fill the cover placeholders before final course submission." >&2
fi
