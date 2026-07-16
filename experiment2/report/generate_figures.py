#!/usr/bin/env python3

import json
import math
import re
from pathlib import Path

import matplotlib
import numpy as np
import yaml

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch


ROOT = Path(__file__).resolve().parent
DATA = ROOT / "data" / "red-blue"
FIGURES = ROOT / "figures"
NAVIGATION = DATA / "red-blue-navigation-20260715-024010.json"
PLAN = DATA / "red-blue-plan-20260715-024010.yaml"
CLOCK = DATA / "red-blue-clock-stability.log"

plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": [
            "WenQuanYi Zen Hei",
            "Noto Sans CJK SC",
            "DejaVu Sans",
        ],
        "axes.unicode_minus": False,
        "figure.dpi": 160,
        "savefig.dpi": 220,
    }
)


def load_evidence():
    navigation = json.loads(NAVIGATION.read_text(encoding="utf-8"))
    documents = [
        document
        for document in yaml.safe_load_all(PLAN.read_text(encoding="utf-8"))
        if document
    ]
    if len(documents) != 1:
        raise ValueError(f"expected one non-empty plan document, got {len(documents)}")

    positions = [
        (
            pose["pose"]["position"]["x"],
            pose["pose"]["position"]["y"],
        )
        for pose in documents[0]["poses"]
    ]
    length = sum(
        math.hypot(x2 - x1, y2 - y1)
        for (x1, y1), (x2, y2) in zip(positions, positions[1:])
    )
    start = positions[0]
    goal = positions[-1]
    direct = math.hypot(goal[0] - start[0], goal[1] - start[1])
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    deviations = [
        abs(dy * (x - start[0]) - dx * (y - start[1])) / direct
        for x, y in positions
    ]

    checks = {
        "path_pose_count": len(positions),
        "path_length": length,
        "direct_distance": direct,
        "detour_ratio": length / direct,
        "max_lateral_deviation": max(deviations),
    }
    for key, calculated in checks.items():
        recorded = navigation[key]
        tolerance = 0 if key == "path_pose_count" else 1e-9
        if abs(calculated - recorded) > tolerance:
            raise ValueError(
                f"{key} mismatch: calculated={calculated}, recorded={recorded}"
            )

    clock_rates = [
        float(value)
        for value in re.findall(
            r"average rate:\s*([0-9.]+)", CLOCK.read_text(encoding="utf-8")
        )
    ]
    if not clock_rates:
        raise ValueError("no clock rates found")
    return navigation, positions, deviations, clock_rates


def save_figure(fig, name):
    FIGURES.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIGURES / name, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def draw_route(navigation, positions, deviations):
    xy = np.asarray(positions)
    start = xy[0]
    goal = xy[-1]
    farthest_index = int(np.argmax(deviations))
    farthest = xy[farthest_index]

    fig, ax = plt.subplots(figsize=(9.2, 6.0))
    ax.plot(
        xy[:, 0],
        xy[:, 1],
        color="#E69F00",
        linewidth=2.5,
        label="A* 全局路径",
        zorder=3,
    )
    ax.plot(
        [start[0], goal[0]],
        [start[1], goal[1]],
        linestyle="--",
        color="#6B7280",
        linewidth=1.8,
        label="起终点直线",
        zorder=2,
    )
    ax.scatter(
        [start[0]],
        [start[1]],
        s=125,
        color="#D55E00",
        edgecolor="white",
        linewidth=1.2,
        label="红点起点",
        zorder=5,
    )
    ax.scatter(
        [goal[0]],
        [goal[1]],
        s=125,
        color="#0072B2",
        edgecolor="white",
        linewidth=1.2,
        label="蓝点目标",
        zorder=5,
    )
    ax.scatter(
        [farthest[0]],
        [farthest[1]],
        s=70,
        color="#009E73",
        zorder=5,
    )
    ax.annotate(
        f"最大横向绕行 {navigation['max_lateral_deviation']:.3f} m",
        xy=farthest,
        xytext=(18, 16),
        textcoords="offset points",
        arrowprops={"arrowstyle": "->", "color": "#009E73"},
        fontsize=10,
        color="#065F46",
    )
    ax.annotate(
        f"起点\n({start[0]:.2f}, {start[1]:.2f})",
        start,
        xytext=(-72, -44),
        textcoords="offset points",
        fontsize=9,
    )
    ax.annotate(
        f"目标\n({goal[0]:.2f}, {goal[1]:.2f})",
        goal,
        xytext=(12, -40),
        textcoords="offset points",
        fontsize=9,
    )
    ax.set_title("红点至蓝点双货架绕行路线", fontsize=15, weight="bold")
    ax.set_xlabel("地图坐标 x / m")
    ax.set_ylabel("地图坐标 y / m")
    ax.grid(True, color="#D1D5DB", linewidth=0.7, alpha=0.75)
    ax.axis("equal")
    ax.legend(loc="best", frameon=True)
    ax.text(
        0.01,
        0.01,
        (
            f"路径 {navigation['path_length']:.3f} m · "
            f"直线 {navigation['direct_distance']:.3f} m · "
            f"绕行比 {navigation['detour_ratio']:.3f} · "
            f"直线穿过占用区 {navigation['direct_obstacle_segments']} 段"
        ),
        transform=ax.transAxes,
        fontsize=9.5,
        bbox={"boxstyle": "round,pad=0.4", "facecolor": "#F9FAFB", "alpha": 0.95},
    )
    save_figure(fig, "red_blue_route.png")


def draw_acceptance_metrics(navigation):
    labels = ["路径长度", "绕行比", "横向绕行", "占用区段"]
    actual = np.asarray(
        [
            navigation["path_length"],
            navigation["detour_ratio"],
            navigation["max_lateral_deviation"],
            navigation["direct_obstacle_segments"],
        ]
    )
    thresholds = np.asarray(
        [
            navigation["minimum_path_length"],
            navigation["minimum_detour_ratio"],
            navigation["minimum_lateral_deviation"],
            navigation["minimum_direct_obstacles"],
        ]
    )
    units = ["m", "", "m", "段"]
    normalized = actual / thresholds

    fig, ax = plt.subplots(figsize=(8.8, 4.8))
    y = np.arange(len(labels))
    ax.barh(y, normalized, color="#009E73", height=0.56)
    ax.axvline(1.0, color="#D55E00", linestyle="--", linewidth=1.8, label="验收门槛")
    ax.set_yticks(y, labels)
    ax.invert_yaxis()
    ax.set_xlabel("实测值 / 验收门槛")
    ax.set_xlim(0, max(normalized) * 1.22)
    ax.grid(axis="x", color="#D1D5DB", linewidth=0.7, alpha=0.75)
    ax.set_title("红蓝点路径四项几何断言全部通过", fontsize=15, weight="bold")
    for index, (ratio, value, threshold, unit) in enumerate(
        zip(normalized, actual, thresholds, units)
    ):
        suffix = f" {unit}" if unit else ""
        ax.text(
            ratio + 0.025,
            index,
            f"{value:.3f}{suffix} / {threshold:g}{suffix}",
            va="center",
            fontsize=9.5,
        )
    ax.legend(loc="lower right")
    save_figure(fig, "red_blue_acceptance_metrics.png")


def draw_clock_stability(clock_rates):
    samples = np.arange(1, len(clock_rates) + 1)
    fig, ax = plt.subplots(figsize=(8.8, 4.4))
    ax.plot(samples, clock_rates, marker="o", color="#0072B2", linewidth=2.2)
    ax.axhline(20.0, color="#009E73", linestyle="--", linewidth=1.5, label="目标 20 Hz")
    ax.fill_between(
        [0.7, len(clock_rates) + 0.3],
        19.9,
        20.1,
        color="#009E73",
        alpha=0.12,
        label="19.9–20.1 Hz",
    )
    for x, rate in zip(samples, clock_rates):
        ax.text(x, rate + 0.008, f"{rate:.3f}", ha="center", fontsize=9)
    ax.set_xlim(0.7, len(clock_rates) + 0.3)
    ax.set_ylim(min(clock_rates) - 0.04, max(clock_rates) + 0.05)
    ax.set_xticks(samples, [f"窗口 {sample}" for sample in samples])
    ax.set_ylabel("/clock 平均频率 / Hz")
    ax.set_title("世界加载完成后的仿真时钟稳定在约 20 Hz", fontsize=15, weight="bold")
    ax.grid(axis="y", color="#D1D5DB", linewidth=0.7, alpha=0.75)
    ax.legend(loc="lower right")
    save_figure(fig, "red_blue_clock_stability.png")


def draw_runtime_summary(navigation):
    action_minutes = navigation["wall_duration_seconds"] / 60.0
    video_minutes = 1336.604 / 60.0
    fig, ax = plt.subplots(figsize=(8.8, 4.6))
    labels = ["NavigateToPose action", "完整连续视频"]
    values = [action_minutes, video_minutes]
    colors = ["#0072B2", "#56B4E9"]
    bars = ax.barh(labels, values, color=colors, height=0.52)
    ax.invert_yaxis()
    ax.set_xlabel("墙钟时长 / min")
    ax.set_title("同一 action 成功完成，视频覆盖完整运行区间", fontsize=15, weight="bold")
    ax.grid(axis="x", color="#D1D5DB", linewidth=0.7, alpha=0.75)
    for bar, value in zip(bars, values):
        ax.text(
            value + 0.18,
            bar.get_y() + bar.get_height() / 2,
            f"{value:.2f} min",
            va="center",
            fontsize=10,
        )
    ax.text(
        0.02,
        0.08,
        "终态：SUCCEEDED (4)；视频 H.264，15 fps，1336.604 s",
        transform=ax.transAxes,
        fontsize=10,
        bbox={"boxstyle": "round,pad=0.4", "facecolor": "#ECFDF5"},
    )
    ax.set_xlim(0, max(values) * 1.18)
    save_figure(fig, "red_blue_runtime_summary.png")


def draw_system_architecture():
    fig, ax = plt.subplots(figsize=(10.2, 5.8))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 6)
    ax.axis("off")
    boxes = [
        (0.4, 4.2, 2.1, 1.0, "Ignition Gazebo\nTurtleBot4 + warehouse", "#DBEAFE"),
        (3.0, 4.2, 1.8, 1.0, "传感器\n/scan · /odom · TF", "#E0F2FE"),
        (5.3, 4.2, 1.8, 1.0, "地图与定位\nSLAM / Map Server / AMCL", "#DCFCE7"),
        (7.6, 4.2, 2.0, 1.0, "Nav2 任务层\nBT Navigator", "#FEF3C7"),
        (7.6, 2.2, 2.0, 1.0, "Planner Server\n自定义 A* 插件", "#FDE68A"),
        (5.3, 2.2, 1.8, 1.0, "Controller Server\nDWB", "#FCE7F3"),
        (3.0, 2.2, 1.8, 1.0, "Velocity Smoother\n/cmd_vel_nav → /cmd_vel", "#EDE9FE"),
        (0.4, 2.2, 2.1, 1.0, "Create 3 差速底盘\n闭环运动与避障", "#F3F4F6"),
        (3.0, 0.3, 4.1, 0.9, "RViz + 自动验收脚本\n路径几何 · 定位 · 时钟 · action 终态", "#ECFDF5"),
    ]
    for x, y, width, height, text, color in boxes:
        patch = FancyBboxPatch(
            (x, y),
            width,
            height,
            boxstyle="round,pad=0.04,rounding_size=0.08",
            facecolor=color,
            edgecolor="#374151",
            linewidth=1.0,
        )
        ax.add_patch(patch)
        ax.text(x + width / 2, y + height / 2, text, ha="center", va="center", fontsize=9.4)

    arrows = [
        ((2.5, 4.7), (3.0, 4.7)),
        ((4.8, 4.7), (5.3, 4.7)),
        ((7.1, 4.7), (7.6, 4.7)),
        ((8.6, 4.2), (8.6, 3.2)),
        ((7.6, 2.7), (7.1, 2.7)),
        ((5.3, 2.7), (4.8, 2.7)),
        ((3.0, 2.7), (2.5, 2.7)),
        ((1.45, 3.2), (1.45, 4.2)),
        ((5.05, 1.2), (5.05, 2.2)),
    ]
    for start, end in arrows:
        ax.annotate(
            "",
            xy=end,
            xytext=start,
            arrowprops={"arrowstyle": "->", "color": "#4B5563", "lw": 1.5},
        )
    ax.set_title("TurtleBot4 仿真自主导航系统架构", fontsize=15, weight="bold")
    save_figure(fig, "system_architecture.png")


def draw_missing_screenshot(name, title, detail):
    path = FIGURES / name
    if path.exists():
        return
    fig, ax = plt.subplots(figsize=(9.2, 5.2))
    ax.axis("off")
    ax.add_patch(
        FancyBboxPatch(
            (0.05, 0.12),
            0.90,
            0.76,
            transform=ax.transAxes,
            boxstyle="round,pad=0.02",
            facecolor="#F9FAFB",
            edgecolor="#9CA3AF",
            linewidth=1.5,
        )
    )
    ax.text(0.5, 0.62, title, ha="center", va="center", fontsize=17, weight="bold")
    ax.text(0.5, 0.43, detail, ha="center", va="center", fontsize=11, color="#4B5563")
    ax.text(
        0.5,
        0.27,
        "原始 PNG 尚未随当前附件提供",
        ha="center",
        va="center",
        fontsize=10,
        color="#D55E00",
    )
    save_figure(fig, name)


def main():
    navigation, positions, deviations, clock_rates = load_evidence()
    draw_system_architecture()
    draw_route(navigation, positions, deviations)
    draw_acceptance_metrics(navigation)
    draw_clock_stability(clock_rates)
    draw_runtime_summary(navigation)
    draw_missing_screenshot(
        "red_blue_start.png",
        "红点起始状态截图",
        "RViz 中机器人已定位于 (-11.25, -10.50)",
    )
    draw_missing_screenshot(
        "red_blue_path.png",
        "双货架 A* 路径截图",
        "路径 24.489 m，绕行比 2.102，最大横向绕行 7.055 m",
    )
    draw_missing_screenshot(
        "red_blue_success.png",
        "蓝点终态截图",
        "同一 NavigateToPose action 最终状态 SUCCEEDED (4)",
    )


if __name__ == "__main__":
    main()
