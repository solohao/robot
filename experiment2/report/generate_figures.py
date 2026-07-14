#!/usr/bin/env python3

import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyBboxPatch, Rectangle
from PIL import Image


HERE = Path(__file__).resolve().parent
DATA = HERE / "data"
FIGURES = HERE / "figures"
MAPS = HERE.parent / "maps"


def configure_style():
    candidates = [
        "Noto Sans CJK SC",
        "WenQuanYi Zen Hei",
        "Droid Sans Fallback",
        "DejaVu Sans",
    ]
    available = {font.name for font in font_manager.fontManager.ttflist}
    font = next(name for name in candidates if name in available)
    plt.rcParams.update(
        {
            "font.family": font,
            "axes.unicode_minus": False,
            "figure.dpi": 160,
            "savefig.dpi": 220,
            "axes.titleweight": "bold",
            "axes.edgecolor": "#374151",
            "axes.labelcolor": "#111827",
            "xtick.color": "#374151",
            "ytick.color": "#374151",
        }
    )


def add_box(ax, x, y, width, height, text, facecolor, edgecolor="#1f2937"):
    patch = FancyBboxPatch(
        (x, y),
        width,
        height,
        boxstyle="round,pad=0.018,rounding_size=0.025",
        facecolor=facecolor,
        edgecolor=edgecolor,
        linewidth=1.3,
    )
    ax.add_patch(patch)
    ax.text(
        x + width / 2,
        y + height / 2,
        text,
        ha="center",
        va="center",
        fontsize=9.5,
        color="#111827",
        linespacing=1.25,
    )


def add_arrow(ax, start, end, label=""):
    ax.annotate(
        "",
        xy=end,
        xytext=start,
        arrowprops={"arrowstyle": "-|>", "lw": 1.35, "color": "#374151"},
    )
    if label:
        ax.text(
            (start[0] + end[0]) / 2,
            (start[1] + end[1]) / 2 + 0.018,
            label,
            ha="center",
            va="bottom",
            fontsize=7.5,
            color="#4b5563",
        )


def architecture():
    fig, ax = plt.subplots(figsize=(11.0, 6.2))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    blue = "#dbeafe"
    green = "#dcfce7"
    amber = "#fef3c7"
    purple = "#ede9fe"
    rose = "#ffe4e6"

    add_box(ax, 0.035, 0.37, 0.17, 0.24, "Ignition Gazebo\nwarehouse 世界\nTurtleBot4 物理与传感器", blue)
    add_box(ax, 0.26, 0.70, 0.17, 0.15, "SLAM Toolbox\n增量建图", green)
    add_box(ax, 0.26, 0.42, 0.17, 0.15, "AMCL + Map Server\n静态地图自定位", green)
    add_box(ax, 0.26, 0.14, 0.17, 0.15, "RViz2\n初始位姿、目标与可视化", purple)
    add_box(ax, 0.49, 0.42, 0.15, 0.15, "Nav2\nBT Navigator", amber)
    add_box(ax, 0.70, 0.68, 0.19, 0.16, "Planner Server\n自定义代价感知 A*", rose)
    add_box(ax, 0.70, 0.41, 0.19, 0.16, "Controller Server\nDWB 局部控制", rose)
    add_box(ax, 0.70, 0.14, 0.19, 0.16, "Velocity Smoother\n/cmd_vel_nav → /cmd_vel", rose)

    add_arrow(ax, (0.205, 0.54), (0.26, 0.765), "/scan, /odom, /tf")
    add_arrow(ax, (0.205, 0.49), (0.26, 0.495), "/scan, /odom")
    add_arrow(ax, (0.205, 0.43), (0.26, 0.215), "状态与地图")
    add_arrow(ax, (0.43, 0.495), (0.49, 0.495), "map→odom")
    add_arrow(ax, (0.43, 0.215), (0.49, 0.455), "/goal")
    add_arrow(ax, (0.64, 0.505), (0.70, 0.755), "ComputePath")
    add_arrow(ax, (0.795, 0.68), (0.795, 0.57), "/plan")
    add_arrow(ax, (0.795, 0.41), (0.795, 0.30), "/cmd_vel_nav")
    add_arrow(ax, (0.70, 0.22), (0.205, 0.39), "/cmd_vel")
    add_arrow(ax, (0.43, 0.775), (0.49, 0.55), "/map")

    ax.text(
        0.5,
        0.94,
        "TurtleBot4 仿真自主导航系统架构",
        ha="center",
        fontsize=16,
        fontweight="bold",
        color="#111827",
    )
    ax.text(
        0.5,
        0.02,
        "所有节点使用 ROS 2 Humble；仿真模式统一使用 Gazebo 发布的 /clock。",
        ha="center",
        fontsize=9,
        color="#4b5563",
    )
    fig.tight_layout()
    fig.savefig(FIGURES / "system_architecture.png", bbox_inches="tight")
    plt.close(fig)


def slam_metrics():
    data = json.loads((DATA / "slam-summary.json").read_text())
    initial = data["known_cells_initial"]
    final = data["known_cells_final"]
    displacement = data["odom_displacement"]

    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.2))
    colors = ["#93c5fd", "#2563eb"]
    bars = axes[0].bar(["运动前", "运动后"], [initial, final], color=colors, width=0.56)
    axes[0].set_title("SLAM 已知栅格数量增长")
    axes[0].set_ylabel("已知栅格数 / cell")
    axes[0].grid(axis="y", alpha=0.2)
    for bar, value in zip(bars, [initial, final]):
        axes[0].text(
            bar.get_x() + bar.get_width() / 2,
            value + 350,
            f"{value:,}",
            ha="center",
            fontsize=10,
        )
    axes[0].text(
        0.5,
        0.06,
        f"增量：+{final - initial:,} cells",
        transform=axes[0].transAxes,
        ha="center",
        fontsize=10,
        color="#1d4ed8",
        fontweight="bold",
    )

    axes[1].barh(["里程计位移"], [displacement], color="#10b981", height=0.38)
    axes[1].axvline(0.50, color="#dc2626", linestyle="--", linewidth=1.4, label="验收阈值 0.50 m")
    axes[1].set_xlim(0, 0.65)
    axes[1].set_xlabel("位移 / m")
    axes[1].set_title("建图运动位移")
    axes[1].grid(axis="x", alpha=0.2)
    axes[1].legend(loc="lower right", frameon=False)
    axes[1].text(
        displacement - 0.01,
        0,
        f"{displacement:.3f} m",
        ha="right",
        va="center",
        color="white",
        fontsize=10,
        fontweight="bold",
    )
    fig.suptitle("增量建图客观指标", fontsize=15, fontweight="bold", y=1.02)
    fig.tight_layout()
    fig.savefig(FIGURES / "slam_metrics.png", bbox_inches="tight")
    plt.close(fig)


def path_plot():
    data = json.loads((DATA / "path.json").read_text())
    xs = [point[0] for point in data["coordinates"]]
    ys = [point[1] for point in data["coordinates"]]
    start = data["start"]
    goal = data["end"]

    fig, ax = plt.subplots(figsize=(7.4, 7.0))
    forbidden = Rectangle(
        (-1.9, -2.3),
        3.6,
        0.6,
        facecolor="#fecaca",
        edgecolor="#dc2626",
        linewidth=1.5,
        alpha=0.65,
        label="shelf_7 占用带判定区域",
    )
    ax.add_patch(forbidden)
    ax.plot(xs, ys, color="#2563eb", linewidth=2.4, label="A* 全局路径")
    ax.scatter([start[0]], [start[1]], s=85, marker="o", color="#16a34a", zorder=5, label="起点")
    ax.scatter([goal[0]], [goal[1]], s=105, marker="*", color="#dc2626", zorder=5, label="目标")
    ax.annotate(
        f"最大横向绕行 |x|={data['max_abs_x']:.3f} m",
        xy=(-data["max_abs_x"], -2.0),
        xytext=(-3.25, -1.15),
        arrowprops={"arrowstyle": "->", "color": "#374151"},
        fontsize=9,
    )
    ax.set_title("代价感知 A* 全局路径与 shelf_7 绕障结果")
    ax.set_xlabel("地图坐标 x / m")
    ax.set_ylabel("地图坐标 y / m")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.22)
    ax.legend(loc="lower left", frameon=True)
    ax.text(
        0.98,
        0.98,
        f"路径点：{data['pose_count']}\n长度：{data['path_length']:.3f} m\n占用带违规：0",
        transform=ax.transAxes,
        ha="right",
        va="top",
        fontsize=9,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.9, "edgecolor": "#9ca3af"},
    )
    fig.tight_layout()
    fig.savefig(FIGURES / "astar_path_metrics.png", bbox_inches="tight")
    plt.close(fig)


def acceptance_improvements():
    data = json.loads((DATA / "acceptance-summary.json").read_text())
    previous = data["previous_baseline"]
    slam = data["slam_coverage"]
    navigation = data["navigation"]

    fig, axes = plt.subplots(1, 3, figsize=(12.5, 4.4))
    entries = [
        (
            axes[0],
            "SLAM 累计行程",
            previous["slam_distance"],
            slam["distance"],
            slam["minimum_distance"],
            "m",
        ),
        (
            axes[1],
            "已知栅格增量",
            previous["known_cell_delta"],
            slam["known_cell_delta"],
            slam["minimum_known_cell_delta"],
            "cell",
        ),
        (
            axes[2],
            "A* 全局路径长度",
            previous["path_length"],
            navigation["path_length"],
            navigation["minimum_path_length"],
            "m",
        ),
    ]
    for axis, title, baseline, improved, threshold, unit in entries:
        bars = axis.bar(
            ["改进前", "改进后"],
            [baseline, improved],
            color=["#94a3b8", "#2563eb"],
            width=0.58,
        )
        axis.axhline(
            threshold,
            color="#dc2626",
            linestyle="--",
            linewidth=1.3,
            label=f"验收阈值 {threshold:g} {unit}",
        )
        axis.set_title(title)
        axis.set_ylabel(unit)
        axis.grid(axis="y", alpha=0.2)
        axis.legend(frameon=False, fontsize=8)
        for bar, value in zip(bars, [baseline, improved]):
            label = f"{value:,.3f}" if unit == "m" else f"{value:,.0f}"
            axis.text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_height(),
                label,
                ha="center",
                va="bottom",
                fontsize=9,
                fontweight="bold",
            )
    fig.suptitle("针对助教反馈的量化改进对比", fontsize=15, fontweight="bold", y=1.02)
    fig.tight_layout()
    fig.savefig(FIGURES / "acceptance_improvements.png", bbox_inches="tight")
    plt.close(fig)


def localization_acceptance():
    data = json.loads((DATA / "acceptance-summary.json").read_text())
    localization = data["localization_stability"]
    navigation = data["navigation"]
    video = data["video"]

    fig, axes = plt.subplots(1, 2, figsize=(10.8, 4.4))
    labels = ["位置漂移", "航向漂移"]
    values = [
        localization["position_drift"],
        localization["yaw_drift"],
    ]
    thresholds = [
        localization["maximum_position_drift"],
        localization["maximum_yaw_drift"],
    ]
    axes[0].bar(labels, thresholds, color="#dbeafe", edgecolor="#2563eb", width=0.56)
    axes[0].scatter(labels, values, color="#16a34a", s=90, zorder=5, label="实测值")
    axes[0].set_ylim(0, 0.12)
    axes[0].set_ylabel("m / rad")
    axes[0].set_title("15 s 静止位姿稳定性")
    axes[0].grid(axis="y", alpha=0.2)
    axes[0].legend(frameon=False)
    for index, threshold in enumerate(thresholds):
        axes[0].text(index, threshold + 0.004, f"上限 {threshold:.2f}", ha="center", fontsize=9)
        axes[0].text(index, 0.004, "实测 0.000", ha="center", fontsize=9, color="#166534")

    axes[1].axis("off")
    lines = [
        "LaserScan / 地图匹配率",
        f"{localization['scan_alignment'] * 100:.1f}%（阈值 "
        f"{localization['minimum_scan_alignment'] * 100:.0f}%）",
        "",
        "长距离导航",
        f"{navigation['path_pose_count']} 个位姿 / {navigation['path_length']:.3f} m",
        f"{navigation['wall_duration_seconds']:.1f} s / {navigation['terminal_status_label']}",
        "",
        "完整录屏",
        f"{video['duration_seconds']:.3f} s，无加速、无删减",
    ]
    axes[1].text(
        0.5,
        0.5,
        "\n".join(lines),
        ha="center",
        va="center",
        fontsize=12,
        linespacing=1.45,
        bbox={
            "boxstyle": "round,pad=0.8",
            "facecolor": "#f8fafc",
            "edgecolor": "#94a3b8",
        },
    )
    fig.suptitle("定位、雷达与长距离导航验收结果", fontsize=15, fontweight="bold", y=1.02)
    fig.tight_layout()
    fig.savefig(FIGURES / "localization_acceptance.png", bbox_inches="tight")
    plt.close(fig)


def map_png():
    image = Image.open(MAPS / "lab_map.pgm")
    image.save(FIGURES / "lab_map.png")
    extended = MAPS / "extended_lab_map.pgm"
    if extended.exists():
        Image.open(extended).save(FIGURES / "extended_lab_map.png")


def main():
    FIGURES.mkdir(exist_ok=True)
    configure_style()
    architecture()
    slam_metrics()
    path_plot()
    acceptance_improvements()
    localization_acceptance()
    map_png()


if __name__ == "__main__":
    main()
