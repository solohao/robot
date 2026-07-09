"""
plot_trajectory.py -- 提取轨迹数据并绘制关节角度、速度、加速度曲线
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS']  # 支持中文
matplotlib.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题


def load_and_plot_trajectory(filepath="walk_traj.npz", smooth_window=5, poly_order=3):
    """
    加载轨迹数据并绘制7个关节的角度、速度、加速度曲线

    Parameters:
    -----------
    filepath : str
        npz文件路径
    smooth_window : int
        平滑窗口大小（必须为奇数）
    poly_order : int
        多项式阶数（必须小于smooth_window）
    """

    # 1. 加载数据
    data = np.load(filepath)

    # 提取轨迹段
    q1 = data["q1"]
    q2 = data["q2"]
    traj_home = data["traj_home"]

    # 合并所有轨迹
    q_all = np.vstack([q1, q2, traj_home])
    n_frames, n_joints = q_all.shape

    print(f"总帧数: {n_frames}")
    print(f"关节数: {n_joints}")
    print(f"各段帧数: q1={len(q1)}, q2={len(q2)}, home={len(traj_home)}")

    # 2. 创建时间轴（假设采样间隔为0.04秒，从walk_main.py中的step_t可以看出）
    dt = 0.04  # 秒
    t = np.arange(n_frames) * dt

    # 3. 计算速度和加速度（使用中心差分法）
    # 先对角度数据进行平滑处理
    q_smooth = np.zeros_like(q_all)
    if smooth_window >= 3 and smooth_window <= n_frames:
        for j in range(n_joints):
            q_smooth[:, j] = savgol_filter(q_all[:, j], smooth_window, poly_order)
    else:
        q_smooth = q_all.copy()

    # 计算速度（一阶导数）
    v = np.zeros_like(q_smooth)
    v[1:-1, :] = (q_smooth[2:, :] - q_smooth[:-2, :]) / (2 * dt)
    v[0, :] = (q_smooth[1, :] - q_smooth[0, :]) / dt
    v[-1, :] = (q_smooth[-1, :] - q_smooth[-2, :]) / dt

    # 计算加速度（二阶导数）
    a = np.zeros_like(v)
    a[1:-1, :] = (v[2:, :] - v[:-2, :]) / (2 * dt)
    a[0, :] = (v[1, :] - v[0, :]) / dt
    a[-1, :] = (v[-1, :] - v[-2, :]) / dt

    # 4. 绘制曲线
    fig, axes = plt.subplots(7, 3, figsize=(15, 18))
    fig.suptitle('7关节运动轨迹分析', fontsize=16)

    joint_names = [f'Joint {i + 1}' for i in range(n_joints)]

    # 标记各段分界
    boundaries = [len(q1), len(q1) + len(q2)]

    for j in range(n_joints):
        # 角度
        ax = axes[j, 0]
        ax.plot(t, q_all[:, j], 'b-', linewidth=1.5, label='原始')
        ax.plot(t, q_smooth[:, j], 'r-', linewidth=2, label='平滑后')
        for b in boundaries:
            ax.axvline(x=t[b], color='gray', linestyle='--', alpha=0.5)
        ax.set_ylabel('角度 (rad)')
        ax.set_title(f'{joint_names[j]} - 角度')
        ax.grid(True, alpha=0.3)
        if j == 0:
            ax.legend()
            ax.set_xlabel('时间 (s)')

        # 速度
        ax = axes[j, 1]
        ax.plot(t, v[:, j], 'g-', linewidth=2)
        for b in boundaries:
            ax.axvline(x=t[b], color='gray', linestyle='--', alpha=0.5)
        ax.set_ylabel('速度 (rad/s)')
        ax.set_title(f'{joint_names[j]} - 速度')
        ax.grid(True, alpha=0.3)
        if j == 0:
            ax.set_xlabel('时间 (s)')

        # 加速度
        ax = axes[j, 2]
        ax.plot(t, a[:, j], 'm-', linewidth=2)
        for b in boundaries:
            ax.axvline(x=t[b], color='gray', linestyle='--', alpha=0.5)
        ax.set_ylabel('加速度 (rad/s²)')
        ax.set_title(f'{joint_names[j]} - 加速度')
        ax.grid(True, alpha=0.3)
        if j == 0:
            ax.set_xlabel('时间 (s)')

    plt.tight_layout()
    plt.savefig('joint_trajectory_analysis.png', dpi=150, bbox_inches='tight')
    plt.show()

    # 5. 打印统计信息
    print("\n=== 统计信息 ===")
    print(f"角度范围:")
    for j in range(n_joints):
        print(f"  Joint {j + 1}: [{np.min(q_all[:, j]):.3f}, {np.max(q_all[:, j]):.3f}] rad")

    print(f"\n速度范围:")
    for j in range(n_joints):
        print(f"  Joint {j + 1}: [{np.min(v[:, j]):.3f}, {np.max(v[:, j]):.3f}] rad/s")

    print(f"\n加速度范围:")
    for j in range(n_joints):
        print(f"  Joint {j + 1}: [{np.min(a[:, j]):.3f}, {np.max(a[:, j]):.3f}] rad/s²")

    return t, q_all, v, a


def plot_individual_joints(filepath="walk_traj.npz", joint_indices=None):
    """
    绘制指定关节的详细曲线（每个关节单独一图，包含角度/速度/加速度）

    Parameters:
    -----------
    filepath : str
        npz文件路径
    joint_indices : list
        要绘制的关节索引列表，例如 [0, 1, 2]
        如果为None，则绘制所有7个关节
    """

    data = np.load(filepath)
    q1 = data["q1"]
    q2 = data["q2"]
    traj_home = data["traj_home"]
    q_all = np.vstack([q1, q2, traj_home])
    n_frames, n_joints = q_all.shape

    if joint_indices is None:
        joint_indices = list(range(n_joints))

    dt = 0.04
    t = np.arange(n_frames) * dt

    # 计算速度和加速度
    v = np.zeros_like(q_all)
    v[1:-1, :] = (q_all[2:, :] - q_all[:-2, :]) / (2 * dt)
    v[0, :] = (q_all[1, :] - q_all[0, :]) / dt
    v[-1, :] = (q_all[-1, :] - q_all[-2, :]) / dt

    a = np.zeros_like(v)
    a[1:-1, :] = (v[2:, :] - v[:-2, :]) / (2 * dt)
    a[0, :] = (v[1, :] - v[0, :]) / dt
    a[-1, :] = (v[-1, :] - v[-2, :]) / dt

    boundaries = [len(q1), len(q1) + len(q2)]

    for j in joint_indices:
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 8))
        fig.suptitle(f'Joint {j + 1} 运动分析', fontsize=14)

        # 角度
        ax1.plot(t, q_all[:, j], 'b-', linewidth=2)
        for b in boundaries:
            ax1.axvline(x=t[b], color='gray', linestyle='--', alpha=0.5, label='段边界')
        ax1.set_ylabel('角度 (rad)')
        ax1.set_title('关节角度')
        ax1.grid(True, alpha=0.3)

        # 速度
        ax2.plot(t, v[:, j], 'g-', linewidth=2)
        for b in boundaries:
            ax2.axvline(x=t[b], color='gray', linestyle='--', alpha=0.5)
        ax2.set_ylabel('速度 (rad/s)')
        ax2.set_title('关节速度')
        ax2.grid(True, alpha=0.3)

        # 加速度
        ax3.plot(t, a[:, j], 'm-', linewidth=2)
        for b in boundaries:
            ax3.axvline(x=t[b], color='gray', linestyle='--', alpha=0.5)
        ax3.set_ylabel('加速度 (rad/s²)')
        ax3.set_xlabel('时间 (s)')
        ax3.set_title('关节加速度')
        ax3.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(f'joint_{j + 1}_analysis.png', dpi=120, bbox_inches='tight')
        plt.show()


def plot_combined_view(filepath="walk_traj.npz"):
    """
    绘制组合视图：所有关节的角度叠加图，便于对比
    """
    data = np.load(filepath)
    q1 = data["q1"]
    q2 = data["q2"]
    traj_home = data["traj_home"]
    q_all = np.vstack([q1, q2, traj_home])
    n_frames, n_joints = q_all.shape

    dt = 0.04
    t = np.arange(n_frames) * dt

    fig, axes = plt.subplots(3, 1, figsize=(14, 10))

    colors = plt.cm.tab10(np.linspace(0, 1, n_joints))
    boundaries = [len(q1), len(q1) + len(q2)]

    # 角度叠加图
    for j in range(n_joints):
        axes[0].plot(t, q_all[:, j], color=colors[j], label=f'Joint {j + 1}', linewidth=1.5)
    for b in boundaries:
        axes[0].axvline(x=t[b], color='gray', linestyle='--', alpha=0.5)
    axes[0].set_ylabel('角度 (rad)')
    axes[0].set_title('所有关节角度')
    axes[0].legend(loc='best', ncol=4)
    axes[0].grid(True, alpha=0.3)

    # 计算速度
    v = np.zeros_like(q_all)
    v[1:-1, :] = (q_all[2:, :] - q_all[:-2, :]) / (2 * dt)
    v[0, :] = (q_all[1, :] - q_all[0, :]) / dt
    v[-1, :] = (q_all[-1, :] - q_all[-2, :]) / dt

    for j in range(n_joints):
        axes[1].plot(t, v[:, j], color=colors[j], label=f'Joint {j + 1}', linewidth=1.5)
    for b in boundaries:
        axes[1].axvline(x=t[b], color='gray', linestyle='--', alpha=0.5)
    axes[1].set_ylabel('速度 (rad/s)')
    axes[1].set_title('所有关节速度')
    axes[1].legend(loc='best', ncol=4)
    axes[1].grid(True, alpha=0.3)

    # 计算加速度
    a = np.zeros_like(v)
    a[1:-1, :] = (v[2:, :] - v[:-2, :]) / (2 * dt)
    a[0, :] = (v[1, :] - v[0, :]) / dt
    a[-1, :] = (v[-1, :] - v[-2, :]) / dt

    for j in range(n_joints):
        axes[2].plot(t, a[:, j], color=colors[j], label=f'Joint {j + 1}', linewidth=1.5)
    for b in boundaries:
        axes[2].axvline(x=t[b], color='gray', linestyle='--', alpha=0.5)
    axes[2].set_ylabel('加速度 (rad/s²)')
    axes[2].set_xlabel('时间 (s)')
    axes[2].set_title('所有关节加速度')
    axes[2].legend(loc='best', ncol=4)
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('all_joints_combined.png', dpi=150, bbox_inches='tight')
    plt.show()


if __name__ == "__main__":
    # 主程序：加载并绘制轨迹

    # 1. 完整分析（7x3子图）
    print("=== 开始完整分析 ===")
    t, q, v, a = load_and_plot_trajectory("walk_traj.npz", smooth_window=5)

    # 2. 绘制特定关节的详细图（例如关节1、3、5）
    print("\n=== 绘制特定关节详细图 ===")
    plot_individual_joints("walk_traj.npz", joint_indices=[0, 2, 4])

    # 3. 组合视图
    print("\n=== 绘制组合视图 ===")
    plot_combined_view("walk_traj.npz")

    print("\n所有图表已保存！")