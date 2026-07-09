"""
robot_control — 7-DOF 空间机械臂控制程序

基于 MATLAB 正逆运动学代码迁移，实现三步走步行任务。
"""

__version__ = "1.0.0"
__all__ = [
    "kinematics_dh",    # DH 正运动学
    "ik_solver",        # 逆运动学求解
    "trajectory",       # 轨迹规划
    "scene_poses",      # 三步走位姿
    "walk_main",        # 主控逻辑
    "coppeliasim_control",  # CoppeliaSim 连接
]
