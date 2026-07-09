"""
kinematics_dh.py — 基于 Modified DH 参数的正运动学（MATLAB 代码迁移版）

直接从 matlab/fk_space_robot.m 等文件迁移而来。
Modified DH 规约：T_i = Rx(alpha_{i-1}) * Tx(a_{i-1}) * Rz(theta_i) * Tz(d_i)

关节排列: Roll-Pitch-Pitch-ElbowPitch-Pitch-Pitch-Roll (7-DOF)
"""

import numpy as np


# ============================================================
# 齐次变换基本算子 (MATLAB: Rx.m, Ry.m, Rz.m, Tx.m, Tz.m)
# ============================================================

def Rx(a: float) -> np.ndarray:
    """绕 X 轴旋转 a (rad) 的 4x4 齐次变换矩阵"""
    T = np.eye(4)
    c, s = np.cos(a), np.sin(a)
    T[1, 1] = c; T[1, 2] = -s
    T[2, 1] = s; T[2, 2] = c
    return T


def Ry(b: float) -> np.ndarray:
    """绕 Y 轴旋转 b (rad) 的 4x4 齐次变换矩阵"""
    T = np.eye(4)
    c, s = np.cos(b), np.sin(b)
    T[0, 0] = c; T[0, 2] = s
    T[2, 0] = -s; T[2, 2] = c
    return T


def Rz(t: float) -> np.ndarray:
    """绕 Z 轴旋转 t (rad) 的 4x4 齐次变换矩阵"""
    T = np.eye(4)
    c, s = np.cos(t), np.sin(t)
    T[0, 0] = c; T[0, 1] = -s
    T[1, 0] = s; T[1, 1] = c
    return T


def Tx(d: float) -> np.ndarray:
    """沿 X 轴平移 d (m) 的 4x4 齐次变换矩阵"""
    T = np.eye(4)
    T[0, 3] = d
    return T


def Tz(d: float) -> np.ndarray:
    """沿 Z 轴平移 d (m) 的 4x4 齐次变换矩阵"""
    T = np.eye(4)
    T[2, 3] = d
    return T


# ============================================================
# Modified DH 参数表 (MATLAB: fk_space_robot.m / fk_mat.m)
# ============================================================
# 每行: [alpha (rad), a (m), d (m), theta_offset (rad)]
# 前 7 行为关节段, 第 8 行为末端固定变换
DH_TABLE = np.array([
    [0,          0,      0.120,  0],            # i=1
    [np.pi / 2,  0,      0.100,  -np.pi / 2],   # i=2
    [np.pi / 2,  0,      0.100,  np.pi],         # i=3
    [0,          0.400,  0.100,  np.pi],         # i=4
    [0,          0.400,  0.100,  0],             # i=5
    [np.pi / 2,  0,      0.100,  np.pi / 2],     # i=6
    [np.pi / 2,  0,      0,      0],             # i=7
    [np.pi,      0,      -0.120, np.pi],         # i=8 (固定)
])


# ============================================================
# 正运动学 (MATLAB: fk_mat.m)
# ============================================================

def fk_mat(theta: np.ndarray, dh: np.ndarray | None = None) -> np.ndarray:
    """
    基于 Modified DH 的正运动学

    参数
    ----------
    theta : ndarray, shape (7,)
        7 个关节角 (rad)
    dh    : ndarray, shape (8, 4), optional
        DH 参数表，默认使用 DH_TABLE

    返回
    -------
    T : ndarray, shape (4, 4)
        末端齐次变换矩阵
    """
    if dh is None:
        dh = DH_TABLE
    T = np.eye(4)
    for i in range(8):
        alpha, a, d, off = dh[i]
        th = off + (theta[i] if i <= 6 else 0.0)
        T = T @ Rx(alpha) @ Tx(a) @ Rz(th) @ Tz(d)
    return T


# ============================================================
# 正运动学 + 欧拉角提取 (MATLAB: fk_space_robot.m)
# ============================================================

def fk_space_robot(theta: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    7-DOF 空间机械臂正运动学

    参数
    ----------
    theta : ndarray, shape (7,)
        7 个关节角 (rad), [theta1, ..., theta7]

    返回
    -------
    pos          : ndarray, shape (3,)
        末端位置 [x, y, z] (m)
    euler_angles : ndarray, shape (3,)
        XY''Z'' 欧拉角 [alpha, beta, gamma] (rad)
        内旋顺序: Z(gamma) -> Y(beta) -> X(alpha)
    """
    T = fk_mat(theta)

    pos = T[:3, 3]

    # 提取 XY'Z' 欧拉角 (内旋: Z-Y-X, R = Rz(gamma) @ Ry(beta) @ Rx(alpha))
    R = T[:3, :3]
    beta = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))

    if abs(np.cos(beta)) > 1e-10:
        alpha = np.arctan2(R[2, 1], R[2, 2])
        gamma = np.arctan2(R[1, 0], R[0, 0])
    else:
        alpha = 0.0
        gamma = np.arctan2(-R[0, 1], R[1, 1])

    return pos, np.array([alpha, beta, gamma])


# ============================================================
# 欧拉角 → 齐次变换矩阵 (MATLAB: euler_XYpZ_to_T.m)
# ============================================================

def euler_XYpZ_to_T(x: float, y: float, z: float,
                    alpha: float, beta: float, gamma: float) -> np.ndarray:
    """
    XY''Z'' 欧拉角 → 4x4 齐次变换矩阵

    内旋顺序: Z(gamma) -> Y(beta) -> X(alpha)

    参数
    ----------
    x, y, z     : float  — 位置 (m)
    alpha, beta, gamma : float — 欧拉角 (rad)

    返回
    -------
    T : ndarray, shape (4, 4)
    """
    T = np.eye(4)
    T[:3, :3] = Rz(gamma)[:3, :3] @ Ry(beta)[:3, :3] @ Rx(alpha)[:3, :3]
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


# ============================================================
# 数值雅可比矩阵 (MATLAB: jac_num.m)
# ============================================================

def jac_num(theta: np.ndarray, dh: np.ndarray | None = None,
            delta: float = 1e-6) -> np.ndarray:
    """
    前向差分数值雅可比矩阵 (6x7)

    参数
    ----------
    theta : ndarray, shape (7,) — 关节角 (rad)
    dh    : ndarray, optional   — DH 参数表
    delta : float               — 扰动步长

    返回
    -------
    J : ndarray, shape (6, 7) — 几何雅可比矩阵
    """
    if dh is None:
        dh = DH_TABLE
    T0 = fk_mat(theta, dh)
    p0 = T0[:3, 3]
    R0 = T0[:3, :3]

    J = np.zeros((6, 7))
    for i in range(7):
        th = theta.copy()
        th[i] += delta
        Tp = fk_mat(th, dh)

        J[:3, i] = (Tp[:3, 3] - p0) / delta

        Rr = Tp[:3, :3] @ R0.T
        tr = np.clip((np.trace(Rr) - 1.0) / 2.0, -1.0, 1.0)
        ang = np.arccos(tr)
        if ang > 1e-9:
            w = ang / (2.0 * np.sin(ang))
            J[3:, i] = w * np.array([
                Rr[2, 1] - Rr[1, 2],
                Rr[0, 2] - Rr[2, 0],
                Rr[1, 0] - Rr[0, 1],
            ]) / delta
    return J


# ============================================================
# 位姿误差向量 (MATLAB: pose_err.m)
# ============================================================

def pose_err(T_cur: np.ndarray, T_des: np.ndarray) -> np.ndarray:
    """
    计算 6 维位姿误差向量 [Δp; Δφ] (位置误差 + 轴角姿态误差)

    参数
    ----------
    T_cur : ndarray, shape (4, 4) — 当前位姿
    T_des : ndarray, shape (4, 4) — 期望位姿

    返回
    -------
    e : ndarray, shape (6,) — 误差向量
    """
    ep = T_des[:3, 3] - T_cur[:3, 3]

    Re = T_des[:3, :3] @ T_cur[:3, :3].T
    tr = np.clip((np.trace(Re) - 1.0) / 2.0, -1.0, 1.0)
    ang = np.arccos(tr)

    if ang < 1e-9:
        eo = np.zeros(3)
    else:
        eo = ang / (2.0 * np.sin(ang)) * np.array([
            Re[2, 1] - Re[1, 2],
            Re[0, 2] - Re[2, 0],
            Re[1, 0] - Re[0, 1],
        ])

    return np.concatenate([ep, eo])
