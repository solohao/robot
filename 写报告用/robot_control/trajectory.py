"""
trajectory.py — 五次多项式轨迹规划

基于参考文件夹中 trajectory.py 实现。
提供五次多项式时间缩放和分段五次多项式路径。
"""

import numpy as np


def quintic_scalar(s: float) -> float:
    """
    归一化五次时间缩放 s(tau):
    s(0)=0, s(1)=1, 零边界速度/加速度
    s = 10*tau^3 - 15*tau^4 + 6*tau^5
    """
    return 10.0 * s ** 3 - 15.0 * s ** 4 + 6.0 * s ** 5


def quintic_scalar_d(s: float) -> float:
    """一阶导数: ds/dtau"""
    return 30.0 * s ** 2 - 60.0 * s ** 3 + 30.0 * s ** 4


def quintic_scalar_dd(s: float) -> float:
    """二阶导数: d^2s/dtau^2"""
    return 60.0 * s - 180.0 * s ** 2 + 120.0 * s ** 3


class QuinticPath:
    """
    分段五次多项式关节空间路径

    用法
    ----
    waypoints: list of (q, duration)
        q 为 ndarray(7,) 关节角, duration 为到达该路径点的时间段
        (第一个路径点的 duration 被忽略)
    """

    def __init__(self, waypoints: list[tuple[np.ndarray, float]]):
        self.qs = [np.asarray(q, dtype=float) for q, _ in waypoints]
        self.durs = [d for _, d in waypoints][1:]      # 跳过第一个
        self.t0s = np.concatenate([[0.0], np.cumsum(self.durs)])

    @property
    def total_time(self) -> float:
        return float(self.t0s[-1])

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        在时间 t 处采样

        返回
        -------
        q   : ndarray(7,) — 关节位置 (rad)
        qd  : ndarray(7,) — 关节速度 (rad/s)
        qdd : ndarray(7,) — 关节加速度 (rad/s^2)
        """
        t = np.clip(t, 0.0, self.total_time)
        i = int(np.searchsorted(self.t0s, t, side="right")) - 1
        i = min(i, len(self.durs) - 1)
        T = self.durs[i]
        tau = (t - self.t0s[i]) / T
        dq = self.qs[i + 1] - self.qs[i]

        q = self.qs[i] + dq * quintic_scalar(tau)
        qd = dq * quintic_scalar_d(tau) / T
        qdd = dq * quintic_scalar_dd(tau) / T ** 2
        return q, qd, qdd
