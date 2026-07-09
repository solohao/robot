"""
coppeliasim_control.py — CoppeliaSim 仿真连接与控制

基于 example.py 和参考 walk.py 的 ZeroMQ Remote API 交互模式。
提供仿真生命周期管理、机器人关节控制、位姿设置等基础操作。
"""

import time
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


# 关节名称 (与 CoppeliaSim 场景中的命名一致)
JOINT_NAMES = [
    "L_Joint1", "L_Joint2", "L_Joint3", "Joint4",
    "R_Joint3", "R_Joint2", "R_Joint1",
]


class CoppeliaSimController:
    """
    CoppeliaSim 仿真控制封装

    负责:
      - 连接仿真器
      - 启停仿真
      - 获取/设置关节位置
      - 设置物体位姿
    """

    def __init__(self, host: str = "localhost", port: int = 23000):
        self.client = RemoteAPIClient(host=host, port=port)
        self.sim = self.client.getObject("sim")
        self._joint_handles: list[int] | None = None
        self._default_fps: int | None = None

    # ============================================================
    # 仿真生命周期
    # ============================================================

    def connect(self) -> None:
        """连接仿真器。"""
        print(f"[Sim] 已连接到 CoppeliaSim (ZMQ)")

    def start_simulation(self, stepping: bool = True) -> None:
        """
        启动仿真

        参数
        ----------
        stepping : bool
            True 以单步模式运行 (需手动 step)
            False 连续运行
        """
        self._default_fps = self.sim.getInt32Param(
            self.sim.intparam_idle_fps)
        self.sim.setInt32Param(self.sim.intparam_idle_fps, 0)

        if stepping:
            self.client.setStepping(True)

        self.sim.startSimulation()
        print("[Sim] 仿真已启动")

    def stop_simulation(self) -> None:
        """停止仿真并恢复 idle fps。"""
        self.sim.stopSimulation()
        if self._default_fps is not None:
            self.sim.setInt32Param(self.sim.intparam_idle_fps,
                                   self._default_fps)
        print("[Sim] 仿真已停止")

    def step(self) -> None:
        """触发下一步仿真 (仅 stepping 模式需要)。"""
        self.client.step()

    def get_simulation_time(self) -> float:
        """获取当前仿真时间 (s)。"""
        return self.sim.getSimulationTime()

    def get_time_step(self) -> float:
        """获取仿真时间步长 (s)。"""
        return self.sim.getSimulationTimeStep()

    # ============================================================
    # 关节控制
    # ============================================================

    @property
    def joint_handles(self) -> list[int]:
        """获取所有关节句柄（已缓存）。"""
        if self._joint_handles is None:
            self._joint_handles = [
                self.sim.getObject(f"/{name}")
                for name in JOINT_NAMES
            ]
        return self._joint_handles

    def set_joint_positions(self, q: np.ndarray) -> None:
        """
        设置所有 7 个关节的位置

        参数
        ----------
        q : ndarray, shape (7,) — 关节角 (rad)
        """
        for handle, qi in zip(self.joint_handles, q):
            self.sim.setJointPosition(handle, float(qi))

    def get_joint_positions(self) -> np.ndarray:
        """读取所有 7 个关节的当前位置 (rad)。"""
        return np.array([
            self.sim.getJointPosition(h) for h in self.joint_handles
        ])

    def get_object_handle(self, path: str) -> int:
        """通过路径获取场景对象句柄。"""
        return self.sim.getObject(path)

    # ============================================================
    # 位姿设置
    # ============================================================

    def set_pose(self, T: np.ndarray, handle: int) -> None:
        """
        设置物体的 4x4 齐次变换矩阵位姿

        参数
        ----------
        T      : ndarray, shape (4, 4) — 齐次变换矩阵
        handle : int                    — 场景对象句柄
        """
        self.sim.setObjectPosition(handle, -1, T[:3, 3].tolist())
        # CoppeliaSim 矩阵格式为 [row0, row1, row2] 平铺
        m = np.zeros(12)
        m[0:12] = np.concatenate([T[0, :4], T[1, :4], T[2, :4]])
        self.sim.setObjectMatrix(handle, -1, m.tolist())

    def get_pose(self, handle: int) -> np.ndarray:
        """
        读取物体的 4x4 齐次变换矩阵位姿

        返回
        -------
        T : ndarray, shape (4, 4)
        """
        pos = np.array(self.sim.getObjectPosition(handle, -1))
        # getObjectMatrix 返回 12 个元素的列表 [row0, row1, row2]
        m = np.array(self.sim.getObjectMatrix(handle, -1))
        T = np.eye(4)
        T[0, :4] = m[0:4]
        T[1, :4] = m[4:8]
        T[2, :4] = m[8:12]
        T[:3, 3] = pos
        return T
