"""
ik_solver.py — 7-DOF 空间机械臂逆运动学求解器（MATLAB 代码迁移版）
"""

import numpy as np
from kinematics_dh import DH_TABLE, fk_mat, jac_num, pose_err, euler_XYpZ_to_T


def ik_space_robot(x: float, y: float, z: float,
                   alpha: float, beta: float, gamma: float,
                   dh: np.ndarray | None = None,
                   num_trials: int = 100,
                   max_iters: int = 200,
                   lam: float = 0.5,
                   tol: float = 1e-6,
                   conv_tol: float = 1e-4,
                   uniq_thresh: float = 0.15,
                   max_solutions: int = 10,
                   seed: int | None = None) -> list[dict]:
    if dh is None:
        dh = DH_TABLE

    rng = np.random.default_rng(seed)
    T_des = euler_XYpZ_to_T(x, y, z, alpha, beta, gamma)
    candidates = []
    # 先尝试零位和多个随机初值
    initial_guesses = [np.zeros(7)] + [rng.standard_normal(7) for _ in range(num_trials)]

    for theta_init in initial_guesses:
        theta = _dls_solve(theta_init.copy(), T_des, dh, max_iters, lam, tol)
        err = np.max(np.abs(fk_mat(theta, dh) - T_des))
        if err < conv_tol:
            candidates.append(theta.copy())

    if not candidates:
        return [{"theta": np.array([]), "error": np.inf, "is_valid": False}]

    unique_solutions = []
    for cand in candidates:
        is_dup = any(np.mean(np.abs(_wrap_to_pi(cand - uq))) < uniq_thresh for uq in unique_solutions)
        if not is_dup:
            unique_solutions.append(cand)

    errors = [np.max(np.abs(fk_mat(sol, dh) - T_des)) for sol in unique_solutions]
    idx = np.argsort(errors)
    n = min(len(unique_solutions), max_solutions)

    return [
        {"theta": unique_solutions[idx[i]], "error": errors[idx[i]],
         "is_valid": errors[idx[i]] < 0.001}
        for i in range(n)
    ]


def _dls_solve(theta: np.ndarray, T_des: np.ndarray,
               dh: np.ndarray, max_iters: int,
               lam: float, tol: float) -> np.ndarray:
    """DLS 迭代求解，带自适应阻尼"""
    for _ in range(max_iters):
        Tc = fk_mat(theta, dh)
        e = pose_err(Tc, T_des)
        en = np.linalg.norm(e)
        if en < tol:
            break
        J = jac_num(theta, dh)
        # 自适应阻尼: 大误差用大阻尼防发散，小误差用小阻尼加速收敛
        lam_adapt = max(lam * min(en, 1.0), 1e-4)
        JTJ = J @ J.T + lam_adapt ** 2 * np.eye(6)
        dq = J.T @ np.linalg.solve(JTJ, e)
        dq = np.clip(dq, -0.5, 0.5)
        theta = theta + dq
        theta = np.arctan2(np.sin(theta), np.cos(theta))
    return theta


def path_ik(path_poses: np.ndarray,
            dh: np.ndarray | None = None,
            lam: float = 0.2,
            max_iters: int = 150,
            tol: float = 1e-6,
            fallback_tol: float = 1e-3,
            ik_kwargs: dict | None = None) -> np.ndarray:
    """
    路径逆运动学 — 热启动跟踪 (Sequential IK)

    关键改进:
      - 自适应阻尼加速收敛
      - 热启动收敛慢时自动加随机扰动重新尝试
      - 回退到 ik_space_robot 多随机搜索
    """
    if dh is None:
        dh = DH_TABLE
    if ik_kwargs is None:
        ik_kwargs = {}

    rng = np.random.default_rng()
    N = path_poses.shape[0]
    traj = np.zeros((N, 7))

    for i in range(N):
        pose = path_poses[i]
        T_des = euler_XYpZ_to_T(pose[0], pose[1], pose[2],
                                  pose[3], pose[4], pose[5])

        if i == 0:
            # 第一帧：多随机搜索
            sols = ik_space_robot(pose[0], pose[1], pose[2],
                                  pose[3], pose[4], pose[5],
                                  dh=dh, **ik_kwargs)
            theta = sols[0]["theta"]
            if theta.size == 0:
                raise RuntimeError(f"path_ik: 第 0 帧无法求解")
        else:
            theta_prev = traj[i - 1].copy()

            # 为避免从奇异位形出发，对前一帧解加微小随机扰动
            if i == 1:
                theta_prev += rng.uniform(-0.05, 0.05, 7)

            # 热启动
            theta = _dls_solve(theta_prev, T_des, dh, max_iters, lam, tol)
            err = np.max(np.abs(fk_mat(theta, dh) - T_des))

            if err > fallback_tol:
                # 随机扰动重试
                found = False
                for _ in range(10):
                    q_pert = traj[i - 1] + rng.uniform(-0.3, 0.3, 7)
                    th_test = _dls_solve(q_pert, T_des, dh, max_iters, lam, tol)
                    if np.max(np.abs(fk_mat(th_test, dh) - T_des)) < fallback_tol:
                        theta, found = th_test, True
                        break
                if not found:
                    # 回退到 ik_space_robot
                    sols = ik_space_robot(pose[0], pose[1], pose[2],
                                          pose[3], pose[4], pose[5],
                                          dh=dh, **ik_kwargs)
                    theta = sols[0]["theta"]
                    if theta.size == 0:
                        raise RuntimeError(f"path_ik: 第 {i} 帧回退后仍无法求解")

        traj[i] = theta

    return traj


def _wrap_to_pi(a: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(a), np.cos(a))
