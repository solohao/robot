"""ik_solver_chain.py -- IK using chain-based FK (matches simulation)."""

import numpy as np
from kinematics_chain import fk, CHAIN


def pose_err(Tc, Td):
    ep = Td[:3,3] - Tc[:3,3]
    Re = Td[:3,:3] @ Tc[:3,:3].T
    tr = np.clip((np.trace(Re)-1)/2, -1, 1)
    ang = np.arccos(tr)
    if ang < 1e-9:
        eo = np.zeros(3)
    else:
        eo = ang/(2*np.sin(ang))*np.array([Re[2,1]-Re[1,2], Re[0,2]-Re[2,0], Re[1,0]-Re[0,1]])
    return np.concatenate([ep, eo])


def _rz(t):
    c,s = np.cos(t), np.sin(t)
    T=np.eye(4)
    T[0,0]=c; T[0,1]=-s; T[1,0]=s; T[1,1]=c
    return T


def _euler_to_T(x,y,z,a,b,g):
    Rz = _rz(g)[:3,:3]
    Ry = np.array([[np.cos(b),0,np.sin(b)],[0,1,0],[-np.sin(b),0,np.cos(b)]])
    Rx = np.array([[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]])
    T=np.eye(4); T[:3,:3]=Rz@Ry@Rx; T[:3,3]=[x,y,z]
    return T


def _fk_frames(q):
    """Return list of (name, T) for each joint frame (after rotation)."""
    T = np.eye(4)
    frames = []
    qi = 0
    for name, A in CHAIN:
        T = T @ A
        if "Joint" in name:
            T = T @ _rz(q[qi])
            qi += 1
            frames.append((name, T.copy()))
    return frames


def _dls_solve(theta, T_des, max_iters=200, lam=0.5, tol=1e-6):
    """DLS iterative solver with adaptive damping (chain-based FK)."""
    q = theta.copy()
    for _ in range(max_iters):
        Tc = fk(q)
        e = pose_err(Tc, T_des)
        en = np.linalg.norm(e)
        if en < tol:
            break
        frames = _fk_frames(q)
        p_tip = Tc[:3, 3]
        J = np.zeros((6, 7))
        for i, (_, T) in enumerate(frames):
            z = T[:3, 2]
            p = T[:3, 3]
            J[:3, i] = np.cross(z, p_tip - p)
            J[3:, i] = z
        lam_adapt = max(lam * min(en, 1.0), 1e-4)
        dq = J.T @ np.linalg.solve(J @ J.T + lam_adapt**2 * np.eye(6), e)
        dq = np.clip(dq, -0.5, 0.5)
        q += dq
        q = np.arctan2(np.sin(q), np.cos(q))
    return q


def path_ik_chain(poses):
    """Simple chain-based path IK (hot-start)."""
    N = len(poses)
    traj = np.zeros((N, 7))
    rng = np.random.default_rng()
    for i in range(N):
        p = poses[i]
        Td = _euler_to_T(p[0], p[1], p[2], p[3], p[4], p[5])
        if i == 0:
            q, err = ik_numeric(Td, np.zeros(7))
            if err > 1e-4:
                bq, bqerr = q, err
                for _ in range(30):
                    qr = rng.uniform(-np.pi, np.pi, 7)
                    qq, ee = ik_numeric(Td, qr)
                    if ee < bqerr:
                        bq, bqerr = qq, ee
                q = bq
            traj[i] = q
        else:
            q, err = ik_numeric(Td, traj[i-1], iters=100, damp=0.05)
            if err > 1e-3:
                q, err = ik_numeric(Td, traj[i-1], iters=300, damp=0.05)
            traj[i] = q
    return traj


def path_ik_robust(poses, num_trials=50, max_iters=200, lam=0.5, tol=1e-6,
                   fallback_tol=1e-3, rng_seed=None):
    """
    Robust chain-based path IK with hot-start tracking, perturbation retry,
    and fallback to multi-trial random search.

    This is the chain-model equivalent of ik_solver.path_ik.
    """
    if len(poses) == 0:
        return np.zeros((0, 7))
    rng = np.random.default_rng(rng_seed)
    N = len(poses)
    traj = np.zeros((N, 7))
    for i in range(N):
        p = poses[i]
        T_des = _euler_to_T(p[0], p[1], p[2], p[3], p[4], p[5])
        if i == 0:
            best_q, best_err = None, 1e9
            for t in range(num_trials + 1):
                q0 = np.zeros(7) if t == 0 else rng.uniform(-np.pi, np.pi, 7)
                q = _dls_solve(q0, T_des, max_iters, lam, tol)
                err = np.max(np.abs(fk(q)[:3, 3] - T_des[:3, 3]))
                if err < best_err:
                    best_q, best_err = q.copy(), err
                if err < 1e-5:
                    break
            if best_q is None:
                raise RuntimeError(f"path_ik_robust: frame 0 unsolvable (err={best_err})")
            traj[i] = best_q
        else:
            q_prev = traj[i - 1].copy()
            if i == 1:
                q_prev += rng.uniform(-0.05, 0.05, 7)
            q = _dls_solve(q_prev, T_des, max_iters, lam, tol)
            err = np.max(np.abs(fk(q)[:3, 3] - T_des[:3, 3]))
            if err > fallback_tol:
                found = False
                for _ in range(10):
                    q_pert = traj[i - 1] + rng.uniform(-0.3, 0.3, 7)
                    q_test = _dls_solve(q_pert, T_des, max_iters, lam, tol)
                    if np.max(np.abs(fk(q_test)[:3, 3] - T_des[:3, 3])) < fallback_tol:
                        q, found = q_test, True
                        break
                if not found:
                    best_q, best_err = None, 1e9
                    for t in range(30):
                        q0 = rng.uniform(-np.pi, np.pi, 7)
                        qt = _dls_solve(q0, T_des, max_iters, lam, tol)
                        err2 = np.max(np.abs(fk(qt)[:3, 3] - T_des[:3, 3]))
                        if err2 < best_err:
                            best_q, best_err = qt.copy(), err2
                    q = best_q
            traj[i] = q
    for i in range(1, N):
        traj[i] = traj[i-1] + np.arctan2(np.sin(traj[i] - traj[i-1]),
                                          np.cos(traj[i] - traj[i-1]))
    return traj


def ik_numeric(Td, q0, tol=1e-6, iters=300, damp=0.05):
    """Damped-least-squares IK (chain-based FK)."""
    q = np.array(q0, float)
    for _ in range(iters):
        T = fk(q)
        e = pose_err(T, Td)
        if np.linalg.norm(e) < tol:
            break
        frames = _fk_frames(q)
        p_tip = T[:3, 3]
        J = np.zeros((6, 7))
        for i, (_, Tf) in enumerate(frames):
            z = Tf[:3, 2]
            p = Tf[:3, 3]
            J[:3, i] = np.cross(z, p_tip - p)
            J[3:, i] = z
        dq = J.T @ np.linalg.solve(J @ J.T + damp**2 * np.eye(6), e)
        dq = np.clip(dq, -0.2, 0.2)
        q += dq
        q = np.arctan2(np.sin(q), np.cos(q))
    return q, np.linalg.norm(pose_err(fk(q), Td))
