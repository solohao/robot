"""Offline planner/verifier for step 2 (R foot -> side pad).

Chooses collision-free IK branches for every waypoint (the station hull is a
cylinder of radius ~0.134 m around the x axis at z=0.101) and verifies the
whole quintic path against the hull with dense sampling.
"""

import numpy as np

from kinematics import L_BASE_WORLD, fk, fk_frames, ik_numeric
from trajectory import QuinticPath

CYL_R = 0.134
CYL_Z = 0.101


def clearance(q, base_T):
    """Min distance of every link segment to the station hull (<0: inside)."""
    frames = fk_frames(q, base_T)
    pts = [base_T[:3, 3]] + [T[:3, 3] for _, T in frames]
    w = 1e9
    for a, b in zip(pts[:-1], pts[1:]):
        for s in np.linspace(0.0, 1.0, 25):
            p = a + s * (b - a)
            w = min(w, np.hypot(p[1], p[2] - CYL_Z) - CYL_R)
    return w


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def solve_clear(T_des, base_T, q_near, margin=-1e-6, seeds=150, seed=0):
    """IK solution with clearance > margin, closest (joint-space) to q_near."""
    rng = np.random.default_rng(seed)
    cands = []
    for q0 in [q_near] + list(rng.uniform(-np.pi, np.pi, (seeds, 7))):
        q, err = ik_numeric(T_des, np.asarray(q0, float), base=base_T,
                            q_pref=np.asarray(q_near, float))
        if err < 1e-5 and clearance(q, base_T) > margin:
            cands.append(q)
    if not cands:
        raise RuntimeError('no collision-free IK solution found')
    return min(cands, key=lambda c: np.abs(wrap(c - q_near)).sum())


def verify_path(path, base_T, n=400):
    worst, t_at = 1e9, 0.0
    for t in np.linspace(0.0, path.total_time, n):
        q, _, _ = path.sample(t)
        c = clearance(q, base_T)
        if c < worst:
            worst, t_at = c, t
    return worst, t_at


if __name__ == '__main__':
    from walk import build_step1_path, build_step2_path
    p1 = build_step1_path()
    q1_end, _, _ = p1.sample(p1.total_time)
    T_r = L_BASE_WORLD @ fk(np.zeros(7))
    B2 = T_r @ np.linalg.inv(fk(q1_end))
    p2 = build_step2_path(q1_end, B2)
    w2, t2 = verify_path(p2, B2)
    print(f'step2 worst clearance {w2:.4f} m at t={t2:.2f} s')
