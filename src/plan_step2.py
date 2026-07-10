"""Offline planner/verifier for motions around the station hull."""

import numpy as np

from kinematics import L_BASE_WORLD, fk, fk_frames, ik_numeric
from trajectory import QuinticPath

CYL_R = 0.134
CYL_Z = 0.101
MIN_LINK_CLEARANCE = 0.018


def segment_clearances(q, base_T, samples=25):
    """Centerline clearance of each chain segment from the station hull."""
    frames = fk_frames(q, base_T)
    pts = ([base_T[:3, 3]] + [T[:3, 3] for _, T in frames]
           + [fk(q, base_T)[:3, 3]])
    clearances = []
    for a, b in zip(pts[:-1], pts[1:]):
        s = np.linspace(0.0, 1.0, samples)
        p = a + s[:, None] * (b - a)
        clearances.append(np.min(np.hypot(p[:, 1], p[:, 2] - CYL_Z)
                                 - CYL_R))
    return np.asarray(clearances)


def clearance(q, base_T):
    """Minimum centerline clearance, including both docking segments."""
    return np.min(segment_clearances(q, base_T))


def link_clearance(q, base_T):
    """Minimum arm-link clearance, excluding both docking-foot segments."""
    return np.min(segment_clearances(q, base_T)[1:-1])


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def solve_near(T_des, base_T, q_near, margin=MIN_LINK_CLEARANCE):
    """Continuous IK solution near q_near with a checked hull margin."""
    q, err = ik_numeric(T_des, q_near, base=base_T, q_pref=q_near,
                        k_null=0.12, iters=600)
    q = q_near + wrap(q - q_near)
    if err >= 2e-5:
        raise RuntimeError(f'IK did not converge (error {err:.3g})')
    c = link_clearance(q, base_T)
    if c <= margin:
        raise RuntimeError(f'IK link clearance {c:.4f} m is below margin')
    return q


def solve_clear(T_des, base_T, q_near, margin=MIN_LINK_CLEARANCE,
                seeds=150, seed=0):
    """IK solution with clearance > margin, closest (joint-space) to q_near."""
    rng = np.random.default_rng(seed)
    cands = []
    for q0 in [q_near] + list(rng.uniform(-np.pi, np.pi, (seeds, 7))):
        q, err = ik_numeric(T_des, np.asarray(q0, float), base=base_T,
                            q_pref=np.asarray(q_near, float))
        q = q_near + wrap(q - q_near)
        if err < 1e-5 and link_clearance(q, base_T) > margin:
            cands.append(q)
    if not cands:
        raise RuntimeError('no collision-free IK solution found')
    return min(cands, key=lambda c: (
        np.max(np.abs(c - q_near)),
        np.linalg.norm(c - q_near),
        np.abs(c - q_near).sum(),
    ))


def verify_path(path, base_T=None, n=400, base_fn=None):
    if base_T is None and base_fn is None:
        raise ValueError('base_T or base_fn is required')
    worst, t_at = 1e9, 0.0
    for t in np.linspace(0.0, path.total_time, n):
        q, _, _ = path.sample(t)
        base = base_T if base_fn is None else base_fn(q)
        c = link_clearance(q, base)
        if c < worst:
            worst, t_at = c, t
    return worst, t_at


if __name__ == '__main__':
    from walk import build_reset_path, build_step1_path, build_step2_path
    p1 = build_step1_path()
    q1_end, _, _ = p1.sample(p1.total_time)
    T_r = L_BASE_WORLD @ fk(np.zeros(7))
    B2 = T_r @ np.linalg.inv(fk(q1_end))
    p2 = build_step2_path(q1_end, B2)
    w2, t2 = verify_path(p2, B2)
    q2_end, _, _ = p2.sample(p2.total_time)
    T_side = B2 @ fk(q2_end)
    reset = build_reset_path(q2_end, T_side)
    wr, tr = verify_path(
        reset, base_fn=lambda q: T_side @ np.linalg.inv(fk(q)))
    print(f'step2 worst clearance {w2:.4f} m at t={t2:.2f} s')
    print(f'reset worst clearance {wr:.4f} m at t={tr:.2f} s')
