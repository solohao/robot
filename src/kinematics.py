"""Kinematics of the 7-DOF space manipulator (scaled Chinese Space Station arm).

Joint order (from L_Base to R_Base):
    q1 = L_Joint1 (roll about pad normal)
    q2 = L_Joint2 (shoulder pitch)
    q3 = L_Joint3 (pitch)
    q4 = Joint4   (elbow pitch)
    q5 = R_Joint3 (pitch)
    q6 = R_Joint2 (wrist pitch)
    q7 = R_Joint1 (roll about pad normal)

Forward kinematics: T_tip = A0 Rz(q1) A1 Rz(q2) ... A6 Rz(q7) A7 where the
constant matrices Ai come from the scene at zero configuration
(see chain_constants.py).  Every joint rotates about its own local z axis.
"""

import numpy as np

from chain_constants import CHAIN

JOINT_NAMES = ['L_Joint1', 'L_Joint2', 'L_Joint3', 'Joint4',
               'R_Joint3', 'R_Joint2', 'R_Joint1']

# world pose of L_Base in the initial scene (feet on top pads x=0.65 / x=0.35)
L_BASE_WORLD = np.array([
    [0., 0., 1., 0.65],
    [0., 1., 0., 0.00],
    [-1., 0., 0., 0.235],
    [0., 0., 0., 1.]])

# geometry (m), matches the assignment drawing (units there: mm)
H_BASE = 0.085   # pad plate -> shoulder pitch joint (120mm from surface: 0.205->0.32)
A_OFF = 0.05     # lateral offset shoulder->boom (100mm total with elbow offsets)
L_LONG = 0.40    # long boom links (400mm)
AXIAL = 0.30     # fixed axial distance between the two shoulder pitch joints


def rz(q):
    c, s = np.cos(q), np.sin(q)
    T = np.eye(4)
    T[0, 0] = c; T[0, 1] = -s; T[1, 0] = s; T[1, 1] = c
    return T


def fk(q, base=None):
    """Pose of R_Base w.r.t. world (or w.r.t. L_Base if base is None->identity)."""
    T = np.eye(4) if base is None else base.copy()
    qi = 0
    for name, A in CHAIN:
        T = T @ A
        if 'Joint' in name:
            T = T @ rz(q[qi])
            qi += 1
    return T


def fk_frames(q, base=None):
    """World pose of every joint frame (after its rotation)."""
    T = np.eye(4) if base is None else base.copy()
    frames = []
    qi = 0
    for name, A in CHAIN:
        T = T @ A
        if 'Joint' in name:
            T = T @ rz(q[qi])
            qi += 1
            frames.append((name, T.copy()))
    return frames


def jacobian(q, base=None):
    """Geometric Jacobian (6x7) of the tip (R_Base) frame."""
    frames = fk_frames(q, base)
    T_tip = fk(q, base)
    p_tip = T_tip[:3, 3]
    J = np.zeros((6, 7))
    for i, (_, T) in enumerate(frames):
        z = T[:3, 2]
        p = T[:3, 3]
        J[:3, i] = np.cross(z, p_tip - p)
        J[3:, i] = z
    return J


def pose_error(T_cur, T_des):
    """6D error twist (position + orientation, axis*angle)."""
    ep = T_des[:3, 3] - T_cur[:3, 3]
    Re = T_des[:3, :3] @ T_cur[:3, :3].T
    ang = np.arccos(np.clip((np.trace(Re) - 1) / 2, -1, 1))
    if ang < 1e-9:
        eo = np.zeros(3)
    else:
        eo = ang / (2 * np.sin(ang)) * np.array(
            [Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
    return np.concatenate([ep, eo])


def ik_planar_arch(D, dz=0.0):
    """Analytic IK for the sagittal (x-z) walking family.

    The support foot is fixed on a top pad; the moving foot must reach a pose
    that is offset by D along x and dz along z, with its normal kept vertical
    and both feet in the y=0 plane.  Redundancy is resolved with the symmetric
    arch  q3 = t, q4 = pi - 2t, q5 = pi - t  (net pitch of the 3R cluster = 0,
    net y displacement = 0), which gives the virtual shoulder-shoulder link
        v = (-AXIAL, h),  h = 2*L_LONG*cos(t)   (in the arm axial frame).
    Together with the shoulder pitch q2 and wrist pitch q6 = -q2 the
    closed-form solution is:
        r   = sqrt(D^2 + dz^2)            (shoulder-to-shoulder distance)
        h   = sqrt(r^2 - AXIAL^2)         => t = acos(h / (2 L_LONG))
        q2  = atan2 solution aligning v with the target direction
    Returns q (7,) with q1 = q7 = 0.  Raises ValueError if unreachable.
    """
    r2 = D * D + dz * dz
    if r2 < AXIAL ** 2 - 1e-12:
        raise ValueError('target closer than the fixed axial offset')
    h = np.sqrt(max(r2 - AXIAL ** 2, 0.0))
    if h > 2 * L_LONG:
        raise ValueError('target out of reach')
    t = np.arccos(h / (2 * L_LONG))
    # rotate v=(-AXIAL, h) in the x-z plane onto (D, dz):
    # angle of target minus angle of v (pitch about +y maps x->? use atan2 in x-z)
    ang_v = np.arctan2(h, -AXIAL)
    ang_d = np.arctan2(dz, D)
    q2 = ang_d - ang_v          # rotation of v in the x-z plane
    q6 = -q2                    # keep the moving foot normal vertical
    return np.array([0.0, q2, t, np.pi - 2 * t, np.pi - t, q6, 0.0])


def ik_numeric(T_des, q0, base=None, tol=1e-6, iters=300, damp=0.05,
               q_pref=None, k_null=0.05):
    """Damped-least-squares IK with optional nullspace bias towards q_pref."""
    q = np.array(q0, dtype=float)
    for _ in range(iters):
        T = fk(q, base)
        e = pose_error(T, T_des)
        if np.linalg.norm(e) < tol:
            break
        J = jacobian(q, base)
        JT = J.T
        dq = JT @ np.linalg.solve(J @ JT + damp ** 2 * np.eye(6), e)
        if q_pref is not None:
            N = np.eye(7) - np.linalg.pinv(J) @ J
            dq += k_null * (N @ (q_pref - q))
        q += np.clip(dq, -0.2, 0.2)
        q = np.arctan2(np.sin(q), np.cos(q))
    return q, np.linalg.norm(pose_error(fk(q, base), T_des))
