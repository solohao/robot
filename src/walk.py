"""Space manipulator walking demo (Experiment 1).

The arm walks two steps on the space station:
  step 1: the L foot detaches from the pad at x=0.65 and, pivoting on the
          R foot (x=0.35), swings to the free top pad at x=-0.25 (position 1);
  step 2: the R foot detaches and moves to the side pad at x=-0.65 whose
          normal points along +y (position 2).

All trajectories are piecewise-quintic in joint space, so joint position,
velocity and acceleration are continuous. Waypoint configurations come from
damped-least-squares IK with checked station-hull clearance.

Run CoppeliaSim with SpaceRobot.ttt loaded, then:  python3 walk.py
"""

import os
import time

import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from kinematics import JOINT_NAMES, L_BASE_WORLD, fk, ik_planar_arch
from plan_step2 import MIN_LINK_CLEARANCE, solve_clear, solve_near, verify_path
from trajectory import QuinticPath

OUT_DIR = os.path.join(os.path.dirname(__file__), '..', 'report', 'figures')
PLAYBACK_SPEED = 0.5

PAD_TOP_1 = np.array([0.65, 0.0, 0.235])    # initial L foot
PAD_TOP_2 = np.array([0.35, 0.0, 0.235])    # initial R foot (pivot of step 1)
PAD_TOP_3 = np.array([-0.25, 0.0, 0.235])   # position 1 (top)
PAD_SIDE = np.array([-0.65, 0.1325, 0.101])  # position 2 (side, normal +y)


def l_foot_pose_top(p):
    """World pose of the L foot standing on a top pad."""
    T = L_BASE_WORLD.copy()
    T[:3, 3] = p
    return T


def foot_pose_top(p):
    """World pose of the R foot standing on a top pad."""
    T = np.eye(4)
    T[:3, :3] = np.array([[0., 1., 0.],
                          [0., 0., -1.],
                          [-1., 0., 0.]])
    T[:3, 3] = p
    return T


def foot_pose_side(p):
    """World pose of the R foot docked on the side pad (normal +y)."""
    T = np.eye(4)
    # local -x must point along the pad normal (+y)
    T[:3, :3] = np.array([[0., 0., 1.],
                          [-1., 0., 0.],
                          [0., -1., 0.]])
    T[:3, 3] = p
    return T


def interpolate_rotation(R_start, R_end, u):
    """Spherical interpolation between two rotation matrices."""
    R = R_end @ R_start.T
    angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if angle < 1e-9:
        return R_start.copy()
    axis = np.array([R[2, 1] - R[1, 2],
                     R[0, 2] - R[2, 0],
                     R[1, 0] - R[0, 1]]) / (2 * np.sin(angle))
    x, y, z = axis
    K = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    Ru = (np.eye(3) + np.sin(angle * u) * K
          + (1 - np.cos(angle * u)) * (K @ K))
    return Ru @ R_start


def blended_pose(T_start, T_end, position, u):
    T = np.eye(4)
    T[:3, :3] = interpolate_rotation(T_start[:3, :3], T_end[:3, :3], u)
    T[:3, 3] = position
    return T


def build_step1_path():
    """Move the L foot to the next top pad on a hull-clear lateral arc."""
    fixed_tip = L_BASE_WORLD @ fk(np.zeros(7))
    q0 = np.zeros(7)
    q = np.array([0., 0., np.pi / 2, 0., np.pi / 2, 0., 0.])
    wps = [(q0, 0.0), (q.copy(), 2.5)]
    outward_targets = [
        (0.62, 0.18, 0.400),
        (0.45, 0.28, 0.500),
        (0.10, 0.34, 0.550),
    ]
    for seed, position in enumerate(outward_targets):
        base_des = l_foot_pose_top(np.asarray(position))
        q = solve_clear(
            fixed_tip, base_des, q, margin=MIN_LINK_CLEARANCE + 0.002,
            seeds=10, seed=seed)
        wps.append((q.copy(), 1.5))

    q_goal = ik_planar_arch(PAD_TOP_2[0] - PAD_TOP_3[0], 0.0)
    q_goal[1] += 2 * np.pi
    q_goal[5] -= 2 * np.pi
    q = q_goal.copy()
    return_targets = [
        (-0.18, 0.20, 0.380),
        (-0.05, 0.32, 0.500),
        (0.10, 0.34, 0.550),
    ]
    return_qs = []
    for seed, position in enumerate(return_targets, start=50):
        base_des = l_foot_pose_top(np.asarray(position))
        q = solve_clear(
            fixed_tip, base_des, q, margin=MIN_LINK_CLEARANCE + 0.002,
            seeds=20, seed=seed)
        return_qs.append(q.copy())
    wps.extend([
        (return_qs[2], 2.5),
        (return_qs[1], 1.5),
        (return_qs[0], 1.5),
        (q_goal, 1.5),
    ])
    path = QuinticPath(wps)
    worst, _ = verify_path(
        path, n=600,
        base_fn=lambda q_now: fixed_tip @ np.linalg.inv(fk(q_now)))
    if worst <= MIN_LINK_CLEARANCE:
        raise RuntimeError(f'step 1 path clearance too small: {worst:.4f} m')
    return path


def build_step2_path(q_start, base_T):
    """Move the R foot to the side pad with a compact, checked Cartesian arc."""
    wps = [(q_start, 0.0)]
    q = q_start.copy()
    top = foot_pose_top(np.zeros(3))
    side = foot_pose_side(np.zeros(3))
    targets = [
        ((0.35, 0.00, 0.340), 0.00, 2.0),
        ((0.15, 0.15, 0.380), 0.10, 2.0),
        ((-0.05, 0.28, 0.360), 0.25, 2.0),
        ((-0.30, 0.35, 0.300), 0.45, 2.0),
        ((-0.52, 0.34, 0.230), 0.65, 2.0),
        ((-0.65, 0.30, 0.160), 0.85, 2.0),
        ((-0.65, 0.28, 0.101), 1.00, 1.5),
        ((-0.65, 0.21, 0.101), 1.00, 1.5),
        (tuple(PAD_SIDE), 1.00, 1.5),
    ]
    for position, blend, duration in targets:
        T_des = blended_pose(top, side, np.asarray(position), blend)
        q = solve_near(T_des, base_T, q)
        wps.append((q.copy(), duration))
    path = QuinticPath(wps)
    worst, _ = verify_path(path, base_T, n=600)
    if worst <= MIN_LINK_CLEARANCE:
        raise RuntimeError(f'step 2 path clearance too small: {worst:.4f} m')
    return path


def build_reset_path(q_start, fixed_tip):
    """Keep the R foot fixed and return all joint angles to the initial pose."""
    base_start = fixed_tip @ np.linalg.inv(fk(q_start))
    base_goal = fixed_tip @ np.linalg.inv(fk(np.zeros(7)))
    q = q_start.copy()
    wps = [(q.copy(), 0.0)]
    targets = [
        ((-0.25, 0.00, 0.320), 0.00, 2.0),
        ((-0.32, 0.18, 0.360), 0.12, 2.0),
        ((-0.45, 0.32, 0.280), 0.30, 2.0),
        ((-0.58, 0.48, 0.160), 0.50, 2.5),
        ((-0.65, 0.45, -0.060), 0.70, 2.5),
        ((-0.65, 0.30, -0.180), 0.88, 2.5),
    ]
    for seed, (position, blend, duration) in enumerate(targets):
        base_des = blended_pose(
            base_start, base_goal, np.asarray(position), blend)
        q = solve_clear(
            fixed_tip, base_des, q, margin=MIN_LINK_CLEARANCE + 0.002,
            seeds=20, seed=200 + seed)
        wps.append((q.copy(), duration))
    wps.append((np.zeros(7), 7.0))
    path = QuinticPath(wps)
    worst, _ = verify_path(
        path, n=700,
        base_fn=lambda q_now: fixed_tip @ np.linalg.inv(fk(q_now)))
    if worst <= MIN_LINK_CLEARANCE:
        raise RuntimeError(f'reset path clearance too small: {worst:.4f} m')
    return path


def sample_times(path, dt, skip_first=False):
    n = int(np.ceil(path.total_time / dt))
    start = 1 if skip_first else 0
    return [min(k * dt, path.total_time) for k in range(start, n + 1)]


def main():
    # Plan all motions before starting the stepped simulation.
    T_r_fixed = L_BASE_WORLD @ fk(np.zeros(7))     # R foot world pose (fixed)
    path1 = build_step1_path()
    q1_end, _, _ = path1.sample(path1.total_time)
    T_lbase_new = T_r_fixed @ np.linalg.inv(fk(q1_end))
    path2 = build_step2_path(q1_end, T_lbase_new)
    q2_end, _, _ = path2.sample(path2.total_time)
    T_r_side = T_lbase_new @ fk(q2_end)
    reset_path = build_reset_path(q2_end, T_r_side)

    client = RemoteAPIClient()
    sim = client.getObject('sim')

    joints = [sim.getObject('/' + name) for name in JOINT_NAMES]
    l_base = sim.getObject('/L_Base')

    default_fps = sim.getInt32Param(sim.intparam_idle_fps)
    log = {'t': [], 'q': [], 'qd': [], 'qdd': [], 'l_foot': [], 'r_foot': []}

    def set_pose(T, handle):
        m = np.concatenate([T[0, :4], T[1, :4], T[2, :4]])
        sim.setObjectMatrix(handle, -1, m.tolist())

    def run_path(path, moving='L', T_support=None, base_T=None, t_off=0.0,
                 skip_first=False):
        for t in sample_times(path, dt, skip_first):
            q, qd, qdd = path.sample(t)
            if moving == 'L':
                # R foot fixed: root of the kinematic tree (L_Base) floats
                T_lbase = T_support @ np.linalg.inv(fk(q))
                set_pose(T_lbase, l_base)
                base_now = T_lbase
            else:
                base_now = base_T
            for h, qi in zip(joints, q):
                sim.setJointPosition(h, float(qi))
            log['t'].append(t_off + t)
            log['q'].append(q)
            log['qd'].append(qd)
            log['qdd'].append(qdd)
            log['l_foot'].append(base_now[:3, 3].copy())
            log['r_foot'].append((base_now @ fk(q))[:3, 3].copy())
            client.step()
            time.sleep(dt / PLAYBACK_SPEED)
        return t_off + path.total_time

    sim.setInt32Param(sim.intparam_idle_fps, 0)
    client.setStepping(True)
    started = False
    try:
        sim.startSimulation()
        started = True
        dt = sim.getSimulationTimeStep()
        t_end = run_path(path1, moving='L', T_support=T_r_fixed)
        t_end = run_path(path2, moving='R', base_T=T_lbase_new, t_off=t_end,
                         skip_first=True)
        run_path(reset_path, moving='L', T_support=T_r_side, t_off=t_end,
                 skip_first=True)
        time.sleep(2.0)
    finally:
        if started:
            sim.stopSimulation()
        sim.setInt32Param(sim.intparam_idle_fps, default_fps)

    os.makedirs(OUT_DIR, exist_ok=True)
    np.savez(os.path.join(OUT_DIR, 'walk_log.npz'),
             **{k: np.array(v) for k, v in log.items()})
    print('done, log saved')


if __name__ == '__main__':
    main()
