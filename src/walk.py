"""Space manipulator walking demo (Experiment 1).

The arm walks two steps on the space station:
  step 1: the L foot detaches from the pad at x=0.65 and, pivoting on the
          R foot (x=0.35), swings to the free top pad at x=-0.25 (position 1);
  step 2: the R foot detaches and moves to the side pad at x=-0.65 whose
          normal points along +y (position 2).

All trajectories are piecewise-quintic in joint space, so joint position,
velocity and acceleration are continuous.  Waypoint configurations come from
the analytic sagittal-plane IK (step 1) and from damped-least-squares IK
(step 2, general 3D pose).

Run CoppeliaSim with SpaceRobot.ttt loaded, then:  python3 walk.py
"""

import os
import time

import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from kinematics import (JOINT_NAMES, L_BASE_WORLD, fk, ik_numeric,
                        ik_planar_arch)
from trajectory import QuinticPath

OUT_DIR = os.path.join(os.path.dirname(__file__), '..', 'report', 'figures')

PAD_TOP_1 = np.array([0.65, 0.0, 0.235])    # initial L foot
PAD_TOP_2 = np.array([0.35, 0.0, 0.235])    # initial R foot (pivot of step 1)
PAD_TOP_3 = np.array([-0.25, 0.0, 0.235])   # position 1 (top)
PAD_SIDE = np.array([-0.65, 0.1325, 0.101])  # position 2 (side, normal +y)


def foot_pose_top(p):
    """World pose of a foot standing on a top pad at position p (normal +z)."""
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


def solve_planar(x_foot, z_foot, x_support=PAD_TOP_2[0], z_support=0.235):
    """Analytic IK waypoint for step 1 (L foot at (x_foot, z_foot), R fixed)."""
    D = x_support - x_foot
    dz = z_support - z_foot
    return ik_planar_arch(D, dz)


def build_step1_path():
    """L foot: 0.65 -> -0.25 (top pad), pivoting on the R foot."""
    wps = []
    q0 = np.zeros(7)
    wps.append((q0, 0.0))
    # self-motion: reconfigure into the arch family without moving the feet
    s = np.pi / 2
    wps.append((np.array([0., 0., s, 0., s, 0., 0.]), 3.0))
    # swing waypoints of the L foot (x, z, duration)
    prev = wps[-1][0]
    for x, z, d in [(0.62, 0.45, 2.5), (0.35, 0.55, 2.0), (0.05, 0.55, 2.0),
                    (-0.15, 0.40, 2.0), (-0.25, 0.235, 2.5)]:
        q = solve_planar(x, z)
        # keep q2/q6 continuous with the previous waypoint (branch selection)
        for i in (1, 5):
            while q[i] - prev[i] > np.pi:
                q[i] -= 2 * np.pi
            while q[i] - prev[i] < -np.pi:
                q[i] += 2 * np.pi
        wps.append((q, d))
        prev = q
    return QuinticPath(wps)


def build_step2_path(q_start, base_T):
    """R foot: 0.35 (top) -> side pad at x=-0.65, using numeric IK."""
    wps = [(q_start, 0.0)]
    q = q_start.copy()
    targets = []
    # lift and travel above the station, normal still up
    for p, d in [((0.30, 0.0, 0.45), 2.5), ((0.05, 0.0, 0.60), 2.0),
                 ((-0.30, 0.0, 0.60), 2.0)]:
        targets.append((foot_pose_top(np.array(p)), d))
    # rotate the normal from +z to +y while approaching the side pad
    T_mid = np.eye(4)
    ang = np.deg2rad(45)
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(-ang), -np.sin(-ang)],
                   [0, np.sin(-ang), np.cos(-ang)]])
    T_mid[:3, :3] = Rx @ foot_pose_top(np.zeros(3))[:3, :3]
    T_mid[:3, 3] = (-0.55, 0.10, 0.45)
    targets.append((T_mid, 2.0))
    targets.append((foot_pose_side(PAD_SIDE + np.array([0., 0.06, 0.])), 2.0))
    targets.append((foot_pose_side(PAD_SIDE), 2.0))
    for T_des, d in targets:
        q, err = ik_numeric(T_des, q, base=base_T, q_pref=q)
        if err > 1e-4:
            raise RuntimeError(f'IK failed (err={err:.2e}) for target\n{T_des}')
        wps.append((q.copy(), d))
    return QuinticPath(wps)


def main():
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    joints = [sim.getObject('/' + n) if n == 'Joint4' else sim.getObject('/' + n)
              for n in JOINT_NAMES]
    l_base = sim.getObject('/L_Base')

    default_fps = sim.getInt32Param(sim.intparam_idle_fps)
    sim.setInt32Param(sim.intparam_idle_fps, 0)
    client.setStepping(True)
    sim.startSimulation()

    dt = sim.getSimulationTimeStep()
    log = {'t': [], 'q': [], 'qd': [], 'qdd': [], 'l_foot': [], 'r_foot': []}

    def set_pose(T, handle):
        sim.setObjectPosition(handle, -1, T[:3, 3].tolist())
        m = np.zeros(12)
        m[0:12] = np.concatenate([T[0, :4], T[1, :4], T[2, :4]])
        sim.setObjectMatrix(handle, -1, m.tolist())

    def run_path(path, moving='L', T_support=None, base_T=None, t_off=0.0):
        n = int(np.ceil(path.total_time / dt))
        for k in range(n + 1):
            t = min(k * dt, path.total_time)
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
            time.sleep(dt * 0.8)  # keep the visualization close to real time
        return t_off + path.total_time

    # ---- step 1: L foot -> position 1 -------------------------------------
    T_r_fixed = L_BASE_WORLD @ fk(np.zeros(7))     # R foot world pose (fixed)
    path1 = build_step1_path()
    t_end = run_path(path1, moving='L', T_support=T_r_fixed)

    # ---- step 2: R foot -> position 2 (side pad) --------------------------
    q1_end, _, _ = path1.sample(path1.total_time)
    T_lbase_new = T_r_fixed @ np.linalg.inv(fk(q1_end))
    path2 = build_step2_path(q1_end, T_lbase_new)
    run_path(path2, moving='R', base_T=T_lbase_new, t_off=t_end)

    time.sleep(2.0)
    sim.stopSimulation()
    sim.setInt32Param(sim.intparam_idle_fps, default_fps)

    os.makedirs(OUT_DIR, exist_ok=True)
    np.savez(os.path.join(OUT_DIR, 'walk_log.npz'),
             **{k: np.array(v) for k, v in log.items()})
    print('done, log saved')


if __name__ == '__main__':
    main()
