"""walk_main.py -- v6: 楂樻晥纰版挒閬垮厤 + 绂荤嚎淇濆瓨 + 杞ㄨ抗骞虫粦"""

import time
import numpy as np
from kinematics_chain import fk, L_BASE_WORLD
from scene_poses import (
    build_step1_lbase_waypoints, build_step2_rbase_waypoints,
    build_step3_lbase_waypoints, path_quintic_arclength
)

TOP_ROT = L_BASE_WORLD[:3, :3].copy()
SIDE_ROT = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])

CYL_Z = 0.101
CYL_R = 0.134
CLR_MARGIN = 0.005  # 5 mm safety


def _make_pose(pos, rotation):
    T = np.eye(4); T[:3, 3] = pos; T[:3, :3] = rotation
    return T


def _rotmat_slerp(R1, R2, t):
    R = R1.T @ R2
    cos_a = np.clip((np.trace(R) - 1) / 2, -1, 1)
    if cos_a > 0.9999: return R2 if t > 0.5 else R1
    a = np.arccos(cos_a)
    if a < 0.001: return R2
    k = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    kn = np.linalg.norm(k)
    if kn < 1e-10: return R1 if t < 0.5 else R2
    k = k / kn
    c, s = np.cos(t * a), np.sin(t * a)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return R1 @ (np.eye(3) + s * K + (1 - c) * K @ K)


def _check_clearance(q, T_l_base=None):
    """Min distance (m) from any link to cabin cylinder surface.
    Positive = safe, negative = colliding.
    T_l_base: world pose of L_Base (4x4). If None, uses identity
    (cylinder expressed in world coords = L_Base initial coords)."""
    from ik_solver_chain import _fk_frames
    frames = _fk_frames(q)
    link_pts = [np.zeros(3)]
    for _, Tf in frames:
        link_pts.append(Tf[:3, 3])
    if T_l_base is None:
        T_l_base = np.eye(4)
    min_d = 1e9
    for seg_i in range(len(link_pts) - 1):
        p1_local, p2_local = link_pts[seg_i], link_pts[seg_i + 1]
        for s in np.linspace(0, 1, 25):
            p_local = p1_local + s * (p2_local - p1_local)
            # Transform local point to world
            pw = T_l_base @ np.array([p_local[0], p_local[1], p_local[2], 1.0])
            d = np.hypot(pw[1], pw[2] - CYL_Z) - CYL_R
            if d < min_d:
                min_d = d
    return min_d


def _dls_solve_chain(theta, T_des, max_iters=200, lam=0.5, tol=1e-6,
                     avoid_cyl=True, q_pref=None, k_null=0.03,
                     T_l_base=None):
    """DLS with collision avoidance and preferred-config nullspace bias.
    T_l_base: world pose of L_Base (4x4). Used to transform link
    points to world coordinates for correct cylinder collision checking."""
    from ik_solver_chain import _fk_frames, pose_err
    q = theta.copy()
    J6 = np.eye(6)
    THRESH = CYL_R + 0.06
    if T_l_base is None:
        T_l_base = np.eye(4)

    for _ in range(max_iters):
        Tc = fk(q)
        e = pose_err(Tc, T_des)
        en = np.linalg.norm(e)
        if en < tol: break

        frames = _fk_frames(q)
        p_tip = Tc[:3, 3]

        J = np.zeros((6, 7))
        for i, (_, Tf) in enumerate(frames):
            z = Tf[:3, 2]; p = Tf[:3, 3]
            J[:3, i] = np.cross(z, p_tip - p); J[3:, i] = z

        lam_adp = max(lam * min(en, 1.0), 1e-4)
        dq = J.T @ np.linalg.solve(J @ J.T + lam_adp**2 * J6, e)
        dq = np.clip(dq, -0.5, 0.5)

        if avoid_cyl:
            link_pts = [np.zeros(3)]
            for _, Tf in frames:
                link_pts.append(Tf[:3, 3])
            coll_grad = np.zeros(7)
            n_links = len(link_pts) - 1
            for seg_i in range(n_links):
                p1_local, p2_local = link_pts[seg_i], link_pts[seg_i + 1]
                for s in np.linspace(0, 1, 8):
                    p_local = p1_local + s * (p2_local - p1_local)
                    # Transform to world coordinates
                    pw = T_l_base @ np.array([p_local[0], p_local[1], p_local[2], 1.0])
                    d = np.hypot(pw[1], pw[2] - CYL_Z)
                    if d < THRESH:
                        # Repulsion direction in WORLD coordinates
                        rad_w = np.array([0.0, pw[1], pw[2] - CYL_Z])
                        rn = np.linalg.norm(rad_w)
                        if rn < 1e-10: rad_w = np.array([0., 0., 1.])
                        else: rad_w = rad_w / rn
                        # Project repulsion direction to joint space via Jacobian in local frame
                        depth = THRESH - d
                        w = depth * depth * 5.0 + depth * 2.5
                        for j in range(min(seg_i + 1, 7)):
                            if j >= len(frames): break
                            z_j = frames[j][1][:3, 2]
                            p_j = frames[j][1][:3, 3]
                            # Jacobian gives motion in world frame for local joint velocities
                            Jp_world = np.cross(z_j, p_local - p_j)
                            # Project rad_w onto Jp (world frame dot product)
                            coll_grad[j] += w * np.dot(Jp_world, rad_w[:3])
            J6x7 = J.copy()
            JJT = J6x7 @ J6x7.T + lam_adp**2 * np.eye(6)
            J_pinv = J6x7.T @ np.linalg.solve(JJT, np.eye(6))
            N_null = np.eye(7) - J_pinv @ J6x7
            dq += N_null @ np.clip(coll_grad * 0.08, -0.4, 0.4)

        if q_pref is not None:
            J6x7 = J.copy()
            JJT = J6x7 @ J6x7.T + lam_adp**2 * np.eye(6)
            J_pinv = J6x7.T @ np.linalg.solve(JJT, np.eye(6))
            N_null = np.eye(7) - J_pinv @ J6x7
            q_diff = np.arctan2(np.sin(q_pref - q), np.cos(q_pref - q))
            dq += N_null @ (k_null * q_diff)

        q += dq
        q = np.arctan2(np.sin(q), np.cos(q))
    return q


def _solve_one_frame(q_near, T_des, max_iters=60, lam=0.3, tol=1e-5,
                     q_pref=None, T_l_base=None):
    """Solve one IK frame with collision avoidance + clearance fallback.
    T_l_base: world pose of L_Base (4x4) for correct collision checking."""
    q = _dls_solve_chain(q_near, T_des, max_iters, lam, tol,
                         avoid_cyl=True, q_pref=q_pref, T_l_base=T_l_base)
    clr = _check_clearance(q, T_l_base=T_l_base)

    if clr >= -0.005:
        return q

    from ik_solver_chain import pose_err
    err = np.linalg.norm(pose_err(fk(q), T_des))
    if err > tol * 100:
        return q

    best_q, best_clr = q.copy(), clr
    rng = np.random.default_rng()
    for _ in range(6):
        q0 = q_near + rng.uniform(-0.4, 0.4, 7)
        qt = _dls_solve_chain(q0, T_des, int(max_iters * 1.0),
                              lam * 0.5, tol, avoid_cyl=True, T_l_base=T_l_base)
        clr_t = _check_clearance(qt, T_l_base=T_l_base)
        err_t = np.linalg.norm(pose_err(fk(qt), T_des))
        if err_t < tol * 20 and clr_t > best_clr:
            best_q, best_clr = qt.copy(), clr_t
            if clr_t >= -0.005:
                break
    return best_q


def _solve_home_path(q_start, T_r_side, n_frames=300):
    """Home: fold arm to q=0 with R_Base fixed at T_r_side.
    
    Uses _solve_one_frame (DLS with collision avoidance) for each frame,
    targeting intermediate q values that smoothly transition to q=0.
    """
    traj = np.zeros((n_frames + 1, 7))
    q = q_start.copy()
    traj[0] = q.copy()
    
    for i in range(1, n_frames + 1):
        tau = i / n_frames
        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        q_target = (1 - s) * q_start + s * np.array([0,0,0,0,0,0,2*np.pi])
        T_des = fk(q_target)
        T_l_base = T_r_side @ np.linalg.inv(fk(q))
        q = _solve_one_frame(q, T_des, 50, 0.3, 1e-4, q_pref=q_target, T_l_base=T_l_base)
        traj[i] = q.copy()
    
    traj[-1] = np.zeros(7)
    return _cont_q(traj)


def _cont_q(q):
    for i in range(1, len(q)):
        q[i] = q[i-1] + np.arctan2(np.sin(q[i] - q[i-1]), np.cos(q[i] - q[i-1]))
    return q


def _smooth_traj(traj, window=3):
    if window < 2 or len(traj) < window + 2:
        return traj
    h = window // 2
    sm = traj.copy()
    for i in range(h, len(traj) - h):
        sm[i] = np.mean(traj[i-h:i+h+1], axis=0)
    return sm


def _solve_path(wps_3d, T_r_fixed, moving_rot, n_frames=0, fg=None,
                rseed=42, is_rbase=False, T_l_fixed=None, init_rot=None):
    """Generic path solver. Correctly transforms cabin cylinder to world frame."""
    nf = n_frames if n_frames > 0 else max(len(wps_3d) * 25, 120)
    path = path_quintic_arclength(wps_3d, nf)
    if len(path) > 3 and np.allclose(path[0], path[-1]):
        path = path[:5]
    N = len(path)
    traj_q = np.zeros((N, 7))
    rng = np.random.default_rng(rseed)

    for i in range(N):
        if is_rbase:
            frac = i / max(N - 1, 1)
            rot = _rotmat_slerp(init_rot, SIDE_ROT, frac)
            T_r = _make_pose(path[i], rot)
            T_rel = np.linalg.inv(T_l_fixed) @ T_r
            T_l_base = T_l_fixed  # L_Base world pose (fixed during R_Base move)
        else:
            T_l = _make_pose(path[i], moving_rot)
            T_rel = np.linalg.inv(T_l) @ T_r_fixed
            T_l_base = T_l  # L_Base world pose (changes during L_Base move)

        if i == 0:
            q0 = np.zeros(7) if fg is None else fg
            q = _solve_one_frame(q0, T_rel, 120, 0.3, 1e-5, T_l_base=T_l_base)
            from ik_solver_chain import pose_err
            err = np.max(np.abs(fk(q)[:3, 3] - T_rel[:3, 3]))
            if err > 1e-3:
                best_q, be = q.copy(), err
                for _ in range(10):
                    qr = rng.uniform(-np.pi, np.pi, 7)
                    qt = _solve_one_frame(qr, T_rel, 80, 0.3, 1e-5, T_l_base=T_l_base)
                    e = np.max(np.abs(fk(qt)[:3, 3] - T_rel[:3, 3]))
                    if e < be: best_q, be = qt.copy(), e
                q = best_q
        else:
            qp = traj_q[i - 1].copy()
            q = _solve_one_frame(qp, T_rel, 50, 0.3, 1e-5, q_pref=qp, T_l_base=T_l_base)
            from ik_solver_chain import pose_err
            err = np.max(np.abs(fk(q)[:3, 3] - T_rel[:3, 3]))
            if err > 1e-3:
                for _ in range(4):
                    qp2 = traj_q[i - 1] + rng.uniform(-0.3, 0.3, 7)
                    qt = _solve_one_frame(qp2, T_rel, 50, 0.3, 1e-5, T_l_base=T_l_base)
                    if np.max(np.abs(fk(qt)[:3, 3] - T_rel[:3, 3])) < 1e-3:
                        q = qt.copy(); break
                else:
                    q = _solve_one_frame(traj_q[i - 1], T_rel, 150, 0.5, 1e-6, T_l_base=T_l_base)
        traj_q[i] = q

    traj_q = _cont_q(traj_q)
    traj_q = _smooth_traj(traj_q, window=5)
    return traj_q


def compute_full_trajectory(n_frames=0, h=0.30, tilt=0.15, k1=0, k2=0,
                            verbose=True):
    """Solve all trajectory segments (Step3+Home merged into Fold)."""
    import scene_poses as sp
    sp.H = h; sp.TILT_ANGLE = tilt; sp.K1 = k1; sp.K2 = k2

    T_l0 = L_BASE_WORLD
    T_r_fixed = T_l0 @ fk(np.zeros(7))

    if verbose: print("  [IK] Step 1")
    wps1 = build_step1_lbase_waypoints()
    q1 = _solve_path(wps1, T_r_fixed, TOP_ROT, n_frames, rseed=42)
    T_l_end1 = T_r_fixed @ np.linalg.inv(fk(q1[-1]))

    if verbose: print("  [IK] Step 2")
    wps2 = build_step2_rbase_waypoints()
    q2 = _solve_path(wps2, None, None, n_frames, fg=q1[-1].copy(),
                     rseed=43, is_rbase=True, T_l_fixed=T_l_end1,
                     init_rot=T_r_fixed[:3, :3])

    # Fold: continuous transition from q2[-1] to q=0
    # R_Base stays fixed at T_r_side throughout
    if verbose: print("  [IK] Fold")
    T_r_side = T_l_end1 @ fk(q2[-1])
    n_fold = max(600, n_frames * 2)
    q_start = q2[-1].copy()
    traj_home = np.zeros((n_fold, 7))
    q = q_start.copy()
    traj_home[0] = q.copy()

    for i in range(1, n_fold):
        tau = i / n_fold
        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        q_target = (1 - s) * q_start + s * np.array([0,0,0,0,0,0,2*np.pi])
        T_des = fk(q_target)
        T_l_base = T_r_side @ np.linalg.inv(fk(q))
        q = _solve_one_frame(q, T_des, 50, 0.3, 1e-4, q_pref=q_target, T_l_base=T_l_base)
        traj_home[i] = q.copy()
    traj_home = _cont_q(traj_home)

    if verbose:
        worst = 1e9
        for i, q in enumerate(q1):
            frac = i / max(len(q1)-1, 1)
            p = build_step1_lbase_waypoints()
            import scene_poses
            pp = scene_poses.path_quintic_arclength(p, len(q1))
            T_l = _make_pose(pp[i], TOP_ROT) if i < len(pp) else _make_pose(pp[-1], TOP_ROT)
            c = _check_clearance(q, T_l_base=T_l)
            if c < worst: worst = c
        print(f"    Step1 worst clearance: {worst*1000:.1f} mm")
        worst = 1e9
        for q in q2:
            c = _check_clearance(q, T_l_base=T_l_end1)
            if c < worst: worst = c
        print(f"    Step2 worst clearance: {worst*1000:.1f} mm")
        worst = 1e9
        for i, q in enumerate(traj_home):
            T_l = T_r_side @ np.linalg.inv(fk(q))
            c = _check_clearance(q, T_l_base=T_l)
            if c < worst: worst = c
        print(f"    Fold worst clearance: {worst*1000:.1f} mm")
        print(f"  Total: s1={len(q1)} s2={len(q2)} fold={len(traj_home)}")

    return q1, q2, traj_home, traj_home[-1:]
def run_full_walk(sim, n_frames=0, h=0.30, tilt=0.15, k1=0, k2=0,
                  speed=1.0, save_log=False, log_path="walk_log.npz"):
    if n_frames == 0:
        n_frames = int(round(250 / max(speed, 0.1)))
    q1, q2, traj_home, _ = compute_full_trajectory(n_frames, h, tilt, k1, k2)
    return _execute_paths(sim, q1, q2, traj_home, [], speed, save_log, log_path)


def _execute_paths(sim, q1, q2, q3, traj_home, speed=1.0,
                   save_log=False, log_path="walk_log.npz"):
    all_q = np.vstack([q1, q2, q3])
    l_base = sim.get_object_handle("/L_Base")
    log = {"t": [], "q": []}
    sim_dt = sim.get_time_step()
    step_t = 0.04 / speed
    sleep_t = sim_dt * 0.85 / max(speed, 0.1)

    sim.start_simulation(stepping=True)
    T_l0 = sim.get_pose(l_base)
    T_r_fixed = T_l0 @ fk(np.zeros(7))

    print(f"  [Sim] Step 1 ({len(q1)} frames)")
    for k in range(len(q1)):
        q = q1[k]
        T_l = T_r_fixed @ np.linalg.inv(fk(q))
        sim.set_pose(T_l, l_base)
        sim.set_joint_positions(q)
        log["t"].append(k * step_t); log["q"].append(q.copy())
        sim.step(); time.sleep(sleep_t)

    print(f"  [Sim] Step 2 ({len(q2)} frames)")
    for k in range(len(q2)):
        q = q2[k]
        sim.set_joint_positions(q)
        log["t"].append((len(q1) + k) * step_t); log["q"].append(q.copy())
        sim.step(); time.sleep(sleep_t)

    print(f"  [Sim] Step 3 ({len(q3)} frames)")
    T_l_fixed = sim.get_pose(l_base)
    T_r_side = T_l_fixed @ fk(q2[-1])
    for k in range(len(q3)):
        q = q3[k]
        T_l = T_r_side @ np.linalg.inv(fk(q))
        sim.set_pose(T_l, l_base)
        sim.set_joint_positions(q)
        log["t"].append((len(q1) + len(q2) + k) * step_t); log["q"].append(q.copy())
        sim.step(); time.sleep(sleep_t)

    print(f"  [Sim] Home ({len(traj_home)} frames)")
    for k in range(len(traj_home)):
        q = traj_home[k]
        T_l = T_r_side @ np.linalg.inv(fk(q))
        sim.set_pose(T_l, l_base)
        sim.set_joint_positions(q)
        log["t"].append((len(all_q) + k) * step_t); log["q"].append(q.copy())
        sim.step(); time.sleep(sleep_t)

    hold_sec = 12.0
    print(f"  [Sim] Hold {hold_sec:.0f}s...")
    for _ in range(int(hold_sec / sim_dt)):
        sim.step(); time.sleep(sleep_t)

    T_r_final = sim.get_pose(l_base) @ fk(np.zeros(7))
    drift = np.linalg.norm(T_r_final[:3, 3] - T_r_fixed[:3, 3])
    print(f"  R_Base drift: {drift*1000:.3f} mm")
    sim.stop_simulation()
    print("  [Exec] Done")
    return log


def run_with_ik_verification(n_frames=0, h=0.30, tilt=0.15, k1=0, k2=0):
    q1, q2, traj_home, _ = compute_full_trajectory(n_frames, h, tilt, k1, k2)
    print("  [OK] All IK paths solved")


def save_trajectory(filepath, n_frames=0, h=0.30, tilt=0.15, k1=0, k2=0):
    """Solve and save to .npz."""
    q1, q2, traj_home, _ = compute_full_trajectory(n_frames, h, tilt, k1, k2)
    np.savez(filepath,
             q1=q1, q2=q2, traj_home=traj_home,
             n1=len(q1), n2=len(q2), n_home=len(traj_home))
    print(f"  [Save] Saved to {filepath}")


def load_and_run(sim, filepath, speed=1.0):
    """Load .npz trajectory and run simulation."""
    data = np.load(filepath)
    q1 = data["q1"]; q2 = data["q2"]; traj_home = data["traj_home"]
    return _execute_paths(sim, q1, q2, traj_home, [], speed)


