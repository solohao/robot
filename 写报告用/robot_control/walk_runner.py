"""walk_runner.py -- load saved trajectory and run simulation (independent of walk_main)"""

import argparse
import time
import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def main():
    p = argparse.ArgumentParser(description="Load trajectory and control simulation")
    p.add_argument("--traj", type=str, default="walk_traj.npz")
    p.add_argument("--host", type=str, default="localhost")
    p.add_argument("--port", type=int, default=23000)
    p.add_argument("--speed", type=float, default=1.0, help="playback speed")
    p.add_argument("--hold", type=float, default=12.0, help="hold seconds after done")
    args = p.parse_args()

    print("=== Walk Runner ===")
    print(f"  Loading: {args.traj}")
    data = np.load(args.traj)
    q1 = data["q1"]; q2 = data["q2"]; traj_home = data["traj_home"]
    print(f"  Segments: s1={len(q1)} s2={len(q2)} fold={len(traj_home)}")

    from kinematics_chain import fk
    from coppeliasim_control import CoppeliaSimController

    sim = CoppeliaSimController(host=args.host, port=args.port)
    l_base = sim.get_object_handle("/L_Base")
    sim_dt = sim.get_time_step()
    sleep_t = sim_dt * 0.85 / max(args.speed, 0.1)

    sim.start_simulation(stepping=True)
    T_l0 = sim.get_pose(l_base)
    T_r_fixed = T_l0 @ fk(np.zeros(7))
    # Fold phase handles the rest (no separate Step 3)

    print(f"  [Sim] Step 1 ({len(q1)} frames)")
    for k in range(len(q1)):
        q = q1[k]
        T_l = T_r_fixed @ np.linalg.inv(fk(q))
        sim.set_pose(T_l, l_base)
        sim.set_joint_positions(q)
        sim.step()
        time.sleep(sleep_t)

    print(f"  [Sim] Step 2 ({len(q2)} frames)")
    for k in range(len(q2)):
        q = q2[k]
        sim.set_joint_positions(q)
        sim.step()
        time.sleep(sleep_t)

    print(f"  [Sim] Fold ({len(traj_home)} frames)")
    T_l_fixed = sim.get_pose(l_base)
    T_r_side_fold = T_l_fixed @ fk(q2[-1])
    for k in range(len(traj_home)):
        q = traj_home[k]
        T_l = T_r_side_fold @ np.linalg.inv(fk(q))
        sim.set_pose(T_l, l_base)
        sim.set_joint_positions(q)
        sim.step()
        time.sleep(sleep_t)



    hold_sec = args.hold
    print(f"  [Sim] Hold {hold_sec:.0f}s...")
    for _ in range(int(hold_sec / sim_dt)):
        sim.step()
        time.sleep(sleep_t)

    T_r_final = sim.get_pose(l_base) @ fk(np.zeros(7))
    drift = np.linalg.norm(T_r_final[:3, 3] - T_r_fixed[:3, 3])
    print(f"  R_Base drift: {drift*1000:.3f} mm")
    sim.stop_simulation()
    print("  [Exec] Done")


if __name__ == "__main__":
    main()
