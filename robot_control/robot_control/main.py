"""main.py -- Entry point (save/run separated)."""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--verify", action="store_true",
                   help="只验证IK求解，不连接仿真")
    p.add_argument("--walk", action="store_true",
                   help="连接仿真并运行行走 (计算+控制同时进行)")
    p.add_argument("--solve", action="store_true",
                   help="离线求解轨迹并保存到文件 (计算与控分离)")
    p.add_argument("--run", type=str, default=None,
                   help="从轨迹文件运行仿真: --run walk_traj.npz")
    p.add_argument("--h", type=float, default=0.30)
    p.add_argument("--tilt", type=float, default=0.15)
    p.add_argument("--k1", type=int, default=0)
    p.add_argument("--k2", type=int, default=0)
    p.add_argument("--n", type=int, default=0, help="每段帧数 (0=自动)")
    p.add_argument("--speed", type=float, default=0.9,
                   help="速度倍率 (默认0.9 ≈ 3x慢速)")
    p.add_argument("--output", type=str, default="walk_traj.npz",
                   help="求解输出路径")
    p.add_argument("--save", action="store_true",
                   help="保存日志")
    p.add_argument("--log", type=str, default="walk_log.npz")
    p.add_argument("--host", type=str, default="localhost")
    p.add_argument("--port", type=int, default=23000)
    args = p.parse_args()

    nf = int(max(args.n, round(250 / max(args.speed, 0.1)))) if args.n == 0 else args.n

    # 离线求解并保存
    if args.solve:
        print(f"  Frames: {nf} per segment  |  speed={args.speed}")
        from walk_main import save_trajectory
        save_trajectory(args.output, nf, args.h, args.tilt, args.k1, args.k2)
        print(f"  Saved to {args.output}")
        print(f"  Run with: python main.py --run {args.output}")
        return

    # 从文件运行仿真
    if args.run:
        from walk_runner import main as runner_main
        sys.argv = sys.argv[:1] + ["--traj", args.run]
        runner_main()
        return

    # 传统模式：验证或连仿真
    print(f"  Frames: {nf} per segment  |  speed={args.speed}")

    if args.verify or not args.walk:
        from walk_main import run_with_ik_verification
        run_with_ik_verification(nf, args.h, args.tilt, args.k1, args.k2)

    if args.walk:
        from coppeliasim_control import CoppeliaSimController
        from walk_main import run_full_walk
        sim = CoppeliaSimController(host=args.host, port=args.port)
        run_full_walk(sim, nf, args.h, args.tilt, args.k1, args.k2,
                      args.speed, args.save, args.log)


if __name__ == "__main__":
    main()
