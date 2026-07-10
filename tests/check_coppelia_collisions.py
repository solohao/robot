"""Replay every planned frame against CoppeliaSim collision geometry."""

import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from kinematics import JOINT_NAMES, L_BASE_WORLD, fk
from walk import build_reset_path, build_step1_path, build_step2_path

LINK_NAMES = [
    'L_Link1', 'L_Link2', 'L_Link3',
    'R_Link3', 'R_Link2', 'R_Link1',
]


def matrix12(T):
    return [*T[0, :4], *T[1, :4], *T[2, :4]]


def main():
    fixed_r = L_BASE_WORLD @ fk(np.zeros(7))
    step1 = build_step1_path()
    q1, _, _ = step1.sample(step1.total_time)
    base2 = fixed_r @ np.linalg.inv(fk(q1))
    step2 = build_step2_path(q1, base2)
    q2, _, _ = step2.sample(step2.total_time)
    fixed_side = base2 @ fk(q2)
    reset = build_reset_path(q2, fixed_side)

    client = RemoteAPIClient()
    sim = client.require('sim')
    if sim.getSimulationState() != sim.simulation_stopped:
        raise RuntimeError('stop the simulation before collision replay')

    base = sim.getObject('/L_Base')
    station = sim.getObject('/SpaceStation')
    joints = [sim.getObject('/' + name) for name in JOINT_NAMES]
    links = [(name, sim.getObject('/' + name)) for name in LINK_NAMES]
    collisions = []
    frames = 0

    paths = [
        ('step1', step1, lambda q: fixed_r @ np.linalg.inv(fk(q))),
        ('step2', step2, lambda q: base2),
        ('reset', reset, lambda q: fixed_side @ np.linalg.inv(fk(q))),
    ]
    try:
        for phase, path, base_fn in paths:
            for t in np.arange(0.0, path.total_time + 1e-9, 0.05):
                q, _, _ = path.sample(float(t))
                sim.setObjectMatrix(base, -1, matrix12(base_fn(q)))
                for handle, value in zip(joints, q):
                    sim.setJointPosition(handle, float(value))
                for name, handle in links:
                    if sim.checkCollision(handle, station)[0]:
                        collisions.append((phase, float(t), name))
                frames += 1
    finally:
        sim.setObjectMatrix(base, -1, matrix12(L_BASE_WORLD))
        for handle in joints:
            sim.setJointPosition(handle, 0.0)

    if collisions:
        for collision in collisions[:20]:
            print(collision)
        raise RuntimeError(
            f'{len(collisions)} arm-link/station collisions detected')
    print(f'checked {frames} frames: zero arm-link/station collisions')


if __name__ == '__main__':
    main()
