"""Generate report figures (joint curves, foot positions) from walk_log.npz."""

import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

FIG = os.path.join(os.path.dirname(__file__), '..', 'report', 'figures')


def main():
    d = np.load(os.path.join(FIG, 'walk_log.npz'))
    t, q, qd, qdd = d['t'], d['q'], d['qd'], d['qdd']

    fig, axes = plt.subplots(3, 1, figsize=(9, 10), sharex=True)
    labels = [f'q{i+1}' for i in range(7)]
    for data, ax, name in [(q, axes[0], 'position [rad]'),
                           (qd, axes[1], 'velocity [rad/s]'),
                           (qdd, axes[2], 'acceleration [rad/s$^2$]')]:
        ax.plot(t, data)
        ax.set_ylabel(name)
        ax.grid(True, alpha=0.3)
    axes[0].legend(labels, ncol=7, fontsize=8, loc='upper right')
    axes[2].set_xlabel('time [s]')
    fig.suptitle('Joint trajectories (quintic, C2-continuous)')
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, 'joint_curves.png'), dpi=150)

    fig, ax = plt.subplots(figsize=(9, 4.5))
    for key, style, name in [('l_foot', '-', 'L foot'), ('r_foot', '--', 'R foot')]:
        p = d[key]
        for i, c in enumerate('xyz'):
            ax.plot(t, p[:, i], style, label=f'{name} {c}')
    ax.set_xlabel('time [s]')
    ax.set_ylabel('world position [m]')
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=3, fontsize=8)
    ax.set_title('Foot positions during the 2-step walk')
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, 'foot_positions.png'), dpi=150)
    print('figures written to', FIG)


if __name__ == '__main__':
    main()
