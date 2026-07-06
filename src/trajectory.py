"""Quintic (5th-order) polynomial trajectory planning.

A quintic segment between two joint configurations with zero boundary
velocity and acceleration guarantees continuous position, velocity and
acceleration along the whole multi-segment path.
"""

import numpy as np


def quintic_scalar(s):
    """Normalized quintic time-scaling s(tau) with s(0)=0, s(1)=1 and zero
    boundary velocity/acceleration:  s = 10 tau^3 - 15 tau^4 + 6 tau^5."""
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


def quintic_scalar_d(s):
    return 30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4


def quintic_scalar_dd(s):
    return 60 * s - 180 * s ** 2 + 120 * s ** 3


class QuinticPath:
    """Piecewise-quintic joint-space path through waypoints.

    waypoints: list of (q, duration) - duration is the time to REACH this
    waypoint from the previous one (first waypoint duration ignored).
    """

    def __init__(self, waypoints):
        self.qs = [np.asarray(q, dtype=float) for q, _ in waypoints]
        self.durs = [d for _, d in waypoints][1:]
        self.t0s = np.concatenate([[0.0], np.cumsum(self.durs)])

    @property
    def total_time(self):
        return self.t0s[-1]

    def sample(self, t):
        """Return (q, qd, qdd) at time t."""
        t = np.clip(t, 0.0, self.total_time)
        i = int(np.searchsorted(self.t0s, t, side='right')) - 1
        i = min(i, len(self.durs) - 1)
        T = self.durs[i]
        tau = (t - self.t0s[i]) / T
        dq = self.qs[i + 1] - self.qs[i]
        q = self.qs[i] + dq * quintic_scalar(tau)
        qd = dq * quintic_scalar_d(tau) / T
        qdd = dq * quintic_scalar_dd(tau) / T ** 2
        return q, qd, qdd
