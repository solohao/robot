"""scene_poses.py -- v7: 更大y偏移防穿模"""

import numpy as np

H = 0.30
TILT_ANGLE = 0.15
K1 = 0
K2 = 0
N_INTERP = 50

L_START = np.array([0.65, 0.0, 0.235])
R_START = np.array([0.35, 0.0, 0.235])
PAD_TOP_3 = np.array([-0.25, 0.0, 0.235])
PAD_SIDE = np.array([-0.65, 0.133, 0.101])


def quintic_blend(t):
    return 10 * t**3 - 15 * t**4 + 6 * t**5


def path_quintic_arclength(waypoints_3d, n_frames=200):
    pts = [np.array(w) for w in waypoints_3d]
    n_wp = len(pts)
    if n_wp == 1: return np.tile(pts[0], (n_frames, 1))
    if n_frames < 2: n_frames = 2
    seg_lens = [np.linalg.norm(pts[i+1] - pts[i]) for i in range(n_wp - 1)]
    cum_len = np.zeros(n_wp)
    for i in range(1, n_wp):
        cum_len[i] = cum_len[i-1] + seg_lens[i-1]
    total = cum_len[-1]
    if total < 1e-12: return np.tile(pts[0], (n_frames, 1))
    result = np.zeros((n_frames, 3))
    for i in range(n_frames):
        tau = i / max(n_frames - 1, 1)
        s = quintic_blend(tau)
        target_arc = s * total
        seg_idx = np.searchsorted(cum_len, target_arc) - 1
        seg_idx = max(0, min(seg_idx, n_wp - 2))
        local_t = (target_arc - cum_len[seg_idx]) / max(seg_lens[seg_idx], 1e-12)
        local_t = np.clip(local_t, 0, 1)
        result[i] = (1 - local_t) * pts[seg_idx] + local_t * pts[seg_idx + 1]
    return result


def build_step1_lbase_waypoints():
    """
    Step 1: R_Base fixed, L_Base from start to PAD_TOP_3.
    Strategy: lift off with larger +y offset to clear cabin top edge.
    """
    return [
        L_START,
        np.array([0.65, 0.06, 0.50]),       # lift + y offset (doubled)
        np.array([0.62, 0.12, 0.68]),       # continue lift + y
        np.array([0.55, 0.15, 0.82]),       # highest point
        np.array([0.42, 0.15, 0.85]),       # clear R_Base area
        np.array([0.28, 0.15, 0.85]),       # maintain height
        np.array([0.12, 0.12, 0.82]),       # start descent
        np.array([0.00, 0.10, 0.78]),       # descending
        np.array([-0.10, 0.08, 0.68]),      # continuing down
        np.array([-0.18, 0.05, 0.55]),      # approach
        np.array([-0.22, 0.02, 0.40]),      # prepare land
        PAD_TOP_3,
    ]


def build_step2_rbase_waypoints():
    """
    Step 2: L_Base fixed at PAD_TOP_3, R_Base walks to side pad.
    Strategy: very high arc (+y offset) + many intermediate points.
    """
    return [
        R_START,                                 # start (0.35, 0.0, 0.235)
        np.array([0.33, 0.03, 0.50]),             # prelift + y
        np.array([0.30, 0.08, 0.70]),             # lift
        np.array([0.26, 0.15, 0.88]),             # climb
        np.array([0.20, 0.20, 1.00]),             # cruise height
        np.array([0.12, 0.22, 1.08]),             # peak
        np.array([0.00, 0.24, 1.10]),             # peak maintain
        np.array([-0.10, 0.25, 1.10]),            # peak maintain
        np.array([-0.20, 0.25, 1.08]),            # start descent
        np.array([-0.30, 0.24, 1.02]),            # slow descent
        np.array([-0.38, 0.22, 0.92]),            # descent
        np.array([-0.45, 0.20, 0.78]),            # approaching side
        np.array([-0.52, 0.18, 0.62]),            # descend
        np.array([-0.57, 0.17, 0.45]),            # prepare dock
        np.array([-0.60, 0.16, 0.28]),            # pre-dock
        np.array([-0.63, 0.14, 0.15]),            # before dock
        PAD_SIDE,                                 # dock (-0.65, 0.133, 0.101)
    ]


def build_step3_lbase_waypoints():
    """
    Step 3: R_Base fixed on side, L_Base lifts up to raise the arm
    above the cabin for a safe Home transition.
    """
    return [
        PAD_TOP_3,
        np.array([-0.25, 0.03, 0.30]),
        np.array([-0.25, 0.06, 0.40]),
        np.array([-0.25, 0.10, 0.50]),
        np.array([-0.25, 0.12, 0.58]),
    ]
