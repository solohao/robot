"""
kinematics_chain.py — Chain-based FK matching SpaceRobot.ttt scene.

Chain constants extracted from SpaceRobot.ttt (参考/chain_constants.py).
This FK is GUARANTEED to match the simulation model.

fk(q) -> pose of R_Base in L_Base frame.
"""

import numpy as np

# Chain: L_Base -> L_Joint1 -> L_Link1 -> ... -> R_Base
# Each entry: (name, fixed_transform_4x4)
CHAIN = [
    ('L_Joint1', np.array([[ 0.   ,  0.   , -1.   , -0.035],
                           [-1.   ,  0.   ,  0.   ,  0.   ],
                           [ 0.   ,  1.   ,  0.   ,  0.   ],
                           [ 0.   ,  0.   ,  0.   ,  1.   ]])),
    ('L_Link1', np.array([[ 1.  ,  0.  ,  0.  ,  0.01],
                          [ 0.  ,  0.  , -1.  ,  0.  ],
                          [ 0.  ,  1.  ,  0.  ,  0.04],
                          [ 0.  ,  0.  ,  0.  ,  1.  ]])),
    ('L_Joint2', np.array([[ 0.  ,  0.  ,  1.  ,  0.04],
                           [-1.  ,  0.  ,  0.  ,  0.01],
                           [ 0.  , -1.  ,  0.  ,  0.  ],
                           [ 0.  ,  0.  ,  0.  ,  1.  ]])),
    ('L_Link2', np.array([[ 1.   ,  0.   ,  0.   ,  0.   ],
                          [ 0.   ,  0.   , -1.   , -0.01 ],
                          [ 0.   ,  1.   ,  0.   ,  0.037],
                          [ 0.   ,  0.   ,  0.   ,  1.   ]])),
    ('L_Joint3', np.array([[ 0.   , -1.   ,  0.   ,  0.   ],
                           [ 1.   ,  0.   ,  0.   ,  0.013],
                           [ 0.   ,  0.   ,  1.   ,  0.04 ],
                           [ 0.   ,  0.   ,  0.   ,  1.   ]])),
    ('L_Link3', np.array([[ 0.  ,  0.  ,  1.  ,  0.  ],
                          [-1.  ,  0.  ,  0.  ,  0.2 ],
                          [ 0.  , -1.  ,  0.  ,  0.05],
                          [ 0.  ,  0.  ,  0.  ,  1.  ]])),
    ('Joint4', np.array([[ 0.  , -1.  ,  0.  , -0.2 ],
                         [ 0.  ,  0.  , -1.  , -0.05],
                         [ 1.  ,  0.  ,  0.  ,  0.  ],
                         [ 0.  ,  0.  ,  0.  ,  1.  ]])),
    ('R_Link3', np.array([[ 0.  ,  0.  , -1.  ,  0.  ],
                          [-1.  ,  0.  ,  0.  , -0.2 ],
                          [ 0.  ,  1.  ,  0.  ,  0.05],
                          [ 0.  ,  0.  ,  0.  ,  1.  ]])),
    ('R_Joint3', np.array([[ 0.  , -1.  ,  0.  ,  0.2 ],
                           [ 0.  ,  0.  , -1.  ,  0.05],
                           [ 1.  ,  0.  ,  0.  ,  0.  ],
                           [ 0.  ,  0.  ,  0.  ,  1.  ]])),
    ('R_Link2', np.array([[ 0.   , -1.   ,  0.   ,  0.013],
                          [ 1.   ,  0.   ,  0.   ,  0.   ],
                          [ 0.   ,  0.   ,  1.   , -0.04 ],
                          [ 0.   ,  0.   ,  0.   ,  1.   ]])),
    ('R_Joint2', np.array([[ 1.   ,  0.   ,  0.   ,  0.   ],
                           [ 0.   ,  0.   ,  1.   , -0.037],
                           [ 0.   , -1.   ,  0.   , -0.01 ],
                           [ 0.   ,  0.   ,  0.   ,  1.   ]])),
    ('R_Link1', np.array([[ 0.  ,  1.  ,  0.  , -0.01],
                          [ 0.  ,  0.  ,  1.  ,  0.  ],
                          [ 1.  ,  0.  ,  0.  , -0.04],
                          [ 0.  ,  0.  ,  0.  ,  1.  ]])),
    ('R_Joint1', np.array([[ 1.  ,  0.  ,  0.  , -0.01],
                           [ 0.  ,  0.  ,  1.  , -0.04],
                           [ 0.  , -1.  ,  0.  ,  0.  ],
                           [ 0.  ,  0.  ,  0.  ,  1.  ]])),
    ('R_Base', np.array([[ 0.   ,  0.   ,  1.   ,  0.   ],
                         [ 0.   ,  1.   ,  0.   ,  0.   ],
                         [-1.   ,  0.   ,  0.   , -0.035],
                         [ 0.   ,  0.   ,  0.   ,  1.   ]])),
]

JOINT_NAMES = ['L_Joint1', 'L_Joint2', 'L_Joint3', 'Joint4',
               'R_Joint3', 'R_Joint2', 'R_Joint1']

L_BASE_WORLD = np.array([
    [0., 0., 1., 0.65],
    [0., 1., 0., 0.00],
    [-1., 0., 0., 0.235],
    [0., 0., 0., 1.]])


def rz(q):
    """Rotation about z axis as 4x4 homogeneous transform."""
    c, s = np.cos(q), np.sin(q)
    T = np.eye(4)
    T[0, 0] = c; T[0, 1] = -s
    T[1, 0] = s; T[1, 1] = c
    return T


def fk(q, base=None):
    """
    Forward kinematics: pose of R_Base w.r.t. world (or w.r.t. L_Base).

    Parameters
    ----------
    q    : ndarray (7,) — joint angles
    base : ndarray (4,4), optional — L_Base world pose

    Returns
    -------
    T : ndarray (4,4) — R_Base pose
    """
    T = np.eye(4) if base is None else base.copy()
    qi = 0
    for name, A in CHAIN:
        T = T @ A
        if 'Joint' in name:
            T = T @ rz(q[qi])
            qi += 1
    return T
