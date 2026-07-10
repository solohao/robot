import unittest

import numpy as np

from kinematics import L_BASE_WORLD, fk
from plan_step2 import MIN_LINK_CLEARANCE, verify_path
from walk import (PAD_SIDE, PAD_TOP_3, PLAYBACK_SPEED, build_reset_path,
                  build_step1_path, build_step2_path, foot_pose_side,
                  l_foot_pose_top, sample_times)


class WalkingPlanTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.step1 = build_step1_path()
        cls.q1_end, _, _ = cls.step1.sample(cls.step1.total_time)
        cls.r_fixed = L_BASE_WORLD @ fk(np.zeros(7))
        cls.base2 = cls.r_fixed @ np.linalg.inv(fk(cls.q1_end))
        cls.step2 = build_step2_path(cls.q1_end, cls.base2)
        cls.q2_end, _, _ = cls.step2.sample(cls.step2.total_time)
        cls.r_side = cls.base2 @ fk(cls.q2_end)
        cls.reset = build_reset_path(cls.q2_end, cls.r_side)

    def test_step1_has_clearance_and_uses_lateral_arc(self):
        clearance, _ = verify_path(
            self.step1, n=800,
            base_fn=lambda q: self.r_fixed @ np.linalg.inv(fk(q)))
        samples = np.array([
            self.step1.sample(t)[0]
            for t in np.linspace(0, self.step1.total_time, 800)
        ])
        foot_poses = [
            self.r_fixed @ np.linalg.inv(fk(q)) for q in samples
        ]

        self.assertGreater(clearance, MIN_LINK_CLEARANCE)
        self.assertGreater(max(T[1, 3] for T in foot_poses), 0.3)
        self.assertTrue(np.allclose(
            foot_poses[-1], l_foot_pose_top(PAD_TOP_3), atol=2e-5))

    def test_step2_has_clearance_and_compact_motion(self):
        clearance, _ = verify_path(self.step2, self.base2, n=800)
        samples = np.array([
            self.step2.sample(t)[0]
            for t in np.linspace(0, self.step2.total_time, 800)
        ])
        foot_positions = np.array([
            (self.base2 @ fk(q))[:3, 3] for q in samples
        ])

        self.assertGreater(clearance, MIN_LINK_CLEARANCE)
        self.assertLess(foot_positions[:, 2].max(), 0.39)
        self.assertLess(foot_positions[:, 1].max(), 0.37)
        self.assertTrue(np.allclose(
            self.r_side, foot_pose_side(PAD_SIDE), atol=2e-5))

    def test_reset_is_clear_and_ends_at_initial_joint_pose(self):
        clearance, _ = verify_path(
            self.reset, n=900,
            base_fn=lambda q: self.r_side @ np.linalg.inv(fk(q)))
        q_final, qd_final, qdd_final = self.reset.sample(
            self.reset.total_time)

        self.assertGreater(clearance, MIN_LINK_CLEARANCE)
        self.assertTrue(np.allclose(q_final, np.zeros(7), atol=1e-10))
        self.assertTrue(np.allclose(qd_final, np.zeros(7), atol=1e-10))
        self.assertTrue(np.allclose(qdd_final, np.zeros(7), atol=1e-10))

    def test_paths_join_without_duplicate_or_pause_frame(self):
        dt = 0.05
        step1_times = sample_times(self.step1, dt)
        step2_times = [
            self.step1.total_time + t
            for t in sample_times(self.step2, dt, skip_first=True)
        ]
        reset_times = [
            self.step1.total_time + self.step2.total_time + t
            for t in sample_times(self.reset, dt, skip_first=True)
        ]

        self.assertAlmostEqual(step2_times[0] - step1_times[-1], dt)
        self.assertAlmostEqual(reset_times[0] - step2_times[-1], dt)
        self.assertEqual(PLAYBACK_SPEED, 0.5)


if __name__ == '__main__':
    unittest.main()
