import os
import unittest

import numpy as np

from common.g1_kinematics import G1ForwardKinematics


class G1ForwardKinematicsTests(unittest.TestCase):
    def test_estimate_returns_full_body_state_and_contact_pairs(self):
        xml_path = os.path.join("g1_description", "g1_29dof_rev_1_0.xml")
        estimator = G1ForwardKinematics(xml_path)

        joint_pos = np.zeros(29, dtype=np.float32)
        base_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

        body_pos_w, body_quat_w, contact_pairs = estimator.estimate(
            joint_pos=joint_pos,
            base_quat=base_quat,
            contact_height_threshold=0.03,
        )

        self.assertEqual(body_pos_w.shape, (31, 3))
        self.assertEqual(body_quat_w.shape, (31, 4))
        self.assertTrue(np.isfinite(body_pos_w).all())
        self.assertTrue(np.isfinite(body_quat_w).all())
        self.assertGreater(body_pos_w[estimator.torso_body_id, 2], body_pos_w[estimator.left_foot_body_id, 2])
        contact_body_ids = {body_id for body_id, _ in contact_pairs}
        self.assertTrue(
            estimator.left_foot_body_id in contact_body_ids or estimator.right_foot_body_id in contact_body_ids
        )


if __name__ == "__main__":
    unittest.main()
