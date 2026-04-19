import unittest
from unittest import mock

import numpy as np
import os
import yaml

from common.utils import FSMStateName
from common.ctrlcomp import StateAndCmd, PolicyOutput
from policy.anyadapter.AnyAdapter import AnyAdapter
from policy.anyadapter.runtime import (
    append_history,
    body_in_contact,
    build_balance_features,
    build_balance_features_from_state,
    combine_base_and_residual,
    compute_entry_yaw_alignment_quat,
    extract_robot_body_pos_by_ids,
    compute_motion_anchor_orientation_b,
    compute_reference_body_pos_relative_w,
    is_deploy_wrapper_model,
)


class AnyAdapterRuntimeTests(unittest.TestCase):
    def test_anyadapter_config_points_to_deploy_onnx(self):
        cfg_path = os.path.join("policy", "anyadapter", "config", "AnyAdapter.yaml")
        with open(cfg_path, "r") as f:
            cfg = yaml.load(f, Loader=yaml.FullLoader)
        self.assertEqual(os.path.basename(cfg["onnx_path"]), "policy_with_adapter_deploy.onnx")

    def test_fsm_state_exposes_anyadapter(self):
        self.assertEqual(FSMStateName.SKILL_ANYADAPTER.name, "SKILL_ANYADAPTER")

    def test_append_history_keeps_oldest_to_newest_order(self):
        history = np.zeros((1, 3, 4), dtype=np.float32)
        history = append_history(history, np.array([1, 2, 3, 4], dtype=np.float32))
        history = append_history(history, np.array([5, 6, 7, 8], dtype=np.float32))
        history = append_history(history, np.array([9, 10, 11, 12], dtype=np.float32))
        history = append_history(history, np.array([13, 14, 15, 16], dtype=np.float32))

        expected = np.array(
            [[[5, 6, 7, 8], [9, 10, 11, 12], [13, 14, 15, 16]]],
            dtype=np.float32,
        )
        np.testing.assert_allclose(history, expected)

    def test_build_balance_features_preserves_order_and_support_flag(self):
        features = build_balance_features(
            projected_gravity=np.array([0.1, -0.2, -0.97], dtype=np.float32),
            anchor_height_error_z=0.3,
            anchor_roll_pitch_error=np.array([0.4, -0.5], dtype=np.float32),
            left_foot_contact=1.0,
            right_foot_contact=1.0,
            left_foot_height_error_z=0.6,
            right_foot_height_error_z=-0.7,
        )

        expected = np.array([0.1, -0.2, 0.3, 0.4, -0.5, 1.0, 1.0, 1.0, 0.6, -0.7], dtype=np.float32)
        np.testing.assert_allclose(features, expected, atol=1e-6)

    def test_body_in_contact_detects_matching_body_pair(self):
        pairs = [(7, 0), (2, 3), (13, 0)]
        self.assertEqual(body_in_contact(pairs, 7), 1.0)
        self.assertEqual(body_in_contact(pairs, 13), 1.0)
        self.assertEqual(body_in_contact(pairs, 16), 0.0)

    def test_compute_motion_anchor_orientation_b_uses_direct_relative_quat(self):
        robot_anchor_quat_w = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        yaw_90 = np.float32(np.sqrt(0.5))
        ref_anchor_quat_w = np.array([yaw_90, 0.0, 0.0, yaw_90], dtype=np.float32)

        orientation = compute_motion_anchor_orientation_b(robot_anchor_quat_w, ref_anchor_quat_w)
        expected = np.array(
            [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float32,
        )
        np.testing.assert_allclose(orientation, expected, atol=1e-6)

    def test_compute_reference_body_pos_relative_w_aligns_xy_to_robot_anchor(self):
        ref_body_pos_w = np.array([[1.0, 0.0, 0.4], [2.0, 0.0, 0.6]], dtype=np.float32)
        transformed = compute_reference_body_pos_relative_w(
            robot_anchor_pos_w=np.array([10.0, 20.0, 1.5], dtype=np.float32),
            robot_anchor_quat_w=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
            ref_body_pos_w=ref_body_pos_w,
            ref_anchor_pos_w=ref_body_pos_w[0],
            ref_anchor_quat_w=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        )
        expected = np.array([[10.0, 20.0, 0.4], [11.0, 20.0, 0.6]], dtype=np.float32)
        np.testing.assert_allclose(transformed, expected, atol=1e-6)

    def test_build_balance_features_from_state_uses_robot_and_reference_bodies(self):
        robot_body_pos_w = np.zeros((20, 3), dtype=np.float32)
        robot_body_pos_w[16, 2] = 1.1
        robot_body_pos_w[7, 2] = 0.2
        robot_body_pos_w[13, 2] = 0.3

        robot_anchor_quat_w = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        ref_body_pos_w = np.zeros((30, 3), dtype=np.float32)
        ref_body_pos_w[15, 2] = 1.4
        ref_body_pos_w[6, 2] = 0.5
        ref_body_pos_w[12, 2] = 0.9
        ref_anchor_quat_w = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

        features = build_balance_features_from_state(
            projected_gravity=np.array([0.1, -0.2, -0.97], dtype=np.float32),
            robot_anchor_pos_w=robot_body_pos_w[16],
            robot_anchor_quat_w=robot_anchor_quat_w,
            robot_body_pos_w=robot_body_pos_w,
            ref_body_pos_w=ref_body_pos_w,
            ref_anchor_quat_w=ref_anchor_quat_w,
            left_foot_contact=1.0,
            right_foot_contact=0.0,
            ref_anchor_body_index=15,
            ref_left_foot_body_index=6,
            ref_right_foot_body_index=12,
            robot_left_foot_body_id=7,
            robot_right_foot_body_id=13,
        )

        expected = np.array([0.1, -0.2, 0.3, 0.0, 0.0, 1.0, 0.0, 0.0, 0.3, 0.6], dtype=np.float32)
        np.testing.assert_allclose(features, expected, atol=1e-6)

    def test_is_deploy_wrapper_model_detects_new_input_contract(self):
        self.assertTrue(
            is_deploy_wrapper_model(
                [
                    "joint_pos",
                    "joint_vel",
                    "projected_gravity_b",
                    "base_ang_vel",
                    "robot_anchor_pos_w",
                    "robot_anchor_quat_w",
                    "robot_body_pos_w",
                    "left_foot_contact",
                    "right_foot_contact",
                    "last_action",
                    "history",
                    "time_step",
                ]
            )
        )
        self.assertFalse(is_deploy_wrapper_model(["obs", "history", "balance_features"]))

    def test_extract_robot_body_pos_by_ids_keeps_requested_order(self):
        body_pos_w = np.arange(31 * 3, dtype=np.float32).reshape(31, 3)
        extracted = extract_robot_body_pos_by_ids(body_pos_w, [1, 7, 16])
        expected = np.vstack([body_pos_w[1], body_pos_w[7], body_pos_w[16]])
        np.testing.assert_allclose(extracted, expected)

    def test_combine_base_and_residual_supports_gain_and_gate_threshold(self):
        base_action = np.array([[0.2, -0.2]], dtype=np.float32)
        residual = np.array([[0.4, -0.4]], dtype=np.float32)
        gate = np.array([[0.5]], dtype=np.float32)

        adaptive = combine_base_and_residual(
            base_action=base_action,
            residual=residual,
            gate=gate,
            adapter_gain=1.0,
            gate_threshold=0.0,
        )
        np.testing.assert_allclose(adaptive, np.array([[0.4, -0.4]], dtype=np.float32))

        suppressed = combine_base_and_residual(
            base_action=base_action,
            residual=residual,
            gate=gate,
            adapter_gain=1.0,
            gate_threshold=0.6,
        )
        np.testing.assert_allclose(suppressed, base_action)

        scaled = combine_base_and_residual(
            base_action=base_action,
            residual=residual,
            gate=gate,
            adapter_gain=0.25,
            gate_threshold=0.0,
        )
        np.testing.assert_allclose(scaled, np.array([[0.25, -0.25]], dtype=np.float32))

    def test_compute_entry_yaw_alignment_quat_cancels_initial_yaw_difference(self):
        yaw_90 = np.float32(np.sqrt(0.5))
        robot_anchor_quat_w = np.array([yaw_90, 0.0, 0.0, yaw_90], dtype=np.float32)
        ref_anchor_quat_w = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

        align_quat = compute_entry_yaw_alignment_quat(robot_anchor_quat_w, ref_anchor_quat_w)
        aligned_robot_quat = compute_motion_anchor_orientation_b(
            np.array(
                [
                    align_quat[0] * robot_anchor_quat_w[0] - align_quat[1] * robot_anchor_quat_w[1] - align_quat[2] * robot_anchor_quat_w[2] - align_quat[3] * robot_anchor_quat_w[3],
                    align_quat[0] * robot_anchor_quat_w[1] + align_quat[1] * robot_anchor_quat_w[0] + align_quat[2] * robot_anchor_quat_w[3] - align_quat[3] * robot_anchor_quat_w[2],
                    align_quat[0] * robot_anchor_quat_w[2] - align_quat[1] * robot_anchor_quat_w[3] + align_quat[2] * robot_anchor_quat_w[0] + align_quat[3] * robot_anchor_quat_w[1],
                    align_quat[0] * robot_anchor_quat_w[3] + align_quat[1] * robot_anchor_quat_w[2] - align_quat[2] * robot_anchor_quat_w[1] + align_quat[3] * robot_anchor_quat_w[0],
                ],
                dtype=np.float32,
            ),
            ref_anchor_quat_w,
        )
        np.testing.assert_allclose(aligned_robot_quat, np.eye(3, dtype=np.float32), atol=1e-5)

    def test_deploy_wrapper_receives_raw_joint_positions_not_offset_joint_pos(self):
        class DummyIo:
            def __init__(self, name):
                self.name = name

        class DummyMeta:
            custom_metadata_map = {
                "body_names": "pelvis,left_hip_roll_link,left_knee_link,left_ankle_roll_link,right_hip_roll_link,right_knee_link,right_ankle_roll_link,torso_link,left_shoulder_roll_link,left_elbow_link,left_wrist_yaw_link,right_shoulder_roll_link,right_elbow_link,right_wrist_yaw_link",
                "anchor_body_name": "torso_link",
            }

        class FakeDeploySession:
            def __init__(self, path):
                self.path = path
                self.last_inputs = None

            def get_inputs(self):
                return [
                    DummyIo("joint_pos"),
                    DummyIo("joint_vel"),
                    DummyIo("projected_gravity_b"),
                    DummyIo("base_ang_vel"),
                    DummyIo("robot_anchor_pos_w"),
                    DummyIo("robot_anchor_quat_w"),
                    DummyIo("robot_body_pos_w"),
                    DummyIo("left_foot_contact"),
                    DummyIo("right_foot_contact"),
                    DummyIo("last_action"),
                    DummyIo("history"),
                    DummyIo("time_step"),
                ]

            def get_outputs(self):
                return [
                    DummyIo("adaptive_action"),
                    DummyIo("base_action"),
                    DummyIo("residual"),
                    DummyIo("gate"),
                    DummyIo("history_out"),
                    DummyIo("obs"),
                    DummyIo("balance_features"),
                    DummyIo("joint_pos_ref"),
                    DummyIo("joint_vel_ref"),
                    DummyIo("body_pos_w_ref"),
                    DummyIo("body_quat_w_ref"),
                ]

            def get_modelmeta(self):
                return DummyMeta()

            def run(self, _, observation):
                self.last_inputs = observation
                batch = 1
                return [
                    np.zeros((batch, 29), dtype=np.float32),
                    np.zeros((batch, 29), dtype=np.float32),
                    np.zeros((batch, 29), dtype=np.float32),
                    np.zeros((batch, 1), dtype=np.float32),
                    np.zeros((batch, 50, 183), dtype=np.float32),
                    np.zeros((batch, 154), dtype=np.float32),
                    np.zeros((batch, 10), dtype=np.float32),
                    np.zeros((batch, 29), dtype=np.float32),
                    np.zeros((batch, 29), dtype=np.float32),
                    np.zeros((batch, 14, 3), dtype=np.float32),
                    np.concatenate(
                        [np.ones((batch, 14, 1), dtype=np.float32), np.zeros((batch, 14, 3), dtype=np.float32)],
                        axis=-1,
                    ),
                ]

        with mock.patch("policy.anyadapter.AnyAdapter.onnxruntime.InferenceSession", FakeDeploySession):
            state = StateAndCmd(29)
            out = PolicyOutput(29)
            policy = AnyAdapter(state, out)
            policy.enter()
            state.q = np.arange(29, dtype=np.float32)
            state.dq = np.zeros(29, dtype=np.float32)
            state.gravity_ori = np.array([0.0, 0.0, -1.0], dtype=np.float32)
            state.ang_vel = np.zeros(3, dtype=np.float32)
            state.base_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
            policy.run()

            sent_q = policy.ort_session.last_inputs["joint_pos"].reshape(-1)
            expected_raw_q = state.q[policy.mj2lab]
            expected_rel_q = expected_raw_q - policy.default_angles_lab
            np.testing.assert_allclose(sent_q, expected_raw_q)
            self.assertFalse(np.allclose(sent_q, expected_rel_q))


if __name__ == "__main__":
    unittest.main()
