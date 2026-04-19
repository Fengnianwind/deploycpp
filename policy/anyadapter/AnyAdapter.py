from FSM.FSMState import FSMStateName, FSMState
from common.ctrlcomp import StateAndCmd, PolicyOutput
from common.utils import FSMCommand, progress_bar
import numpy as np
import yaml
import onnxruntime
import os
from typing import Tuple

from policy.anyadapter.runtime import (
    append_history,
    body_in_contact,
    body_ids_from_names,
    combine_base_and_residual,
    compute_entry_yaw_alignment_quat,
    euler_single_axis_to_quat,
    estimate_contact_from_body_height,
    extract_robot_body_pos_by_ids,
    extract_robot_body_quat_by_ids,
    is_deploy_wrapper_model,
    quat_mul,
)


class AnyAdapter(FSMState):
    def __init__(self, state_cmd: StateAndCmd, policy_output: PolicyOutput):
        super().__init__()
        self.state_cmd = state_cmd
        self.policy_output = policy_output
        self.name = FSMStateName.SKILL_ANYADAPTER
        self.name_str = "anyadapter"
        self.counter_step = 0
        self.ref_motion_phase = 0.0

        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(current_dir, "config", "AnyAdapter.yaml")
        with open(config_path, "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        self.onnx_path = os.path.join(current_dir, "model", config["onnx_path"])
        self.kps_lab = np.array(config["kp_lab"], dtype=np.float32)
        self.kds_lab = np.array(config["kd_lab"], dtype=np.float32)
        self.default_angles_lab = np.array(config["default_angles_lab"], dtype=np.float32)
        self.mj2lab = np.array(config["mj2lab"], dtype=np.int32)
        self.tau_limit = np.array(config["tau_limit"], dtype=np.float32)
        self.num_actions = config["num_actions"]
        self.num_obs = config["num_obs"]
        self.action_scale_lab = np.array(config["action_scale_lab"], dtype=np.float32)
        self.history_length = int(config["history_length"])
        self.contact_height_threshold = float(config["contact_height_threshold"])
        self.control_dt = 0.02
        self.adapter_gain = float(config.get("adapter_gain", 1.0))
        self.gate_threshold = float(config.get("gate_threshold", 0.0))
        self.action_source = str(config.get("action_source", "adaptive")).strip().lower()
        self.debug_print_every = int(config.get("debug_print_every", 0))

        if not os.path.isfile(self.onnx_path):
            raise FileNotFoundError(f"AnyAdapter ONNX not found: {self.onnx_path}")

        self.ort_session = onnxruntime.InferenceSession(self.onnx_path)
        self.input_names = [item.name for item in self.ort_session.get_inputs()]
        self.output_names = [item.name for item in self.ort_session.get_outputs()]
        self.metadata = dict(self.ort_session.get_modelmeta().custom_metadata_map)
        self.is_deploy_wrapper = is_deploy_wrapper_model(self.input_names)
        if not self.is_deploy_wrapper:
            raise ValueError(
                "AnyAdapter now requires deploy-wrapper ONNX inputs. "
                f"Got inputs: {self.input_names}"
            )

        self.left_foot_min_z = np.inf
        self.right_foot_min_z = np.inf

        body_names_csv = self.metadata.get("body_names", "")
        if not body_names_csv:
            raise ValueError("Deploy AnyAdapter ONNX is missing body_names metadata")
        self.deploy_body_names = [name.strip() for name in body_names_csv.split(",") if name.strip()]
        self.deploy_anchor_body_name = self.metadata.get("anchor_body_name", "torso_link")
        self.deploy_anchor_body_index = self.deploy_body_names.index(self.deploy_anchor_body_name)
        self.deploy_left_foot_body_index = self.deploy_body_names.index("left_ankle_roll_link")
        self.deploy_right_foot_body_index = self.deploy_body_names.index("right_ankle_roll_link")
        self.deploy_body_ids = np.array(body_ids_from_names(self.deploy_body_names), dtype=np.int32)
        self.deploy_anchor_body_id = int(self.deploy_body_ids[self.deploy_anchor_body_index])
        self.deploy_left_foot_body_id = int(self.deploy_body_ids[self.deploy_left_foot_body_index])
        self.deploy_right_foot_body_id = int(self.deploy_body_ids[self.deploy_right_foot_body_index])
        self.deploy_body_pos_est = np.zeros((len(self.deploy_body_names), 3), dtype=np.float32)
        self.deploy_body_quat_est = np.zeros((len(self.deploy_body_names), 4), dtype=np.float32)
        self.deploy_body_quat_est[:, 0] = 1.0
        self.motion_length = float(config.get("motion_length", 0.0))
        self.obs = np.zeros(self.num_obs, dtype=np.float32)
        self.last_action = np.zeros(self.num_actions, dtype=np.float32)
        self.history = np.zeros((1, self.history_length, self.num_obs + self.num_actions), dtype=np.float32)
        self.last_gate_value = 0.0
        self.last_residual_l2 = 0.0
        self.last_base_action = np.zeros(self.num_actions, dtype=np.float32)
        self.last_residual = np.zeros(self.num_actions, dtype=np.float32)
        self.entry_yaw_align_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.entry_yaw_align_initialized = False

        print("AnyAdapter policy initializing ...")

    def _reset_runtime_state(self):
        self.obs.fill(0.0)
        self.last_action.fill(0.0)
        self.history.fill(0.0)
        self.last_gate_value = 0.0
        self.last_residual_l2 = 0.0
        self.last_base_action.fill(0.0)
        self.last_residual.fill(0.0)
        self.entry_yaw_align_quat[:] = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.entry_yaw_align_initialized = False
        self.deploy_body_pos_est.fill(0.0)
        self.deploy_body_quat_est.fill(0.0)
        self.deploy_body_quat_est[:, 0] = 1.0
        self.left_foot_min_z = np.inf
        self.right_foot_min_z = np.inf

    def enter(self):
        self.counter_step = 0
        self.ref_motion_phase = 0.0
        self._reset_runtime_state()

    def _compute_robot_anchor_quat(self) -> np.ndarray:
        robot_quat = np.asarray(self.state_cmd.base_quat, dtype=np.float32)
        qj = self.state_cmd.q[self.mj2lab] - self.default_angles_lab

        quat_yaw = euler_single_axis_to_quat(float(qj[2]), "z")
        quat_roll = euler_single_axis_to_quat(float(qj[5]), "x")
        quat_pitch = euler_single_axis_to_quat(float(qj[8]), "y")

        temp = quat_mul(quat_roll, quat_pitch)
        torso_quat = quat_mul(quat_yaw, temp)
        return quat_mul(robot_quat, torso_quat)

    def _deploy_fallback_inputs(self, robot_anchor_quat_w: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float, float]:
        robot_body_pos_w = self.deploy_body_pos_est.copy()
        robot_body_quat_w = self.deploy_body_quat_est.copy()
        robot_body_quat_w[self.deploy_anchor_body_index] = robot_anchor_quat_w
        robot_anchor_pos_w = robot_body_pos_w[self.deploy_anchor_body_index].copy()
        left_foot_contact = estimate_contact_from_body_height(
            robot_body_pos_w,
            self.deploy_left_foot_body_index,
            self.left_foot_min_z,
            self.contact_height_threshold,
        )
        right_foot_contact = estimate_contact_from_body_height(
            robot_body_pos_w,
            self.deploy_right_foot_body_index,
            self.right_foot_min_z,
            self.contact_height_threshold,
        )
        return robot_anchor_pos_w, robot_body_pos_w, left_foot_contact, right_foot_contact

    def _build_deploy_observation(
        self,
        raw_qj: np.ndarray,
        dqj: np.ndarray,
        ang_vel: np.ndarray,
        robot_anchor_quat_w: np.ndarray,
        apply_entry_yaw_alignment: bool,
    ):
        use_exact_body_state = self.state_cmd.body_pos_w is not None and len(self.state_cmd.body_pos_w) > int(self.deploy_body_ids.max())

        if use_exact_body_state:
            robot_body_pos_w = extract_robot_body_pos_by_ids(self.state_cmd.body_pos_w, self.deploy_body_ids)
            robot_anchor_pos_w = robot_body_pos_w[self.deploy_anchor_body_index]
            if self.state_cmd.body_quat_w is not None:
                robot_anchor_quat_w = np.asarray(
                    self.state_cmd.body_quat_w[self.deploy_anchor_body_id], dtype=np.float32
                )
                self.deploy_body_quat_est = extract_robot_body_quat_by_ids(self.state_cmd.body_quat_w, self.deploy_body_ids)
            left_foot_contact = body_in_contact(self.state_cmd.contact_body_pairs, self.deploy_left_foot_body_id)
            right_foot_contact = body_in_contact(self.state_cmd.contact_body_pairs, self.deploy_right_foot_body_id)
        else:
            robot_anchor_pos_w, robot_body_pos_w, left_foot_contact, right_foot_contact = self._deploy_fallback_inputs(
                robot_anchor_quat_w
            )

        robot_anchor_quat_input = robot_anchor_quat_w
        if apply_entry_yaw_alignment:
            robot_anchor_quat_input = quat_mul(self.entry_yaw_align_quat, robot_anchor_quat_w)

        observation = {
            "joint_pos": raw_qj.reshape(1, -1).astype(np.float32),
            "joint_vel": dqj.reshape(1, -1).astype(np.float32),
            "projected_gravity_b": np.asarray(self.state_cmd.gravity_ori, dtype=np.float32).reshape(1, -1),
            "base_ang_vel": ang_vel.reshape(1, -1).astype(np.float32),
            "robot_anchor_pos_w": robot_anchor_pos_w.reshape(1, -1).astype(np.float32),
            "robot_anchor_quat_w": robot_anchor_quat_input.reshape(1, -1).astype(np.float32),
            "robot_body_pos_w": robot_body_pos_w.reshape(1, len(self.deploy_body_names), 3).astype(np.float32),
            "left_foot_contact": np.array([[left_foot_contact]], dtype=np.float32),
            "right_foot_contact": np.array([[right_foot_contact]], dtype=np.float32),
            "last_action": self.last_action.reshape(1, -1).astype(np.float32),
            "history": self.history.astype(np.float32),
            "time_step": np.array([[self.counter_step]], dtype=np.float32),
        }
        return observation, robot_anchor_quat_w

    def _run_deploy_wrapper(self, raw_qj: np.ndarray, dqj: np.ndarray, ang_vel: np.ndarray, robot_anchor_quat_w: np.ndarray):
        if not self.entry_yaw_align_initialized:
            warm_observation, measured_robot_anchor_quat_w = self._build_deploy_observation(
                raw_qj=raw_qj,
                dqj=dqj,
                ang_vel=ang_vel,
                robot_anchor_quat_w=robot_anchor_quat_w,
                apply_entry_yaw_alignment=False,
            )
            warm_outputs = self.ort_session.run(None, warm_observation)
            warm_output_map = {name: value for name, value in zip(self.output_names, warm_outputs)}
            if "body_quat_w_ref" in warm_output_map:
                ref_body_quat_w = warm_output_map["body_quat_w_ref"].astype(np.float32).reshape(len(self.deploy_body_names), 4)
                ref_anchor_quat_w = ref_body_quat_w[self.deploy_anchor_body_index]
                self.entry_yaw_align_quat = compute_entry_yaw_alignment_quat(
                    measured_robot_anchor_quat_w,
                    ref_anchor_quat_w,
                ).astype(np.float32)
            self.entry_yaw_align_initialized = True

        observation, _ = self._build_deploy_observation(
            raw_qj=raw_qj,
            dqj=dqj,
            ang_vel=ang_vel,
            robot_anchor_quat_w=robot_anchor_quat_w,
            apply_entry_yaw_alignment=True,
        )
        outputs = self.ort_session.run(None, observation)
        output_map = {name: value for name, value in zip(self.output_names, outputs)}
        base_action = output_map["base_action"].astype(np.float32).reshape(1, -1)
        residual = output_map["residual"].astype(np.float32).reshape(1, -1)
        gate = output_map["gate"].astype(np.float32).reshape(1, -1)
        adaptive_action = output_map["adaptive_action"].astype(np.float32).reshape(1, -1)

        if self.action_source == "base":
            selected_action = base_action
        elif self.action_source == "blended":
            selected_action = combine_base_and_residual(
                base_action=base_action,
                residual=residual,
                gate=gate,
                adapter_gain=self.adapter_gain,
                gate_threshold=self.gate_threshold,
            )
        else:
            selected_action = adaptive_action

        if "obs" in output_map:
            self.obs = output_map["obs"].astype(np.float32).reshape(-1)
        if "history_out" in output_map:
            if self.action_source == "adaptive":
                self.history = output_map["history_out"].astype(np.float32)
            else:
                obs_action = np.concatenate((self.obs, selected_action.squeeze(0)), axis=0).astype(np.float32)
                self.history = append_history(self.history, obs_action)
        else:
            obs_action = np.concatenate((self.obs, selected_action.squeeze(0)), axis=0).astype(np.float32)
            self.history = append_history(self.history, obs_action)

        if "body_pos_w_ref" in output_map:
            self.deploy_body_pos_est = output_map["body_pos_w_ref"].astype(np.float32).reshape(len(self.deploy_body_names), 3)
            self.left_foot_min_z = min(self.left_foot_min_z, float(self.deploy_body_pos_est[self.deploy_left_foot_body_index, 2]))
            self.right_foot_min_z = min(self.right_foot_min_z, float(self.deploy_body_pos_est[self.deploy_right_foot_body_index, 2]))
        if "body_quat_w_ref" in output_map:
            self.deploy_body_quat_est = output_map["body_quat_w_ref"].astype(np.float32).reshape(len(self.deploy_body_names), 4)

        self.last_gate_value = float(gate.reshape(-1)[0])
        self.last_residual_l2 = float(np.linalg.norm(residual.reshape(-1)))
        self.last_base_action = base_action.reshape(-1).astype(np.float32)
        self.last_residual = residual.reshape(-1).astype(np.float32)
        return selected_action

    def run(self):
        raw_qj = self.state_cmd.q[self.mj2lab]
        dqj = self.state_cmd.dq[self.mj2lab]
        ang_vel = np.asarray(self.state_cmd.ang_vel, dtype=np.float32)
        robot_quat = self._compute_robot_anchor_quat()
        adaptive_action = self._run_deploy_wrapper(
            raw_qj=raw_qj,
            dqj=dqj,
            ang_vel=ang_vel,
            robot_anchor_quat_w=robot_quat,
        )

        target_dof_pos_lab = adaptive_action * self.action_scale_lab + self.default_angles_lab
        target_dof_pos_mj = np.zeros(29, dtype=np.float32)
        target_dof_pos_mj[self.mj2lab] = target_dof_pos_lab.squeeze(0)

        self.policy_output.actions = target_dof_pos_mj
        self.policy_output.kps[self.mj2lab] = self.kps_lab
        self.policy_output.kds[self.mj2lab] = self.kds_lab

        self.last_action = adaptive_action.squeeze(0)
        if self.debug_print_every > 0 and self.counter_step % self.debug_print_every == 0:
            print(
                f"\n[AnyAdapter] source={self.action_source} gate={self.last_gate_value:.3f} "
                f"res_l2={self.last_residual_l2:.3f} gain={self.adapter_gain:.2f}",
                flush=True,
            )

        self.counter_step += 1
        motion_time = min(self.counter_step * self.control_dt, self.motion_length)
        self.ref_motion_phase = motion_time / self.motion_length if self.motion_length > 0 else 0.0
        print(progress_bar(motion_time, self.motion_length), end="", flush=True)

    def exit(self):
        self.counter_step = 0
        self.ref_motion_phase = 0.0
        self._reset_runtime_state()
        print()

    def checkChange(self):
        if self.state_cmd.skill_cmd == FSMCommand.LOCO:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_COOLDOWN
        if self.state_cmd.skill_cmd == FSMCommand.PASSIVE:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.PASSIVE
        if self.state_cmd.skill_cmd == FSMCommand.POS_RESET:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.FIXEDPOSE
        self.state_cmd.skill_cmd = FSMCommand.INVALID
        return FSMStateName.SKILL_ANYADAPTER
