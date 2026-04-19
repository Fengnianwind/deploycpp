from __future__ import annotations

import numpy as np

DEPLOY_WRAPPER_REQUIRED_INPUTS = {
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
}

G1_BODY_NAME_TO_MUJOCO_ID = {
    "pelvis": 1,
    "left_hip_pitch_link": 2,
    "left_hip_roll_link": 3,
    "left_hip_yaw_link": 4,
    "left_knee_link": 5,
    "left_ankle_pitch_link": 6,
    "left_ankle_roll_link": 7,
    "right_hip_pitch_link": 8,
    "right_hip_roll_link": 9,
    "right_hip_yaw_link": 10,
    "right_knee_link": 11,
    "right_ankle_pitch_link": 12,
    "right_ankle_roll_link": 13,
    "waist_yaw_link": 14,
    "waist_roll_link": 15,
    "torso_link": 16,
    "left_shoulder_pitch_link": 17,
    "left_shoulder_roll_link": 18,
    "left_shoulder_yaw_link": 19,
    "left_elbow_link": 20,
    "left_wrist_roll_link": 21,
    "left_wrist_pitch_link": 22,
    "left_wrist_yaw_link": 23,
    "right_shoulder_pitch_link": 24,
    "right_shoulder_roll_link": 25,
    "right_shoulder_yaw_link": 26,
    "right_elbow_link": 27,
    "right_wrist_roll_link": 28,
    "right_wrist_pitch_link": 29,
    "right_wrist_yaw_link": 30,
}


def is_deploy_wrapper_model(input_names: list[str]) -> bool:
    return DEPLOY_WRAPPER_REQUIRED_INPUTS.issubset(set(input_names))


def body_ids_from_names(body_names: list[str]) -> list[int]:
    return [G1_BODY_NAME_TO_MUJOCO_ID[name] for name in body_names]


def extract_robot_body_pos_by_ids(body_pos_w: np.ndarray, body_ids: list[int] | np.ndarray) -> np.ndarray:
    source = np.asarray(body_pos_w, dtype=np.float32)
    return source[np.asarray(body_ids, dtype=np.int32)]


def extract_robot_body_quat_by_ids(body_quat_w: np.ndarray, body_ids: list[int] | np.ndarray) -> np.ndarray:
    source = np.asarray(body_quat_w, dtype=np.float32)
    return source[np.asarray(body_ids, dtype=np.int32)]


def estimate_contact_from_body_height(
    body_pos_w: np.ndarray,
    body_index: int,
    min_height: float,
    contact_height_threshold: float,
) -> float:
    if not np.isfinite(min_height):
        return 0.0
    height = float(np.asarray(body_pos_w, dtype=np.float32)[body_index, 2])
    return 1.0 if height <= min_height + contact_height_threshold else 0.0


def combine_base_and_residual(
    *,
    base_action: np.ndarray,
    residual: np.ndarray,
    gate: np.ndarray,
    adapter_gain: float,
    gate_threshold: float,
) -> np.ndarray:
    base = np.asarray(base_action, dtype=np.float32)
    residual = np.asarray(residual, dtype=np.float32)
    gate = np.asarray(gate, dtype=np.float32)
    effective_gate = np.where(gate >= np.float32(gate_threshold), gate, 0.0).astype(np.float32)
    return (base + np.float32(adapter_gain) * effective_gate * residual).astype(np.float32)


def compute_entry_yaw_alignment_quat(robot_anchor_quat_w: np.ndarray, ref_anchor_quat_w: np.ndarray) -> np.ndarray:
    robot_yaw_quat = yaw_quat(np.asarray(robot_anchor_quat_w, dtype=np.float32))
    ref_yaw_quat = yaw_quat(np.asarray(ref_anchor_quat_w, dtype=np.float32))
    return quat_mul(ref_yaw_quat, quat_inv(robot_yaw_quat))


def append_history(history: np.ndarray, obs_action: np.ndarray) -> np.ndarray:
    updated = np.roll(history, shift=-1, axis=1)
    updated[:, -1, :] = obs_action.reshape(1, -1)
    return updated


def build_balance_features(
    *,
    projected_gravity: np.ndarray,
    anchor_height_error_z: float,
    anchor_roll_pitch_error: np.ndarray,
    left_foot_contact: float,
    right_foot_contact: float,
    left_foot_height_error_z: float,
    right_foot_height_error_z: float,
) -> np.ndarray:
    projected_gravity = np.asarray(projected_gravity, dtype=np.float32).reshape(-1)
    anchor_roll_pitch_error = np.asarray(anchor_roll_pitch_error, dtype=np.float32).reshape(-1)
    double_support = 1.0 if left_foot_contact > 0.5 and right_foot_contact > 0.5 else 0.0
    return np.array(
        [
            projected_gravity[0],
            projected_gravity[1],
            np.float32(anchor_height_error_z),
            anchor_roll_pitch_error[0],
            anchor_roll_pitch_error[1],
            np.float32(left_foot_contact),
            np.float32(right_foot_contact),
            np.float32(double_support),
            np.float32(left_foot_height_error_z),
            np.float32(right_foot_height_error_z),
        ],
        dtype=np.float32,
    )


def quat_rotate_inverse(quat_wxyz: np.ndarray, vec: np.ndarray) -> np.ndarray:
    q = np.asarray(quat_wxyz, dtype=np.float32)
    v = np.asarray(vec, dtype=np.float32)
    q_xyz = q[1:]
    q_w = q[0]
    uv = np.cross(q_xyz, v)
    uuv = np.cross(q_xyz, uv)
    return v - 2.0 * (q_w * uv + uuv)


def quat_inv(q: np.ndarray) -> np.ndarray:
    quat = np.asarray(q, dtype=np.float32)
    norm_sq = float(np.dot(quat, quat))
    if norm_sq == 0.0:
        raise ValueError("Quaternion norm must be non-zero")
    return np.array([quat[0], -quat[1], -quat[2], -quat[3]], dtype=np.float32) / norm_sq


def quat_apply(quat_wxyz: np.ndarray, vec: np.ndarray) -> np.ndarray:
    rotation = matrix_from_quat(quat_wxyz)
    values = np.asarray(vec, dtype=np.float32)
    if values.ndim == 1:
        return rotation @ values
    return values @ rotation.T


def compute_anchor_roll_pitch_error_from_quats(
    robot_anchor_quat_w: np.ndarray,
    ref_anchor_quat_w: np.ndarray,
    gravity_world: np.ndarray | None = None,
) -> np.ndarray:
    gravity_world = np.array([0.0, 0.0, -1.0], dtype=np.float32) if gravity_world is None else np.asarray(gravity_world, dtype=np.float32)
    robot_gravity = quat_rotate_inverse(robot_anchor_quat_w, gravity_world)
    target_gravity = quat_rotate_inverse(ref_anchor_quat_w, gravity_world)
    return (target_gravity[:2] - robot_gravity[:2]).astype(np.float32)


def body_in_contact(contact_body_pairs: list[tuple[int, int]], target_body_id: int) -> float:
    for body_a, body_b in contact_body_pairs:
        if (body_a == target_body_id and body_b != target_body_id) or (body_b == target_body_id and body_a != target_body_id):
            return 1.0
    return 0.0


def build_balance_features_from_state(
    *,
    projected_gravity: np.ndarray,
    robot_anchor_pos_w: np.ndarray,
    robot_anchor_quat_w: np.ndarray,
    robot_body_pos_w: np.ndarray,
    ref_body_pos_w: np.ndarray,
    ref_anchor_quat_w: np.ndarray,
    left_foot_contact: float,
    right_foot_contact: float,
    ref_anchor_body_index: int,
    ref_left_foot_body_index: int,
    ref_right_foot_body_index: int,
    robot_left_foot_body_id: int,
    robot_right_foot_body_id: int,
) -> np.ndarray:
    anchor_height_error_z = float(ref_body_pos_w[ref_anchor_body_index, 2] - robot_anchor_pos_w[2])
    anchor_roll_pitch_error = compute_anchor_roll_pitch_error_from_quats(robot_anchor_quat_w, ref_anchor_quat_w)
    ref_body_pos_relative_w = compute_reference_body_pos_relative_w(
        robot_anchor_pos_w=robot_anchor_pos_w,
        robot_anchor_quat_w=robot_anchor_quat_w,
        ref_body_pos_w=ref_body_pos_w,
        ref_anchor_pos_w=ref_body_pos_w[ref_anchor_body_index],
        ref_anchor_quat_w=ref_anchor_quat_w,
    )
    left_foot_height_error_z = float(
        ref_body_pos_relative_w[ref_left_foot_body_index, 2] - robot_body_pos_w[robot_left_foot_body_id, 2]
    )
    right_foot_height_error_z = float(
        ref_body_pos_relative_w[ref_right_foot_body_index, 2] - robot_body_pos_w[robot_right_foot_body_id, 2]
    )
    return build_balance_features(
        projected_gravity=projected_gravity,
        anchor_height_error_z=anchor_height_error_z,
        anchor_roll_pitch_error=anchor_roll_pitch_error,
        left_foot_contact=left_foot_contact,
        right_foot_contact=right_foot_contact,
        left_foot_height_error_z=left_foot_height_error_z,
        right_foot_height_error_z=right_foot_height_error_z,
    )


def compute_motion_anchor_orientation_b(robot_anchor_quat_w: np.ndarray, ref_anchor_quat_w: np.ndarray) -> np.ndarray:
    relative_quat = quat_mul(quat_inv(robot_anchor_quat_w), ref_anchor_quat_w)
    return matrix_from_quat(relative_quat)


def compute_reference_body_pos_relative_w(
    *,
    robot_anchor_pos_w: np.ndarray,
    robot_anchor_quat_w: np.ndarray,
    ref_body_pos_w: np.ndarray,
    ref_anchor_pos_w: np.ndarray,
    ref_anchor_quat_w: np.ndarray,
) -> np.ndarray:
    delta_pos_w = np.asarray(robot_anchor_pos_w, dtype=np.float32).copy()
    ref_anchor_pos_w = np.asarray(ref_anchor_pos_w, dtype=np.float32)
    delta_pos_w[2] = ref_anchor_pos_w[2]
    delta_ori_w = yaw_quat(quat_mul(np.asarray(robot_anchor_quat_w, dtype=np.float32), quat_inv(ref_anchor_quat_w)))
    return delta_pos_w + quat_apply(delta_ori_w, np.asarray(ref_body_pos_w, dtype=np.float32) - ref_anchor_pos_w)


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    ww = (z1 + x1) * (x2 + y2)
    yy = (w1 - y1) * (w2 + z2)
    zz = (w1 + y1) * (w2 - z2)
    xx = ww + yy + zz
    qq = 0.5 * (xx + (z1 - x1) * (x2 - y2))
    w = qq - ww + (z1 - y1) * (y2 - z2)
    x = qq - xx + (x1 + w1) * (x2 + w2)
    y = qq - yy + (w1 - x1) * (y2 + z2)
    z = qq - zz + (z1 + y1) * (w2 - x2)
    return np.array([w, x, y, z], dtype=np.float32)


def matrix_from_quat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
        ],
        dtype=np.float32,
    )


def yaw_quat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    return np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)], dtype=np.float32)


def euler_single_axis_to_quat(angle: float, axis: str) -> np.ndarray:
    half_angle = angle * 0.5
    cos_half = np.cos(half_angle)
    sin_half = np.sin(half_angle)
    if axis == "x":
        return np.array([cos_half, sin_half, 0.0, 0.0], dtype=np.float32)
    if axis == "y":
        return np.array([cos_half, 0.0, sin_half, 0.0], dtype=np.float32)
    if axis == "z":
        return np.array([cos_half, 0.0, 0.0, sin_half], dtype=np.float32)
    raise ValueError("axis must be one of: x, y, z")
