#!/usr/bin/env python3
import json
from pathlib import Path

import numpy as np
import onnxruntime as ort
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[3]
CONFIG_PATH = PROJECT_ROOT / "policy/beyond_mimic/config/BeyondMimic.yaml"


def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
    w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]
    ww = (z1 + x1) * (x2 + y2)
    yy = (w1 - y1) * (w2 + z2)
    zz = (w1 + y1) * (w2 - z2)
    xx = ww + yy + zz
    qq = 0.5 * (xx + (z1 - x1) * (x2 - y2))
    w = qq - ww + (z1 - y1) * (y2 - z2)
    x = qq - xx + (x1 + w1) * (x2 + w2)
    y = qq - yy + (w1 - x1) * (y2 + z2)
    z = qq - zz + (z1 + y1) * (w2 - x2)
    return np.array([w, x, y, z])


def matrix_from_quat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
    ])


def yaw_quat(q):
    w, x, y, z = q
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    return np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])


def euler_single_axis_to_quat(angle, axis):
    half_angle = angle * 0.5
    cos_half = np.cos(half_angle)
    sin_half = np.sin(half_angle)
    if axis == "x":
        return np.array([cos_half, sin_half, 0.0, 0.0])
    if axis == "y":
        return np.array([cos_half, 0.0, sin_half, 0.0])
    if axis == "z":
        return np.array([cos_half, 0.0, 0.0, sin_half])
    raise ValueError(axis)


def deterministic_state():
    q = np.array([-0.20 + 0.015 * i for i in range(29)], dtype=np.float32)
    dq = np.array([0.001 * (i - 14) for i in range(29)], dtype=np.float32)
    base_quat = np.array([0.997851, 0.0137651, -0.000958114, -0.0640478], dtype=np.float32)
    ang_vel = np.array([-0.00213053, -0.00426106, -0.00213053], dtype=np.float32)
    return q, dq, base_quat, ang_vel


def to_list(array):
    return np.asarray(array, dtype=np.float32).reshape(-1).tolist()


def main():
    with CONFIG_PATH.open("r", encoding="utf-8") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    model_path = PROJECT_ROOT / "policy/beyond_mimic/model" / config["onnx_path"]
    session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
    input_names = [inp.name for inp in session.get_inputs()]

    mj2lab = np.array(config["mj2lab"], dtype=np.int32)
    default_angles_lab = np.array(config["default_angles_lab"], dtype=np.float32)
    action_scale_lab = np.array(config["action_scale_lab"], dtype=np.float32)
    num_obs = int(config["num_obs"])

    warmup = {
        input_names[0]: np.zeros((1, num_obs), dtype=np.float32),
        input_names[1]: np.zeros((1, 1), dtype=np.float32),
    }
    action, ref_joint_pos, ref_joint_vel, _, ref_body_quat_w, _, _ = session.run(None, warmup)

    q, dq, base_quat, ang_vel = deterministic_state()
    qj = q[mj2lab] - default_angles_lab
    base_torso_yaw = qj[2]
    base_torso_roll = qj[5]
    base_torso_pitch = qj[8]

    quat_yaw = euler_single_axis_to_quat(base_torso_yaw, "z")
    quat_roll = euler_single_axis_to_quat(base_torso_roll, "x")
    quat_pitch = euler_single_axis_to_quat(base_torso_pitch, "y")
    robot_quat = quat_mul(base_quat, quat_mul(quat_yaw, quat_mul(quat_roll, quat_pitch)))
    ref_anchor_ori_w = ref_body_quat_w[:, 7].squeeze(0)

    init_to_world = None
    for _ in range(2):
        init_to_anchor = matrix_from_quat(yaw_quat(ref_anchor_ori_w))
        world_to_anchor = matrix_from_quat(yaw_quat(robot_quat))
        init_to_world = world_to_anchor @ init_to_anchor.T

    motion_anchor_ori_b = (
        matrix_from_quat(robot_quat).T @ init_to_world @ matrix_from_quat(ref_anchor_ori_w)
    )
    mimic_obs = np.concatenate(
        (
            ref_joint_pos.squeeze(0),
            ref_joint_vel.squeeze(0),
            motion_anchor_ori_b[:, :2].reshape(-1),
            ang_vel,
            qj,
            dq[mj2lab],
            action.squeeze(0),
        ),
        axis=-1,
        dtype=np.float32,
    )

    observation = {
        input_names[0]: mimic_obs.reshape(1, -1).astype(np.float32),
        input_names[1]: np.array([[2]], dtype=np.float32),
    }
    action, ref_joint_pos, ref_joint_vel, _, ref_body_quat_w, _, _ = session.run(None, observation)
    target_dof_pos_mj = np.zeros(29, dtype=np.float32)
    target_dof_pos_lab = action * action_scale_lab + default_angles_lab
    target_dof_pos_mj[mj2lab] = target_dof_pos_lab.squeeze(0)

    fixture = {
        "qj_mj2lab": to_list(qj),
        "robot_quat": to_list(robot_quat),
        "motion_anchor_ori_b": to_list(motion_anchor_ori_b),
        "mimic_obs": to_list(mimic_obs),
        "target_dof_pos_mj": to_list(target_dof_pos_mj),
    }

    out_path = Path(__file__).resolve().parents[1] / "testdata/beyond_mimic_fixture.json"
    out_path.write_text(json.dumps(fixture, indent=2), encoding="utf-8")
    print(f"wrote={out_path}")


if __name__ == "__main__":
    main()
