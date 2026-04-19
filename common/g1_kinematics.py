import numpy as np

try:
    import mujoco
except ImportError:  # pragma: no cover - exercised via deploy_real fallback, not unit test
    mujoco = None


class G1ForwardKinematics:
    def __init__(
        self,
        xml_path: str,
        *,
        left_foot_body_name: str = "left_ankle_roll_link",
        right_foot_body_name: str = "right_ankle_roll_link",
        torso_body_name: str = "torso_link",
    ):
        if mujoco is None:
            raise ImportError("mujoco is required for G1ForwardKinematics")
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.left_foot_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, left_foot_body_name)
        self.right_foot_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, right_foot_body_name)
        self.torso_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, torso_body_name)
        self.left_foot_min_z = np.inf
        self.right_foot_min_z = np.inf

    @staticmethod
    def _normalize_quat(quat_wxyz: np.ndarray) -> np.ndarray:
        quat = np.asarray(quat_wxyz, dtype=np.float32)
        norm = np.linalg.norm(quat)
        if norm <= 0.0:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        return quat / norm

    def estimate(
        self,
        *,
        joint_pos: np.ndarray,
        base_quat: np.ndarray,
        contact_height_threshold: float = 0.03,
    ):
        qpos = np.zeros(self.model.nq, dtype=np.float64)
        qpos[3:7] = self._normalize_quat(base_quat)
        qpos[7 : 7 + self.model.nu] = np.asarray(joint_pos, dtype=np.float64)

        self.data.qpos[:] = qpos
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

        root_z_shift = -float(min(self.data.xpos[self.left_foot_body_id, 2], self.data.xpos[self.right_foot_body_id, 2]))
        self.data.qpos[2] = root_z_shift
        mujoco.mj_forward(self.model, self.data)

        body_pos_w = self.data.xpos.copy().astype(np.float32)
        body_quat_w = self.data.xquat.copy().astype(np.float32)

        left_foot_z = float(body_pos_w[self.left_foot_body_id, 2])
        right_foot_z = float(body_pos_w[self.right_foot_body_id, 2])
        self.left_foot_min_z = min(self.left_foot_min_z, left_foot_z)
        self.right_foot_min_z = min(self.right_foot_min_z, right_foot_z)

        contact_pairs = []
        if left_foot_z <= self.left_foot_min_z + contact_height_threshold:
            contact_pairs.append((self.left_foot_body_id, 0))
        if right_foot_z <= self.right_foot_min_z + contact_height_threshold:
            contact_pairs.append((self.right_foot_body_id, 0))

        return body_pos_w, body_quat_w, contact_pairs
