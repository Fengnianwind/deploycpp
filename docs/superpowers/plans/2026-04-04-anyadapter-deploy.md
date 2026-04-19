# AnyAdapter Deploy Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a standalone AnyAdapter deployment policy that can be selected in MuJoCo and real-robot flows without replacing the existing BeyondMimic policies.

**Architecture:** Implement a new `policy/anyadapter` module that loads a three-input ONNX and motion `.npz`, builds the 154-D tracking observation locally, maintains the history buffer externally, and exposes an FSM state identical in shape to the existing deployment policies. Wire the new state through `FSM`, `LocoMode`, and both deployment entrypoints using the existing `SKILL_2` command path.

**Tech Stack:** Python, NumPy, ONNX Runtime, existing FSM deployment framework, `unittest`

---

### Task 1: Add failing runtime tests

**Files:**
- Create: `tests/test_anyadapter_runtime.py`
- Test: `tests/test_anyadapter_runtime.py`

- [ ] **Step 1: Write the failing test**

```python
import unittest
import numpy as np

from policy.anyadapter.runtime import (
    append_history,
    build_balance_features,
    build_tracking_observation,
)


class AnyAdapterRuntimeTests(unittest.TestCase):
    def test_build_tracking_observation_matches_expected_order(self):
        ref_joint_pos = np.arange(29, dtype=np.float32)
        ref_joint_vel = np.arange(29, dtype=np.float32) + 100
        motion_anchor_ori_b = np.arange(6, dtype=np.float32) + 200
        ang_vel = np.arange(3, dtype=np.float32) + 300
        qj = np.arange(29, dtype=np.float32) + 400
        dqj = np.arange(29, dtype=np.float32) + 500
        last_action = np.arange(29, dtype=np.float32) + 600

        obs = build_tracking_observation(
            ref_joint_pos,
            ref_joint_vel,
            motion_anchor_ori_b,
            ang_vel,
            qj,
            dqj,
            last_action,
        )

        self.assertEqual(obs.shape, (154,))
        self.assertEqual(obs[0], 0.0)
        self.assertEqual(obs[29], 100.0)
        self.assertEqual(obs[58], 200.0)
        self.assertEqual(obs[64], 300.0)
        self.assertEqual(obs[67], 400.0)
        self.assertEqual(obs[96], 500.0)
        self.assertEqual(obs[125], 600.0)

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
        np.testing.assert_allclose(features, expected)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m unittest discover -s tests -p 'test_anyadapter_runtime.py' -v`
Expected: FAIL with `ModuleNotFoundError` for `policy.anyadapter.runtime`

- [ ] **Step 3: Write minimal implementation**

Create a small runtime helper module with `append_history`, `build_tracking_observation`, and `build_balance_features`.

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m unittest discover -s tests -p 'test_anyadapter_runtime.py' -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add tests/test_anyadapter_runtime.py policy/anyadapter/runtime.py
git commit -m "test: add anyadapter runtime coverage"
```

### Task 2: Implement standalone AnyAdapter policy

**Files:**
- Create: `policy/anyadapter/AnyAdapter.py`
- Create: `policy/anyadapter/config/AnyAdapter.yaml`
- Modify: `policy/anyadapter/runtime.py`

- [ ] **Step 1: Write the failing test**

Add a unit test that imports the new policy helper code without loading the ONNX file and checks that motion frame indexing clamps to the last frame.

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m unittest discover -s tests -p 'test_anyadapter_runtime.py' -v`
Expected: FAIL because frame-index helper does not exist yet

- [ ] **Step 3: Write minimal implementation**

Implement:
- motion `.npz` loading
- tracking observation assembly
- history maintenance
- approximate balance features
- ONNX inference returning `adaptive_action`

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m unittest discover -s tests -p 'test_anyadapter_runtime.py' -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add policy/anyadapter/AnyAdapter.py policy/anyadapter/config/AnyAdapter.yaml policy/anyadapter/runtime.py
git commit -m "feat: add anyadapter deployment policy"
```

### Task 3: Wire FSM and deployment entrypoints

**Files:**
- Modify: `common/utils.py`
- Modify: `FSM/FSM.py`
- Modify: `policy/loco_mode/LocoMode.py`
- Modify: `deploy_real/deploy_real.py`
- Modify: `deploy_mujoco/deploy_mujoco.py`

- [ ] **Step 1: Write the failing test**

Extend tests to assert:
- `FSMStateName.SKILL_ANYADAPTER` exists
- `LocoMode` maps `FSMCommand.SKILL_2` to the new state name

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m unittest discover -s tests -v`
Expected: FAIL because the enum/state mapping is not added yet

- [ ] **Step 3: Write minimal implementation**

Wire the new state through:
- enum definitions
- FSM constructor/import routing
- `LocoMode.checkChange`
- `deploy_real` `R1+Y`
- `deploy_mujoco` keyboard `R1+Y`

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m unittest discover -s tests -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add common/utils.py FSM/FSM.py policy/loco_mode/LocoMode.py deploy_real/deploy_real.py deploy_mujoco/deploy_mujoco.py
git commit -m "feat: wire anyadapter into deployment flows"
```

### Task 4: Place initial model assets and verify imports

**Files:**
- Create: `policy/anyadapter/model/`

- [ ] **Step 1: Write the failing check**

Run a small import/initialization script that verifies the configured ONNX path and motion path both exist.

- [ ] **Step 2: Run check to verify it fails**

Run: `python - <<'PY'\nfrom pathlib import Path\nfrom policy.anyadapter.config import AnyAdapter\nPY`
Expected: FAIL until the asset paths are valid

- [ ] **Step 3: Write minimal implementation**

Place the current trained ONNX into `policy/anyadapter/model/` and point config to the existing motion `.npz`.

- [ ] **Step 4: Run check to verify it passes**

Run a Python one-liner that loads the YAML and asserts both paths exist.
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add policy/anyadapter/model policy/anyadapter/config/AnyAdapter.yaml
git commit -m "chore: add anyadapter deployment assets"
```
