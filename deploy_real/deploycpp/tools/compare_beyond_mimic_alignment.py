#!/usr/bin/env python3
import json
import sys
from pathlib import Path

import numpy as np


THRESHOLDS = {
    "qj_mj2lab": 1e-5,
    "robot_quat": 1e-5,
    "motion_anchor_ori_b": 1e-5,
    "mimic_obs": 1e-4,
    "target_dof_pos_mj": 1e-4,
}


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def main():
    if len(sys.argv) != 3:
        print("usage: compare_beyond_mimic_alignment.py expected.json actual.json", file=sys.stderr)
        return 2

    expected = load(sys.argv[1])
    actual = load(sys.argv[2])
    passed = True

    for key, threshold in THRESHOLDS.items():
        exp = np.asarray(expected[key], dtype=np.float32)
        act = np.asarray(actual[key], dtype=np.float32)
        if exp.shape != act.shape:
            print(f"{key}_shape_mismatch expected={exp.shape} actual={act.shape}")
            passed = False
            continue
        error = float(np.max(np.abs(exp - act))) if exp.size else 0.0
        print(f"{key}_max_abs_error={error:.9g}")
        if error > threshold:
            passed = False

    print(f"alignment_pass={1 if passed else 0}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
