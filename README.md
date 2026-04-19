<div align="center">
  <h1 align="center">RoboMimic Deploy</h1>
  <p align="center">
    <span>English</span> | <a href="README_zh.md">中文</a>
  </p>
</div>

<p align="center">
  <strong>A focused C++ onboard deployment workspace for Unitree G1 (29-DoF) on Orin NX.</strong>
</p>

## Overview

This repository is now trimmed to the C++ onboard deployment path centered on `deploy_real/deploycpp`.

The current runtime stack is:

- `Passive`
- `FixedPose`
- `LocoMode`
- `BeyondMimic`
- `BeyondMimic2`

Model backends:

- `LocoMode` -> LibTorch / TorchScript
- `BeyondMimic` -> ONNX Runtime
- `BeyondMimic2` -> ONNX Runtime

The intended target is Unitree G1 with a 3-DoF waist, running onboard on Orin NX (`aarch64`).

## Repository Layout

| Path | Role |
| --- | --- |
| `deploy_real/deploycpp/` | Main C++ onboard deployment program |
| `deploy_real/config/real.yaml` | Runtime DDS / timing config for deploycpp |
| `policy/passive/` | Passive state config |
| `policy/fixedpose/` | FixedPose state config |
| `policy/loco_mode/` | Walking policy config and TorchScript model |
| `policy/beyond_mimic/` | BeyondMimic ONNX config and model |
| `policy/beyond_mimic2/` | BeyondMimic2 ONNX config and model |
| `g1_description/` | Robot description assets kept in the workspace |

## FSM and Controls

### Runner States

The onboard runner uses this state chain:

```text
Passive -> FixedPose -> LocoMode / BeyondMimic / BeyondMimic2
```

Supported transitions from the current C++ implementation:

- `Passive`
  - `Start` -> `FixedPose`
- `FixedPose`
  - `R1 + A` -> `LocoMode` after fixed pose completes
  - `L1 + Y` -> `BeyondMimic` after fixed pose completes
  - `R1 + B` -> `BeyondMimic2` after fixed pose completes
  - `F1` -> `Passive`
- `LocoMode`
  - `L1 + Y` -> `BeyondMimic`
  - `R1 + B` -> `BeyondMimic2`
  - `Start` -> `FixedPose`
  - `F1` -> `Passive`
- `BeyondMimic`
  - `R1 + A` -> `LocoMode`
  - `Start` -> `FixedPose`
  - `F1` -> `Passive`
- `BeyondMimic2`
  - `R1 + A` -> `LocoMode`
  - `Start` -> `FixedPose`
  - `F1` -> `Passive`
- During runner execution:
  - `Select` -> send damping and exit

### Launcher Flow

The repository also includes a guarded launcher mode:

- `Idle`
  - wait for `low_state`
  - `L2 + R2` -> `Armed`
- `Armed`
  - press `X` within the arm timeout window -> spawn runner
- `Running`
  - monitor child process
  - restart with backoff on repeated failures
- `Locked`
  - entered after repeated fast failures
  - requires re-arm

Current launcher backoff policy in code:

- arm timeout: 5 seconds
- fast-failure threshold: 2 seconds
- failure window: 60 seconds
- restart backoff schedule: `1s -> 3s -> 10s`

## Build

Run on the robot under `deploy_real/deploycpp`:

```bash
cmake -S . -B build_release \
  -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT=/home/unitree/onnxruntime-linux-aarch64-1.19.2 \
  -DLIBTORCH_ROOT=/home/unitree/RoboMimic_Deploy/deploy_real/deploycpp/libtorch
cmake --build build_release -j4
```

Output binary:

```text
build_release/robomimic_deploycpp
```

## Run

Run the controller directly:

```bash
./build_release/robomimic_deploycpp --run /home/unitree/RoboMimic_Deploy
```

Run through the launcher:

```bash
./build_release/robomimic_deploycpp --launcher /home/unitree/RoboMimic_Deploy
```

## systemd Service

Included service files:

```text
deploy_real/deploycpp/systemd/robomimic-launcher.service
deploy_real/deploycpp/systemd/install_robomimic_launcher_service.sh
```

The provided service starts:

```text
/home/unitree/RoboMimic_Deploy/deploy_real/deploycpp/build_release/robomimic_deploycpp --launcher /home/unitree/RoboMimic_Deploy
```

If your onboard path differs, update the service file before installing it.

## Diagnostics

The C++ binary exposes these checks:

```bash
./build_release/robomimic_deploycpp --check-config /home/unitree/RoboMimic_Deploy
./build_release/robomimic_deploycpp --check-onnx /home/unitree/RoboMimic_Deploy
./build_release/robomimic_deploycpp --check-torch /home/unitree/RoboMimic_Deploy
./build_release/robomimic_deploycpp --check-math
./build_release/robomimic_deploycpp --check-fsm /home/unitree/RoboMimic_Deploy
./build_release/robomimic_deploycpp --probe-lowstate /home/unitree/RoboMimic_Deploy
./build_release/robomimic_deploycpp --dump-beyond-mimic-alignment /home/unitree/RoboMimic_Deploy
```

Use them to verify:

- runtime config paths
- ONNX / Torch interfaces
- math utilities
- FSM transitions
- DDS low-state connectivity
- BeyondMimic alignment fixture export

## Runtime Config

The C++ path reads:

- `deploy_real/config/real.yaml`
- `policy/passive/config/Passive.yaml`
- `policy/fixedpose/config/FixedPose.yaml`
- `policy/loco_mode/config/LocoMode.yaml`
- `policy/beyond_mimic/config/BeyondMimic.yaml`
- `policy/beyond_mimic2/config/BeyondMimic2.yaml`

Current default runtime values in `real.yaml`:

- `net: eth0`
- `num_joints: 29`
- `lowcmd_topic: rt/lowcmd`
- `lowstate_topic: rt/lowstate`
- `control_dt: 0.02`
- `error_over_time: 5`

Current default model filenames:

- `policy/loco_mode/model/policy_29dof.pt`
- `policy/beyond_mimic/model/dance_763.onnx`
- `policy/beyond_mimic2/model/dance_763.onnx`

## Notes

- This workspace is intended for the C++ onboard deployment flow only.
- For onboard deployment, do not reuse desktop `x86_64` build artifacts or desktop ONNX / LibTorch packages on Orin NX.
- Build and run with `aarch64` dependencies on the robot.
- Read the detailed onboard deployment guide in `deploy_real/deploycpp/README.md` before first deployment.
