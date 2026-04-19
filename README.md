<div align="center">
  <h1 align="center">RoboMimic Deploy</h1>
  <p align="center">
    <span>English</span> | <a href="README_zh.md">中文</a>
  </p>
</div>

<p align="center">
  <strong>Deployment workspace for Unitree G1 (29-DoF), covering Mujoco simulation, a legacy Python real-robot entry, and the current C++ onboard deployment path for Orin NX.</strong>
</p>

## Current Status

This repository has evolved from a Python-first demo into a project whose main real-robot path is `deploy_real/deploycpp`.

What is current today:

- The recommended real-robot deployment path is the C++ program under `deploy_real/deploycpp`.
- The C++ deploy path targets Unitree G1 with a 3-DoF waist and is intended to run onboard on Orin NX (`aarch64`).
- The active C++ finite-state machine currently supports 5 states:
  - `Passive`
  - `FixedPose`
  - `LocoMode`
  - `BeyondMimic`
  - `BeyondMimic2`
- `LocoMode` uses LibTorch / TorchScript.
- `BeyondMimic` and `BeyondMimic2` use ONNX Runtime.
- A launcher mode and `systemd` service are included for unattended onboard startup.

What is still in the repo but is no longer the main story of the root README:

- `deploy_real/deploy_real.py` remains as the older Python real-robot entry.
- Older policy assets such as `dance`, `kungfu`, `kick`, `skill_cast`, and `skill_cooldown` are still present under `policy/`, but the current onboard C++ flow is centered on `Passive -> FixedPose -> LocoMode / BeyondMimic / BeyondMimic2`.

## Repository Layout

| Path | Role |
| --- | --- |
| `deploy_mujoco/` | Mujoco simulation entry |
| `deploy_real/deploy_real.py` | Legacy Python real-robot entry |
| `deploy_real/deploycpp/` | Current C++ onboard deployment program |
| `deploy_real/config/real.yaml` | Robot / DDS / timing config shared by real deployment |
| `policy/loco_mode/` | Walking policy config and TorchScript model |
| `policy/beyond_mimic/` | BeyondMimic ONNX policy assets |
| `policy/beyond_mimic2/` | BeyondMimic2 ONNX policy assets |
| `tests/` | Python-side tests and utilities already present in the repo |
| `deploy_real/deploycpp/tests/` | C++ unit / smoke / FSM / launcher tests |
| `deploy_real/deploycpp/systemd/` | Onboard auto-start service files |

## Execution Paths

### 1. Mujoco Simulation

Use this when you want to validate policies and controller flow without hardware:

```bash
python deploy_mujoco/deploy_mujoco.py
```

### 2. Legacy Python Real Deployment

This path still exists, but it is no longer the main deployment direction of the project:

```bash
python deploy_real/deploy_real.py
```

### 3. Current C++ Onboard Deployment

This is the main path the project is now organized around:

- executable: `deploy_real/deploycpp/build_release/robomimic_deploycpp`
- intended platform: Unitree G1 onboard Orin NX
- dependencies: `unitree_sdk2` C++ SDK, ONNX Runtime (`aarch64`), LibTorch (`aarch64`), `yaml-cpp`, `cmake`, `g++`

The detailed onboard deployment guide lives in:

- English / Chinese mixed board-deploy note: `deploy_real/deploycpp/README.md`

## Current C++ FSM and Controls

### Runner States

The onboard runner currently uses this state machine:

```text
Passive -> FixedPose -> LocoMode / BeyondMimic / BeyondMimic2
```

Supported transitions from the current implementation:

- `Passive`
  - `Start` -> `FixedPose`
- `FixedPose`
  - `R1 + A` -> `LocoMode` after fixed pose is complete
  - `L1 + Y` -> `BeyondMimic` after fixed pose is complete
  - `R1 + B` -> `BeyondMimic2` after fixed pose is complete
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
- At any time inside the runner:
  - `Select` -> send damping and exit the runner

### Launcher Flow

The onboard launcher adds a guarded startup flow before the runner is spawned:

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

## Build and Run the Current C++ Deploy Path

### Build

From `deploy_real/deploycpp` on the robot:

```bash
cmake -S . -B build_release \
  -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT=/home/unitree/onnxruntime-linux-aarch64-1.19.2 \
  -DLIBTORCH_ROOT=/home/unitree/RoboMimic_Deploy/deploy_real/deploycpp/libtorch
cmake --build build_release -j4
```

The output binary is:

```text
build_release/robomimic_deploycpp
```

### Run Directly

```bash
./build_release/robomimic_deploycpp --run /home/unitree/RoboMimic_Deploy
```

### Run Through the Launcher

```bash
./build_release/robomimic_deploycpp --launcher /home/unitree/RoboMimic_Deploy
```

### Install the Provided `systemd` Service

Service file:

```text
deploy_real/deploycpp/systemd/robomimic-launcher.service
```

Installer script:

```text
deploy_real/deploycpp/systemd/install_robomimic_launcher_service.sh
```

The provided service runs:

```text
/home/unitree/RoboMimic_Deploy/deploy_real/deploycpp/build_release/robomimic_deploycpp --launcher /home/unitree/RoboMimic_Deploy
```

If your onboard path differs, update the service file before installing it.

## Useful CLI Checks

The current C++ binary exposes several validation and diagnostics commands:

```bash
./build_release/robomimic_deploycpp --check-config /home/unitree/RoboMimic_Deploy
./build_release/robomimic_deploycpp --check-onnx /home/unitree/RoboMimic_Deploy
./build_release/robomimic_deploycpp --check-torch /home/unitree/RoboMimic_Deploy
./build_release/robomimic_deploycpp --check-math
./build_release/robomimic_deploycpp --check-fsm /home/unitree/RoboMimic_Deploy
./build_release/robomimic_deploycpp --probe-lowstate /home/unitree/RoboMimic_Deploy
./build_release/robomimic_deploycpp --dump-beyond-mimic-alignment /home/unitree/RoboMimic_Deploy
```

These commands are useful for:

- confirming paths loaded from `real.yaml` and policy YAMLs
- verifying ONNX / Torch model interfaces
- checking quaternion / math utilities
- validating FSM transitions
- checking whether DDS `low_state` is actually arriving
- exporting a deterministic BeyondMimic alignment fixture for debugging

## Policies and Config

The current C++ path reads from:

- `deploy_real/config/real.yaml`
- `policy/loco_mode/config/LocoMode.yaml`
- `policy/beyond_mimic/config/BeyondMimic.yaml`
- `policy/beyond_mimic2/config/BeyondMimic2.yaml`

The checked-in default config currently indicates:

- `net: eth0`
- `num_joints: 29`
- `control_dt: 0.02`
- FK state estimation enabled with `g1_description/g1_29dof_rev_1_0.xml`

Current default model filenames:

- `policy/loco_mode/model/policy_29dof.pt`
- `policy/beyond_mimic/model/dance_763.onnx`
- `policy/beyond_mimic2/model/dance_763.onnx`

## Notes and Safety

- This project is intended for G1 with a 3-DoF waist.
- The current real-robot path assumes Unitree DDS communication is correctly configured on the robot-side network interface.
- For onboard deployment, do not reuse `x86_64` desktop builds or desktop ONNX / LibTorch packages on Orin NX. Build and run with `aarch64` dependencies on the robot.
- The safest workflow is still: validate in Mujoco first, then validate config / low-state / model interfaces on the robot, and only then run the onboard controller.

## Where To Read Next

- Root Chinese overview: `README_zh.md`
- Detailed onboard deployment note: `deploy_real/deploycpp/README.md`
