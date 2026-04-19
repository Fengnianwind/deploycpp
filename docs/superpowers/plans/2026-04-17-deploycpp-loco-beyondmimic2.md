# DeployCpp LocoMode + BeyondMimic2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend `deploy_real/deploycpp` to support `LocoMode` via LibTorch and `BeyondMimic2` via ONNX while preserving the existing `Passive -> FixedPose -> BeyondMimic` behavior.

**Architecture:** Keep the current ONNX path intact for `BeyondMimic` and add a separate `TorchPolicyRunner` only for `LocoMode`. Expand the FSM from three states to five states, add joystick velocity plumbing in `StateAndCmd`/`RobotIO`, and validate the new paths through small C++ smoke tests plus the existing CLI checks.

**Tech Stack:** C++17, CMake, ONNX Runtime, LibTorch, yaml-cpp, unitree_sdk2, ctest

> Status note (2026-04-17): implementation, automated checks, and local real-robot smoke validation are complete. `commit` steps remain pending because this machine still lacks configured `git user.name` / `git user.email`.

---

### Task 1: Expand Shared Types and Config Surface

**Files:**
- Modify: `deploy_real/deploycpp/src/types/StateAndCmd.h`
- Modify: `deploy_real/deploycpp/src/config/Config.h`
- Modify: `deploy_real/deploycpp/src/config/Config.cpp`
- Modify: `deploy_real/deploycpp/src/main.cpp`
- Modify: `deploy_real/deploycpp/CMakeLists.txt`
- Create: `deploy_real/deploycpp/tests/ConfigSurfaceTest.cpp`

- [x] **Step 1: Write the failing config/state-surface test**

Add `tests/ConfigSurfaceTest.cpp` with explicit expectations for the new config fields and `vel_cmd`:

```cpp
#include <iostream>

#include "config/Config.h"
#include "types/StateAndCmd.h"

int main() {
  const Config config = loadConfig("/home/abc/RoboMimic_Deploy");

  if (config.loco_mode_model_path.find("policy/loco_mode/model/") == std::string::npos) {
    std::cerr << "missing loco model path\n";
    return 1;
  }
  if (config.beyond_mimic2_model_path.find("policy/beyond_mimic2/model/") == std::string::npos) {
    std::cerr << "missing beyond mimic2 model path\n";
    return 1;
  }

  StateAndCmd state;
  state.vel_cmd[0] = 0.1f;
  state.vel_cmd[1] = -0.2f;
  state.vel_cmd[2] = 0.3f;
  if (state.vel_cmd[0] != 0.1f || state.vel_cmd[1] != -0.2f || state.vel_cmd[2] != 0.3f) {
    std::cerr << "vel_cmd storage mismatch\n";
    return 1;
  }

  std::cout << "config_surface_test_pass=1\n";
  return 0;
}
```

- [ ] **Step 2: Register the new test and run it to verify RED**

Extend `CMakeLists.txt` with a new executable and test entry:

```cmake
add_executable(config_surface_test
  src/config/Config.cpp
  tests/ConfigSurfaceTest.cpp
)

target_include_directories(config_surface_test PRIVATE src)
target_link_libraries(config_surface_test PRIVATE yaml-cpp)

add_test(NAME config_surface_test COMMAND config_surface_test)
```

Run:

```bash
cd /home/abc/RoboMimic_Deploy/deploy_real/deploycpp
cmake -S . -B build -DONNXRUNTIME_ROOT=/home/abc/onnxruntime-linux-x64-1.19.2
cmake --build build -j4 --target config_surface_test
ctest --test-dir build -R config_surface_test --output-on-failure
```

Expected: FAIL because `Config` and `StateAndCmd` do not yet expose the new fields.

- [x] **Step 3: Implement the shared surface changes**

Update `src/types/StateAndCmd.h` to add `vel_cmd` and the new state names:

```cpp
enum class FSMCommand {
  Invalid,
  Passive,
  FixedPose,
  LocoMode,
  BeyondMimic,
  BeyondMimic2,
  Exit,
};

struct StateAndCmd {
  std::array<float, 29> q{};
  std::array<float, 29> dq{};
  std::array<float, 4> base_quat{1.0f, 0.0f, 0.0f, 0.0f};
  std::array<float, 3> ang_vel{};
  std::array<float, 3> vel_cmd{};
  RemoteState remote;
  FSMCommand skill_cmd = FSMCommand::Invalid;
};
```

Update `src/config/Config.h` / `src/config/Config.cpp` to load the additional policy files:

```cpp
std::string loco_mode_yaml_path;
std::string loco_mode_model_path;
std::string beyond_mimic2_yaml_path;
std::string beyond_mimic2_model_path;
```

```cpp
config.loco_mode_yaml_path = joinPath(project_root, "policy/loco_mode/config/LocoMode.yaml");
config.beyond_mimic2_yaml_path = joinPath(project_root, "policy/beyond_mimic2/config/BeyondMimic2.yaml");

const YAML::Node loco = YAML::LoadFile(config.loco_mode_yaml_path);
config.loco_mode_model_path =
    joinPath(project_root, "policy/loco_mode/model/" + requiredValue<std::string>(loco, "policy_path"));

const YAML::Node beyond_mimic2 = YAML::LoadFile(config.beyond_mimic2_yaml_path);
config.beyond_mimic2_model_path =
    joinPath(project_root, "policy/beyond_mimic2/model/" + requiredValue<std::string>(beyond_mimic2, "onnx_path"));
```

Update `src/main.cpp` `stateName()` and `--check-config` output:

```cpp
std::cout << "loco_model=" << config.loco_mode_model_path << '\n';
std::cout << "beyond_mimic2_model=" << config.beyond_mimic2_model_path << '\n';
```

- [x] **Step 4: Re-run the test to verify GREEN**

Run:

```bash
cd /home/abc/RoboMimic_Deploy/deploy_real/deploycpp
cmake --build build -j4 --target config_surface_test robomimic_deploycpp
ctest --test-dir build -R config_surface_test --output-on-failure
./build/robomimic_deploycpp --check-config /home/abc/RoboMimic_Deploy
```

Expected:

- `ctest` passes
- `--check-config` now prints `loco_model=` and `beyond_mimic2_model=`

- [ ] **Step 5: Commit**

```bash
git -C /home/abc/RoboMimic_Deploy add \
  deploy_real/deploycpp/src/types/StateAndCmd.h \
  deploy_real/deploycpp/src/config/Config.h \
  deploy_real/deploycpp/src/config/Config.cpp \
  deploy_real/deploycpp/src/main.cpp \
  deploy_real/deploycpp/CMakeLists.txt \
  deploy_real/deploycpp/tests/ConfigSurfaceTest.cpp
git -C /home/abc/RoboMimic_Deploy commit -m "feat: extend deploycpp config and state surface"
```

### Task 2: Add LibTorch Build Integration and TorchPolicyRunner

**Files:**
- Modify: `deploy_real/deploycpp/CMakeLists.txt`
- Create: `deploy_real/deploycpp/src/policy/TorchPolicyRunner.h`
- Create: `deploy_real/deploycpp/src/policy/TorchPolicyRunner.cpp`
- Create: `deploy_real/deploycpp/tests/TorchPolicyRunnerSmokeTest.cpp`
- Modify: `deploy_real/deploycpp/src/main.cpp`

- [x] **Step 1: Write the failing LibTorch smoke test**

Add `tests/TorchPolicyRunnerSmokeTest.cpp`:

```cpp
#include <iostream>
#include <vector>

#include "config/Config.h"
#include "policy/TorchPolicyRunner.h"

int main() {
  const Config config = loadConfig("/home/abc/RoboMimic_Deploy");
  TorchPolicyRunner runner(config.loco_mode_model_path);
  const std::vector<float> obs(96, 0.0f);
  const std::vector<float> action = runner.run(obs);
  if (action.size() != 29) {
    std::cerr << "expected 29 actions, got " << action.size() << "\n";
    return 1;
  }
  std::cout << "torch_policy_runner_smoke_test_pass=1\n";
  return 0;
}
```

- [ ] **Step 2: Register the smoke test and run it to verify RED**

Extend `CMakeLists.txt` to find LibTorch and register the test:

```cmake
if(NOT DEFINED LIBTORCH_ROOT)
  set(LIBTORCH_ROOT "/home/abc/unitree_rl_gym/deploy/deploy_real/cpp_g1/libtorch" CACHE PATH "LibTorch root")
endif()

list(PREPEND CMAKE_PREFIX_PATH "${LIBTORCH_ROOT}")
find_package(Torch REQUIRED)
```

```cmake
add_executable(torch_policy_runner_smoke_test
  src/config/Config.cpp
  src/policy/TorchPolicyRunner.cpp
  tests/TorchPolicyRunnerSmokeTest.cpp
)

target_include_directories(torch_policy_runner_smoke_test PRIVATE
  src
  "${ONNXRUNTIME_INCLUDE_DIR}"
  ${TORCH_INCLUDE_DIRS}
)

target_link_libraries(torch_policy_runner_smoke_test PRIVATE
  yaml-cpp
  "${TORCH_LIBRARIES}"
)

add_test(NAME torch_policy_runner_smoke_test COMMAND torch_policy_runner_smoke_test)
```

Run:

```bash
cd /home/abc/RoboMimic_Deploy/deploy_real/deploycpp
cmake -S . -B build \
  -DONNXRUNTIME_ROOT=/home/abc/onnxruntime-linux-x64-1.19.2 \
  -DLIBTORCH_ROOT=/home/abc/unitree_rl_gym/deploy/deploy_real/cpp_g1/libtorch
cmake --build build -j4 --target torch_policy_runner_smoke_test
ctest --test-dir build -R torch_policy_runner_smoke_test --output-on-failure
```

Expected: FAIL because `TorchPolicyRunner` does not exist yet.

- [x] **Step 3: Implement `TorchPolicyRunner` and `--check-torch`**

Create `src/policy/TorchPolicyRunner.h`:

```cpp
#pragma once

#include <string>
#include <vector>

#include <torch/script.h>

class TorchPolicyRunner {
 public:
  explicit TorchPolicyRunner(const std::string& model_path);
  std::vector<float> run(const std::vector<float>& obs);

 private:
  torch::jit::script::Module module_;
};
```

Create `src/policy/TorchPolicyRunner.cpp`:

```cpp
#include "policy/TorchPolicyRunner.h"

#include <stdexcept>

TorchPolicyRunner::TorchPolicyRunner(const std::string& model_path) {
  module_ = torch::jit::load(model_path);
  module_.eval();
}

std::vector<float> TorchPolicyRunner::run(const std::vector<float>& obs) {
  if (obs.size() != 96) {
    throw std::runtime_error("Expected obs size 96");
  }
  torch::NoGradGuard no_grad;
  torch::Tensor input = torch::from_blob(
      const_cast<float*>(obs.data()), {1, static_cast<long>(obs.size())}, torch::kFloat32).clone();
  torch::Tensor output = module_.forward({input}).toTensor().to(torch::kCPU).contiguous();
  return std::vector<float>(output.data_ptr<float>(), output.data_ptr<float>() + output.numel());
}
```

Add `--check-torch` to `src/main.cpp`:

```cpp
int checkTorch(const std::string& project_root) {
  const Config config = loadConfig(project_root);
  std::cout << "model=" << config.loco_mode_model_path << '\n';
  TorchPolicyRunner runner(config.loco_mode_model_path);
  const std::vector<float> obs(96, 0.0f);
  const std::vector<float> action = runner.run(obs);
  std::cout << "actions_size=" << action.size() << '\n';
  return action.size() == 29 ? 0 : 1;
}
```

- [x] **Step 4: Run the smoke test and CLI check to verify GREEN**

Run:

```bash
cd /home/abc/RoboMimic_Deploy/deploy_real/deploycpp
cmake --build build -j4 --target torch_policy_runner_smoke_test robomimic_deploycpp
ctest --test-dir build -R torch_policy_runner_smoke_test --output-on-failure
./build/robomimic_deploycpp --check-torch /home/abc/RoboMimic_Deploy
```

Expected:

- `torch_policy_runner_smoke_test` passes
- `--check-torch` prints `actions_size=29`

- [ ] **Step 5: Commit**

```bash
git -C /home/abc/RoboMimic_Deploy add \
  deploy_real/deploycpp/CMakeLists.txt \
  deploy_real/deploycpp/src/policy/TorchPolicyRunner.h \
  deploy_real/deploycpp/src/policy/TorchPolicyRunner.cpp \
  deploy_real/deploycpp/tests/TorchPolicyRunnerSmokeTest.cpp \
  deploy_real/deploycpp/src/main.cpp
git -C /home/abc/RoboMimic_Deploy commit -m "feat: add libtorch policy runner for deploycpp"
```

### Task 3: Implement LocoModeState and Joystick Velocity Plumbing

**Files:**
- Modify: `deploy_real/deploycpp/src/RobotIO.cpp`
- Modify: `deploy_real/deploycpp/src/states/FSMState.h`
- Create: `deploy_real/deploycpp/src/states/LocoModeState.h`
- Create: `deploy_real/deploycpp/src/states/LocoModeState.cpp`
- Modify: `deploy_real/deploycpp/src/FSMController.h`
- Modify: `deploy_real/deploycpp/src/FSMController.cpp`

- [x] **Step 1: Write the failing LocoMode state smoke test**

Create `tests/LocoModeStateTest.cpp`:

```cpp
#include <iostream>

#include "config/Config.h"
#include "states/LocoModeState.h"

int main() {
  const Config config = loadConfig("/home/abc/RoboMimic_Deploy");
  LocoModeState state(config);
  StateAndCmd input;
  PolicyOutput output;
  state.enter(input, output);
  state.run(input, output);
  if (!output.isFinite()) {
    std::cerr << "loco output not finite\n";
    return 1;
  }
  std::cout << "loco_mode_state_test_pass=1\n";
  return 0;
}
```

- [ ] **Step 2: Run the test to verify RED**

Register the test in `CMakeLists.txt`, then run:

```bash
cd /home/abc/RoboMimic_Deploy/deploy_real/deploycpp
cmake --build build -j4 --target loco_mode_state_test
ctest --test-dir build -R loco_mode_state_test --output-on-failure
```

Expected: FAIL because `LocoModeState` does not exist yet.

- [x] **Step 3: Implement `vel_cmd` plumbing and `LocoModeState`**

Update `src/RobotIO.cpp`:

```cpp
state.remote = decodeRemote(low_state_);
state.vel_cmd[0] = state.remote.ly;
state.vel_cmd[1] = -state.remote.lx;
state.vel_cmd[2] = -state.remote.rx;
```

Create `src/states/LocoModeState.h`:

```cpp
#pragma once

#include <vector>

#include "config/Config.h"
#include "policy/TorchPolicyRunner.h"
#include "states/FSMState.h"

class LocoModeState : public FSMState {
 public:
  explicit LocoModeState(const Config& config);
  FSMStateName name() const override;
  void enter(const StateAndCmd& state, PolicyOutput& output) override;
  void run(const StateAndCmd& state, PolicyOutput& output) override;
  void exit(const StateAndCmd& state, PolicyOutput& output) override;
  FSMStateName checkChange(const StateAndCmd& state) const override;

 private:
  void loadYaml(const std::string& yaml_path);
  std::vector<float> buildObs(const StateAndCmd& state) const;
  void writeOutput(const std::vector<float>& action, PolicyOutput& output) const;

  TorchPolicyRunner runner_;
  std::vector<float> prev_action_;
  std::vector<float> kps_;
  std::vector<float> kds_;
  std::vector<float> default_angles_;
  std::vector<int> joint2motor_idx_;
  std::vector<float> cmd_scale_;
  std::vector<float> cmd_min_;
  std::vector<float> cmd_max_;
  float ang_vel_scale_ = 1.0f;
  float dof_pos_scale_ = 1.0f;
  float dof_vel_scale_ = 1.0f;
  float action_scale_ = 1.0f;
};
```

Implement `buildObs()` in `src/states/LocoModeState.cpp` to match the Python layout:

```cpp
obs[0..2]   = scaled ang_vel
obs[3..5]   = gravity orientation from base_quat
obs[6..8]   = scaled vel_cmd mapped into cmd_range
obs[9..37]  = reordered q
obs[38..66] = reordered dq
obs[67..95] = prev_action
```

Implement `checkChange()`:

```cpp
if (state.remote.F1) return FSMStateName::Passive;
if (state.remote.start) return FSMStateName::FixedPose;
if (state.remote.L1 && state.remote.Y) return FSMStateName::BeyondMimic;
if (state.remote.R1 && state.remote.B) return FSMStateName::BeyondMimic2;
return FSMStateName::LocoMode;
```

- [x] **Step 4: Re-run the LocoMode test**

Run:

```bash
cd /home/abc/RoboMimic_Deploy/deploy_real/deploycpp
cmake --build build -j4 --target loco_mode_state_test fsm_controller_test
ctest --test-dir build -R "loco_mode_state_test|fsm_controller_test" --output-on-failure
```

Expected:

- `loco_mode_state_test` passes

- [ ] **Step 5: Commit**

```bash
git -C /home/abc/RoboMimic_Deploy add \
  deploy_real/deploycpp/src/RobotIO.cpp \
  deploy_real/deploycpp/src/states/FSMState.h \
  deploy_real/deploycpp/src/states/LocoModeState.h \
  deploy_real/deploycpp/src/states/LocoModeState.cpp \
  deploy_real/deploycpp/src/FSMController.h \
  deploy_real/deploycpp/src/FSMController.cpp
git -C /home/abc/RoboMimic_Deploy commit -m "feat: add locomode state to deploycpp"
```

### Task 4: Implement BeyondMimic2State and Complete FSM Wiring

**Files:**
- Create: `deploy_real/deploycpp/src/states/BeyondMimic2State.h`
- Create: `deploy_real/deploycpp/src/states/BeyondMimic2State.cpp`
- Modify: `deploy_real/deploycpp/src/FSMController.h`
- Modify: `deploy_real/deploycpp/src/FSMController.cpp`
- Modify: `deploy_real/deploycpp/src/states/FixedPoseState.cpp`
- Modify: `deploy_real/deploycpp/src/states/BeyondMimicState.cpp`
- Modify: `deploy_real/deploycpp/src/main.cpp`
- Create: `deploy_real/deploycpp/tests/FSMControllerTest.cpp`

- [x] **Step 1: Write the failing BeyondMimic2 smoke test**

Create `tests/BeyondMimic2StateTest.cpp`:

```cpp
#include <iostream>

#include "config/Config.h"
#include "states/BeyondMimic2State.h"

int main() {
  const Config config = loadConfig("/home/abc/RoboMimic_Deploy");
  BeyondMimic2State state(config);
  StateAndCmd input;
  PolicyOutput output;

  state.enter(input, output);
  state.run(input, output);
  state.run(input, output);
  state.run(input, output);

  if (!output.isFinite()) {
    std::cerr << "beyond mimic2 output not finite\n";
    return 1;
  }
  std::cout << "beyond_mimic2_state_test_pass=1\n";
  return 0;
}
```

- [ ] **Step 2: Run the test to verify RED**

Register and run:

```bash
cd /home/abc/RoboMimic_Deploy/deploy_real/deploycpp
cmake --build build -j4 --target beyond_mimic2_state_test
ctest --test-dir build -R beyond_mimic2_state_test --output-on-failure
```

Expected: FAIL because `BeyondMimic2State` does not exist yet.

- [x] **Step 3: Add the failing FSM transition test**

Create `tests/FSMControllerTest.cpp`:

```cpp
#include <iostream>

#include "FSMController.h"
#include "config/Config.h"

int main() {
  const Config config = loadConfig("/home/abc/RoboMimic_Deploy");
  FSMController fsm(config, true);
  StateAndCmd state;
  PolicyOutput output;

  fsm.run(state, output);
  state.remote.start = true;
  fsm.run(state, output);
  state.remote.start = false;

  for (int i = 0; i < fsm.fixedPoseState().totalSteps(); ++i) {
    fsm.run(state, output);
  }

  state.remote.R1 = true;
  state.remote.A = true;
  fsm.run(state, output);
  if (fsm.currentState() != FSMStateName::LocoMode) return 1;

  state.remote.A = false;
  state.remote.B = true;
  fsm.run(state, output);
  if (fsm.currentState() != FSMStateName::BeyondMimic2) return 1;

  state.remote.R1 = false;
  state.remote.B = false;
  state.remote.F1 = true;
  fsm.run(state, output);
  if (fsm.currentState() != FSMStateName::Passive) return 1;

  std::cout << "fsm_controller_test_pass=1\n";
  return 0;
}
```

Register and run:

```bash
cd /home/abc/RoboMimic_Deploy/deploy_real/deploycpp
cmake --build build -j4 --target fsm_controller_test
ctest --test-dir build -R fsm_controller_test --output-on-failure
```

Expected: FAIL because `BeyondMimic2State` and the new transitions are not implemented yet.

- [x] **Step 4: Implement `BeyondMimic2State` and transition rules**

Create `src/states/BeyondMimic2State.h/.cpp` by mirroring the current `BeyondMimicState` structure, but loading `config.beyond_mimic2_yaml_path` and `config.beyond_mimic2_model_path`.

The constructor should match this shape:

```cpp
BeyondMimic2State::BeyondMimic2State(const Config& config)
    : runner_(config.beyond_mimic2_model_path) {
  loadYaml(config.beyond_mimic2_yaml_path);
}
```

Set transitions:

```cpp
if (state.remote.F1) return FSMStateName::Passive;
if (state.remote.start) return FSMStateName::FixedPose;
if (state.remote.R1 && state.remote.A) return FSMStateName::LocoMode;
return FSMStateName::BeyondMimic2;
```

Update `src/states/FixedPoseState.cpp`:

```cpp
if (state.remote.F1) {
  return FSMStateName::Passive;
}
if (state.remote.R1 && state.remote.A && complete()) {
  return FSMStateName::LocoMode;
}
if (state.remote.L1 && state.remote.Y && complete()) {
  return FSMStateName::BeyondMimic;
}
if (state.remote.R1 && state.remote.B && complete()) {
  return FSMStateName::BeyondMimic2;
}
```

Update `src/states/BeyondMimicState.cpp` to remove `B -> Passive` and add `R1 + A -> LocoMode`:

```cpp
if (state.remote.F1) return FSMStateName::Passive;
if (state.remote.start) return FSMStateName::FixedPose;
if (state.remote.R1 && state.remote.A) return FSMStateName::LocoMode;
return FSMStateName::BeyondMimic;
```

Update `src/FSMController.h/.cpp` to hold and dispatch all five states.

- [x] **Step 5: Expand CLI state names and `--check-fsm` coverage**

Update `stateName()` in `src/main.cpp`:

```cpp
case FSMStateName::LocoMode:
  return "LocoMode";
case FSMStateName::BeyondMimic2:
  return "BeyondMimic2";
```

Extend `checkFsm()` so it verifies:

```cpp
FixedPose -> LocoMode
LocoMode -> BeyondMimic
LocoMode -> BeyondMimic2
BeyondMimic2 -> Passive via F1
```

Also update `printUsage()`:

```cpp
std::cerr << "  " << program << " --check-torch /home/abc/RoboMimic_Deploy\n";
```

- [x] **Step 6: Re-run all C++ tests**

Run:

```bash
cd /home/abc/RoboMimic_Deploy/deploy_real/deploycpp
cmake --build build -j4
ctest --test-dir build --output-on-failure
./build/robomimic_deploycpp --check-fsm /home/abc/RoboMimic_Deploy
./build/robomimic_deploycpp --check-onnx /home/abc/RoboMimic_Deploy
./build/robomimic_deploycpp --check-torch /home/abc/RoboMimic_Deploy
```

Expected:

- all `ctest` entries pass
- `--check-fsm` reports the new transition coverage
- `--check-onnx` still succeeds
- `--check-torch` succeeds

- [ ] **Step 7: Commit**

```bash
git -C /home/abc/RoboMimic_Deploy add \
  deploy_real/deploycpp/src/states/BeyondMimic2State.h \
  deploy_real/deploycpp/src/states/BeyondMimic2State.cpp \
  deploy_real/deploycpp/src/FSMController.h \
  deploy_real/deploycpp/src/FSMController.cpp \
  deploy_real/deploycpp/src/states/FixedPoseState.cpp \
  deploy_real/deploycpp/src/states/BeyondMimicState.cpp \
  deploy_real/deploycpp/src/main.cpp \
  deploy_real/deploycpp/tests/FSMControllerTest.cpp
git -C /home/abc/RoboMimic_Deploy commit -m "feat: add beyond mimic2 state and complete fsm transitions"
```

### Task 5: Controller Logging and Local Real-Robot Verification

**Files:**
- Modify: `deploy_real/deploycpp/src/Controller.cpp`

- [x] **Step 1: Add transition logging for the new states**

Update `src/Controller.cpp` so the transition logs include:

```cpp
transition=FixedPose->LocoMode
transition=LocoMode->BeyondMimic
transition=LocoMode->BeyondMimic2
transition=BeyondMimic->LocoMode
transition=BeyondMimic2->LocoMode
transition=...->Passive
```

And remove any `B`-driven passive log path.

- [x] **Step 2: Build and verify the final binary**

Run:

```bash
cd /home/abc/RoboMimic_Deploy/deploy_real/deploycpp
cmake --build build -j4
ctest --test-dir build --output-on-failure
./build/robomimic_deploycpp --check-config /home/abc/RoboMimic_Deploy
./build/robomimic_deploycpp --check-onnx /home/abc/RoboMimic_Deploy
./build/robomimic_deploycpp --check-torch /home/abc/RoboMimic_Deploy
./build/robomimic_deploycpp --check-fsm /home/abc/RoboMimic_Deploy
```

Expected:

- build succeeds
- all ctests pass
- config output includes `loco_model=` and `beyond_mimic2_model=`
- both model checks succeed

- [x] **Step 3: Local robot smoke test**

Run on the local PC connected to the robot:

```bash
cd /home/abc/RoboMimic_Deploy/deploy_real/deploycpp
./build/robomimic_deploycpp --probe-lowstate /home/abc/RoboMimic_Deploy
./build/robomimic_deploycpp --run /home/abc/RoboMimic_Deploy
```

Verify this manual path:

```text
Start -> FixedPose
R1 + A -> LocoMode
L1 + Y -> BeyondMimic
F1 -> Passive
Start -> FixedPose
R1 + A -> LocoMode
R1 + B -> BeyondMimic2
F1 -> Passive
Select -> Exit
```

- [ ] **Step 4: Commit**

```bash
git -C /home/abc/RoboMimic_Deploy add deploy_real/deploycpp/src/Controller.cpp
git -C /home/abc/RoboMimic_Deploy commit -m "feat: add deploycpp transition logging and verification flow"
```
