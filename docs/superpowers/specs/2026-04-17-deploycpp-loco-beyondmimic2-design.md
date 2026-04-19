# DeployCpp LocoMode + BeyondMimic2 Design

## Goal

在现有 `RoboMimic_Deploy/deploy_real/deploycpp` C++ 实机部署程序中新增 `LocoMode` 与 `BeyondMimic2` 两个状态，并保持当前已经跑通的 `Passive -> FixedPose -> BeyondMimic` 链路不回退。

本次设计明确采用混合推理运行时：

- `LocoMode` 使用 `LibTorch` 加载 `.pt`
- `BeyondMimic` 与 `BeyondMimic2` 继续使用 `ONNX Runtime` 加载 `.onnx`

不尝试把所有策略统一成一种模型格式，也不在本次范围内接入 `AnyAdapter`、`Dance`、`SkillCooldown` 等其他状态。

## Scope

本次只处理本地电脑上的 `deploycpp` C++ 程序，不处理：

- Python 部署逻辑改写
- 板载 Orin NX 文档扩展
- 其他技能状态
- 模型重新导出或训练

目标是让本地 C++ 版本具备以下稳定状态流转：

```text
Passive
  -> FixedPose
  -> LocoMode
      -> BeyondMimic
      -> BeyondMimic2
      -> Passive
      -> FixedPose
```

并且维持技能态也可退出回：

- `LocoMode`
- `FixedPose`
- `Passive`

## Architecture

### 1. 保持现有 ONNX 路线不动

当前 `BeyondMimic` 的 C++ 版本已经验证可用，因此保留：

- `OnnxPolicyRunner`
- `BeyondMimicState`
- `Config` 中现有 `BeyondMimic.yaml` / `.onnx` 读取方式

`BeyondMimic2` 将复用同一套 ONNX 推理基础设施，只更换：

- YAML 路径
- 模型路径
- 状态名
- 遥控切换规则

### 2. 只给 LocoMode 引入 LibTorch

`LocoMode` 当前模型资产是 `policy/loco_mode/model/policy_29dof.pt`，其 Python 版部署逻辑也是 `torch.jit.load(...)`。因此 C++ 侧新增一个独立的 `TorchPolicyRunner`，仅供 `LocoModeState` 使用。

这意味着最终二进制会同时链接：

- `unitree_sdk2`
- `ddsc/ddscxx`
- `yaml-cpp`
- `libonnxruntime.so`
- `LibTorch`

但状态层边界保持清晰：

- `LocoModeState` 不依赖 ONNX
- `BeyondMimicState` / `BeyondMimic2State` 不依赖 LibTorch 细节

### 3. 扩展 FSM 到五态

当前 C++ FSM 只有：

- `Passive`
- `FixedPose`
- `BeyondMimic`

本次扩为：

- `Passive`
- `FixedPose`
- `LocoMode`
- `BeyondMimic`
- `BeyondMimic2`

状态切换语义对齐当前 Python `deploy_real.py` 的按键约定，但只覆盖本次需要的三个运行态：

- `Start` -> `FixedPose`
- `R1 + A` -> `LocoMode`
- `L1 + Y` -> `BeyondMimic`
- `R1 + B` -> `BeyondMimic2`
- `F1` -> `Passive`
- `Select` -> 退出程序
- 取消 `B` 单键退出，避免与 `R1 + B` 的 `BeyondMimic2` 触发冲突

## Components

### 1. `TorchPolicyRunner`

新增一个轻量 C++ 运行时包装，负责：

- 加载 TorchScript `.pt`
- 验证模型可以前向执行
- 接收 `(1, num_obs)` 的 float 输入
- 输出 `(1, num_actions)` 的 float 动作

该类不负责状态机逻辑，不负责遥控解释，也不负责关节重排。

### 2. `LocoModeState`

新增 `LocoModeState`，对齐 Python `policy/loco_mode/LocoMode.py` 的核心逻辑：

- 从 `LocoMode.yaml` 读取：
  - `policy_path`
  - `kps`
  - `kds`
  - `default_angles`
  - `joint2motor_idx`
  - `cmd_scale`
  - `cmd_range`
  - `ang_vel_scale`
  - `dof_pos_scale`
  - `dof_vel_scale`
  - `action_scale`
  - `num_obs`
  - `num_actions`
- 构造 96 维观测：
  - 角速度
  - 重力方向
  - 遥控速度命令
  - 关节位置
  - 关节速度
  - 上一帧动作
- 调用 `TorchPolicyRunner`
- 动作缩放并按 `joint2motor_idx` 回写到 29 维 motor 顺序
- 同时输出对应的 `kp/kd`

### 3. `BeyondMimic2State`

新增 `BeyondMimic2State`，实现方式尽量贴近 `BeyondMimicState`：

- 使用 `policy/beyond_mimic2/config/BeyondMimic2.yaml`
- 使用 `policy/beyond_mimic2/model/*.onnx`
- 观测构造、anchor 初始化、输出映射基本复用 `BeyondMimic` 的 tracking 逻辑

两者主要差异是配置与状态切换语义，本次不引入新的通用技能态基类。实现上采用直接新增独立状态类的方式；只有在共享逻辑已经完全一致时，才提取为局部 helper 函数。

### 4. `StateAndCmd` / `RobotIO`

当前 C++ `StateAndCmd` 没有 `vel_cmd`，而 `LocoMode` 必须读取遥控速度命令。因此需要：

- 在 `StateAndCmd` 中新增 `vel_cmd[3]`
- 在 `RobotIO` 中把遥控器输入映射为：
  - `vel_cmd[0] = ly`
  - `vel_cmd[1] = -lx`
  - `vel_cmd[2] = -rx`

同时保留现有按钮解析。

### 5. `Controller` / `main.cpp`

需要同步扩展：

- 转场打印
- `--check-fsm` 的离线验证路径
- 新增 `--check-torch`
- 在 `--check-config` 中补充 `loco_model=...`
- 在 `probe-lowstate` 输出中补充 `A/L1`

## Data Flow

### LocoMode

1. 程序从 `FixedPose` 收到 `R1 + A`
2. FSM 切到 `LocoMode`
3. `RobotIO` 提供关节状态、IMU、遥控摇杆输入
4. `LocoModeState` 构造 96 维观测
5. `TorchPolicyRunner` 执行 `.pt` 前向
6. 动作经 `action_scale + default_angles` 后映射回 motor 顺序
7. 输出 `actions/kps/kds`

### BeyondMimic2

1. 程序在 `LocoMode` 下收到 `R1 + B`
2. FSM 切到 `BeyondMimic2`
3. 前两帧建立 yaw anchor
4. 第三帧开始构造 154 维 tracking 观测
5. `OnnxPolicyRunner` 执行 `.onnx` 前向
6. 动作按 `mj2lab` 写回 motor 顺序
7. 输出 `actions/kps/kds`

## State Transition Rules

### Passive

- `Start` -> `FixedPose`
- 其他输入保持 `Passive`

### FixedPose

- 完成前禁止进入任意技能态
- 完成后：
  - `R1 + A` -> `LocoMode`
  - `L1 + Y` -> `BeyondMimic`
  - `R1 + B` -> `BeyondMimic2`
  - `F1` -> `Passive`

### LocoMode

- `L1 + Y` -> `BeyondMimic`
- `R1 + B` -> `BeyondMimic2`
- `Start` -> `FixedPose`
- `F1` -> `Passive`
- 否则保持 `LocoMode`

### BeyondMimic

- `R1 + A` -> `LocoMode`
- `Start` -> `FixedPose`
- `F1` -> `Passive`
- 否则保持 `BeyondMimic`

### BeyondMimic2

- `R1 + A` -> `LocoMode`
- `Start` -> `FixedPose`
- `F1` -> `Passive`
- 否则保持 `BeyondMimic2`

## Error Handling

- 缺少 `LocoMode.yaml`、`BeyondMimic2.yaml`、`.pt`、`.onnx` 文件时，初始化直接报错
- LibTorch 模型无法加载或前向 shape 不匹配时，启动时报清晰异常
- `BeyondMimic2` ONNX 输入输出名与当前 runner 约定不一致时，启动时报错
- 观测长度与配置不一致时，状态 `run()` 中直接抛异常，避免静默发错命令
- 任意状态输出非有限值时，沿用控制层现有保护，切回阻尼输出

## Testing

本次实现必须新增最小但有效的本地检查，覆盖以下风险：

### 1. 构建与依赖检查

- CMake 能同时找到 `ONNX Runtime` 和 `LibTorch`
- 新增 `--check-torch`，验证：
  - `.pt` 文件能加载
  - 零输入可前向
  - 输出尺寸正确

### 2. FSM 离线检查

扩展 `--check-fsm`，验证：

- `Passive -> FixedPose`
- `FixedPose` 完成前不能进 `LocoMode`
- `FixedPose` 完成后可进 `LocoMode`
- `LocoMode -> BeyondMimic`
- `LocoMode -> BeyondMimic2`
- `BeyondMimic -> LocoMode`
- `BeyondMimic2 -> LocoMode`
- 各运行态都可回 `Passive`

### 3. 状态局部检查

- `LocoModeState` 零输入时能给出 29 维动作与完整 `kp/kd`
- `BeyondMimic2State` 的前三帧行为与 `BeyondMimicState` 一致：
  - 前两帧 anchor
  - 第三帧开始真正输出策略动作

### 4. 本地实机回归

在本地电脑连机器人时，按顺序验证：

1. `--probe-lowstate`
2. `--check-config`
3. `--check-onnx`
4. `--check-torch`
5. `--check-fsm`
6. `--run`

重点确认：

- `FixedPose` 姿态不退化
- `LocoMode` 能稳定进入并响应摇杆
- `BeyondMimic` 仍保持已验证行为
- `BeyondMimic2` 能正常进入、执行、退出

## File Impact

会触达这些 C++ 文件：

- `deploy_real/deploycpp/CMakeLists.txt`
- `deploy_real/deploycpp/src/types/StateAndCmd.h`
- `deploy_real/deploycpp/src/RobotIO.cpp`
- `deploy_real/deploycpp/src/config/Config.h`
- `deploy_real/deploycpp/src/config/Config.cpp`
- `deploy_real/deploycpp/src/FSMController.h`
- `deploy_real/deploycpp/src/FSMController.cpp`
- `deploy_real/deploycpp/src/Controller.cpp`
- `deploy_real/deploycpp/src/main.cpp`
- `deploy_real/deploycpp/src/states/FSMState.h`

会新增这些文件：

- `deploy_real/deploycpp/src/policy/TorchPolicyRunner.h`
- `deploy_real/deploycpp/src/policy/TorchPolicyRunner.cpp`
- `deploy_real/deploycpp/src/states/LocoModeState.h`
- `deploy_real/deploycpp/src/states/LocoModeState.cpp`
- `deploy_real/deploycpp/src/states/BeyondMimic2State.h`
- `deploy_real/deploycpp/src/states/BeyondMimic2State.cpp`

## Non-Goals

- 不把 `LocoMode` 转成 ONNX
- 不把 `BeyondMimic` / `BeyondMimic2` 改成 LibTorch
- 不为所有技能状态建立通用大一统基类
- 不在本次接入 Python 版全部 FSM 状态
- 不在本次修改板载 README
