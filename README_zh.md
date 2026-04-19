<div align="center">
  <h1 align="center">RoboMimic Deploy</h1>
  <p align="center">
    <a href="README.md">English</a> | <span>中文</span>
  </p>
</div>

<p align="center">
  <strong>面向 Unitree G1（29 自由度）的部署工作区，包含 Mujoco 仿真、旧版 Python 实机入口，以及当前主线的 Orin NX 板载 C++ 部署链路。</strong>
</p>

## 项目现状

这个仓库现在已经不是单纯的 Python 演示项目了，当前真正的实机主线已经转到 `deploy_real/deploycpp`。

目前可以这样理解这个仓库：

- 当前推荐的实机部署路径是 `deploy_real/deploycpp` 里的 C++ 程序。
- 这套 C++ 部署面向带 3 自由度腰部的 Unitree G1，设计目标是在机载 Orin NX（`aarch64`）上运行。
- 当前 C++ 状态机一共支持 5 个状态：
  - `Passive`
  - `FixedPose`
  - `LocoMode`
  - `BeyondMimic`
  - `BeyondMimic2`
- `LocoMode` 走 LibTorch / TorchScript。
- `BeyondMimic` 和 `BeyondMimic2` 走 ONNX Runtime。
- 仓库里已经包含 `launcher` 模式和 `systemd` 自启服务，适合做板载自动启动。

同时，仓库里也还保留了一些旧内容，但它们已经不是根 README 应该重点强调的主线：

- `deploy_real/deploy_real.py` 仍然保留，作为旧版 Python 实机入口。
- `policy/` 下面的 `dance`、`kungfu`、`kick`、`skill_cast`、`skill_cooldown` 等策略资产还在仓库里，但当前板载 C++ 主链路聚焦的是 `Passive -> FixedPose -> LocoMode / BeyondMimic / BeyondMimic2`。

## 仓库结构

| 路径 | 作用 |
| --- | --- |
| `deploy_mujoco/` | Mujoco 仿真入口 |
| `deploy_real/deploy_real.py` | 旧版 Python 实机入口 |
| `deploy_real/deploycpp/` | 当前 C++ 板载部署程序 |
| `deploy_real/config/real.yaml` | 实机 DDS / 控制周期 / 机器人基础配置 |
| `policy/loco_mode/` | 行走策略配置和 TorchScript 模型 |
| `policy/beyond_mimic/` | BeyondMimic ONNX 策略资产 |
| `policy/beyond_mimic2/` | BeyondMimic2 ONNX 策略资产 |
| `tests/` | 仓库已有的 Python 侧测试与工具 |
| `deploy_real/deploycpp/tests/` | C++ 单测、冒烟测试、FSM / launcher 测试 |
| `deploy_real/deploycpp/systemd/` | 板载自启服务文件 |

## 运行路径

### 1. Mujoco 仿真

不接真机时，先用它验证策略和状态切换：

```bash
python deploy_mujoco/deploy_mujoco.py
```

### 2. 旧版 Python 实机入口

这条路径还在，但已经不是当前项目的主推部署方向：

```bash
python deploy_real/deploy_real.py
```

### 3. 当前 C++ 板载部署

这是现在仓库重点围绕的一条链路：

- 可执行文件：`deploy_real/deploycpp/build_release/robomimic_deploycpp`
- 目标平台：机载 Orin NX
- 依赖：`unitree_sdk2` C++ SDK、ONNX Runtime `aarch64`、LibTorch `aarch64`、`yaml-cpp`、`cmake`、`g++`

更详细的板载部署说明见：

- `deploy_real/deploycpp/README.md`

## 当前 C++ 状态机与按键

### Runner 状态机

当前板载 runner 使用的是这条状态链路：

```text
Passive -> FixedPose -> LocoMode / BeyondMimic / BeyondMimic2
```

当前代码里明确支持的切换规则如下：

- `Passive`
  - `Start` -> `FixedPose`
- `FixedPose`
  - `R1 + A` -> `LocoMode`，前提是 `FixedPose` 已完成
  - `L1 + Y` -> `BeyondMimic`，前提是 `FixedPose` 已完成
  - `R1 + B` -> `BeyondMimic2`，前提是 `FixedPose` 已完成
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
- runner 运行期间任意时刻：
  - `Select` -> 发送阻尼并退出 runner

### Launcher 启动流程

在 runner 之外，当前板载方案还加了一层 `launcher` 守护启动逻辑：

- `Idle`
  - 等待 `low_state`
  - `L2 + R2` -> `Armed`
- `Armed`
  - 在超时窗口内按一次 `X` -> 拉起 runner
- `Running`
  - 监控子进程
  - 失败时按退避策略自动重启
- `Locked`
  - 连续快速失败过多后进入锁定
  - 需要重新 arm

当前代码里的 launcher 参数是：

- arm 超时：5 秒
- 快速失败阈值：2 秒
- 失败统计窗口：60 秒
- 自动重启退避：`1s -> 3s -> 10s`

## 当前 C++ 板载构建与运行

### 编译

在机器人上的 `deploy_real/deploycpp` 目录执行：

```bash
cmake -S . -B build_release \
  -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT=/home/unitree/onnxruntime-linux-aarch64-1.19.2 \
  -DLIBTORCH_ROOT=/home/unitree/RoboMimic_Deploy/deploy_real/deploycpp/libtorch
cmake --build build_release -j4
```

生成的目标文件是：

```text
build_release/robomimic_deploycpp
```

### 直接运行 runner

```bash
./build_release/robomimic_deploycpp --run /home/unitree/RoboMimic_Deploy
```

### 通过 launcher 运行

```bash
./build_release/robomimic_deploycpp --launcher /home/unitree/RoboMimic_Deploy
```

### 安装仓库自带的 `systemd` 自启服务

服务文件：

```text
deploy_real/deploycpp/systemd/robomimic-launcher.service
```

安装脚本：

```text
deploy_real/deploycpp/systemd/install_robomimic_launcher_service.sh
```

当前仓库自带的服务默认执行：

```text
/home/unitree/RoboMimic_Deploy/deploy_real/deploycpp/build_release/robomimic_deploycpp --launcher /home/unitree/RoboMimic_Deploy
```

如果你的板载路径不是这个，先改服务文件再安装。

## 常用检查命令

当前 C++ 可执行程序已经内置了不少自检入口：

```bash
./build_release/robomimic_deploycpp --check-config /home/unitree/RoboMimic_Deploy
./build_release/robomimic_deploycpp --check-onnx /home/unitree/RoboMimic_Deploy
./build_release/robomimic_deploycpp --check-torch /home/unitree/RoboMimic_Deploy
./build_release/robomimic_deploycpp --check-math
./build_release/robomimic_deploycpp --check-fsm /home/unitree/RoboMimic_Deploy
./build_release/robomimic_deploycpp --probe-lowstate /home/unitree/RoboMimic_Deploy
./build_release/robomimic_deploycpp --dump-beyond-mimic-alignment /home/unitree/RoboMimic_Deploy
```

这些命令分别适合拿来检查：

- `real.yaml` 和策略 YAML 是否被正确加载
- ONNX / Torch 模型接口是否匹配
- 数学工具函数是否正常
- FSM 切换逻辑是否符合当前实现
- DDS `low_state` 是否真的通了
- BeyondMimic 对齐数据是否能导出用于调试

## 当前配置与模型

当前 C++ 链路会读取这些配置：

- `deploy_real/config/real.yaml`
- `policy/loco_mode/config/LocoMode.yaml`
- `policy/beyond_mimic/config/BeyondMimic.yaml`
- `policy/beyond_mimic2/config/BeyondMimic2.yaml`

当前仓库里默认配置可以概括为：

- `net: eth0`
- `num_joints: 29`
- `control_dt: 0.02`
- 开启 FK 状态估计，模型路径为 `g1_description/g1_29dof_rev_1_0.xml`

当前默认模型文件名：

- `policy/loco_mode/model/policy_29dof.pt`
- `policy/beyond_mimic/model/dance_763.onnx`
- `policy/beyond_mimic2/model/dance_763.onnx`

## 注意事项

- 这套仓库当前面向的是带 3 自由度腰部的 G1。
- 实机部署前要先确认机器人侧 DDS 通信网卡配置正确。
- 板载部署时不要直接复用桌面机上编出来的 `x86_64` 产物，也不要复用桌面机版本的 ONNX Runtime / LibTorch。
- 最稳妥的流程仍然是：先在 Mujoco 里熟悉流程，再在真机上检查配置、模型和 `low_state`，最后再启动板载控制。

## 下一步阅读

- 英文版总览：`README.md`
- 板载部署详细说明：`deploy_real/deploycpp/README.md`
