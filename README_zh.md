<div align="center">
  <h1 align="center">RoboMimic Deploy</h1>
  <p align="center">
    <a href="README.md">English</a> | <span>中文</span>
  </p>
</div>

<p align="center">
  <strong>面向 Unitree G1（29 自由度）Orin NX 板载环境的精简 C++ 部署工作区。</strong>
</p>

## 项目定位

这个仓库现在已经收敛成 `deploy_real/deploycpp` 这条 C++ 板载部署主线。

当前运行态只有这 5 个：

- `Passive`
- `FixedPose`
- `LocoMode`
- `BeyondMimic`
- `BeyondMimic2`

模型后端：

- `LocoMode` -> LibTorch / TorchScript
- `BeyondMimic` -> ONNX Runtime
- `BeyondMimic2` -> ONNX Runtime

目标平台是带 3 自由度腰部的 Unitree G1，运行环境是机载 Orin NX（`aarch64`）。

## 仓库结构

| 路径 | 作用 |
| --- | --- |
| `deploy_real/deploycpp/` | 主 C++ 板载部署程序 |
| `deploy_real/config/real.yaml` | deploycpp 运行时 DDS / 控制周期配置 |
| `policy/passive/` | Passive 状态配置 |
| `policy/fixedpose/` | FixedPose 状态配置 |
| `policy/loco_mode/` | 行走策略配置和 TorchScript 模型 |
| `policy/beyond_mimic/` | BeyondMimic 的 ONNX 配置和模型 |
| `policy/beyond_mimic2/` | BeyondMimic2 的 ONNX 配置和模型 |
| `g1_description/` | 仍保留在工作区里的机器人描述资产 |

## 状态机与按键

### Runner 状态机

当前板载 runner 走的是这条链路：

```text
Passive -> FixedPose -> LocoMode / BeyondMimic / BeyondMimic2
```

当前 C++ 实现支持的切换规则：

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
- runner 运行中：
  - `Select` -> 发送阻尼并退出

### Launcher 流程

仓库里还包含一层受保护的 launcher 启动逻辑：

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

当前代码里的 launcher 参数：

- arm 超时：5 秒
- 快速失败阈值：2 秒
- 失败统计窗口：60 秒
- 自动重启退避：`1s -> 3s -> 10s`

## 编译

在机器人上的 `deploy_real/deploycpp` 目录执行：

```bash
cmake -S . -B build_release \
  -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT=/home/unitree/onnxruntime-linux-aarch64-1.19.2 \
  -DLIBTORCH_ROOT=/home/unitree/RoboMimic_Deploy/deploy_real/deploycpp/libtorch
cmake --build build_release -j4
```

输出文件：

```text
build_release/robomimic_deploycpp
```

## 运行

直接运行控制器：

```bash
./build_release/robomimic_deploycpp --run /home/unitree/RoboMimic_Deploy
```

通过 launcher 运行：

```bash
./build_release/robomimic_deploycpp --launcher /home/unitree/RoboMimic_Deploy
```

## systemd 自启服务

仓库中自带：

```text
deploy_real/deploycpp/systemd/robomimic-launcher.service
deploy_real/deploycpp/systemd/install_robomimic_launcher_service.sh
```

默认启动命令是：

```text
/home/unitree/RoboMimic_Deploy/deploy_real/deploycpp/build_release/robomimic_deploycpp --launcher /home/unitree/RoboMimic_Deploy
```

如果你的板载路径不一样，先改 service 文件再安装。

## 自检命令

当前 C++ 可执行程序支持：

```bash
./build_release/robomimic_deploycpp --check-config /home/unitree/RoboMimic_Deploy
./build_release/robomimic_deploycpp --check-onnx /home/unitree/RoboMimic_Deploy
./build_release/robomimic_deploycpp --check-torch /home/unitree/RoboMimic_Deploy
./build_release/robomimic_deploycpp --check-math
./build_release/robomimic_deploycpp --check-fsm /home/unitree/RoboMimic_Deploy
./build_release/robomimic_deploycpp --probe-lowstate /home/unitree/RoboMimic_Deploy
./build_release/robomimic_deploycpp --dump-beyond-mimic-alignment /home/unitree/RoboMimic_Deploy
```

可以拿来检查：

- 运行时配置路径
- ONNX / Torch 模型接口
- 数学工具函数
- FSM 切换逻辑
- DDS `low_state` 连通性
- BeyondMimic 对齐数据导出

## 运行时配置

C++ 链路会读取：

- `deploy_real/config/real.yaml`
- `policy/passive/config/Passive.yaml`
- `policy/fixedpose/config/FixedPose.yaml`
- `policy/loco_mode/config/LocoMode.yaml`
- `policy/beyond_mimic/config/BeyondMimic.yaml`
- `policy/beyond_mimic2/config/BeyondMimic2.yaml`

`real.yaml` 当前默认值：

- `net: eth0`
- `num_joints: 29`
- `lowcmd_topic: rt/lowcmd`
- `lowstate_topic: rt/lowstate`
- `control_dt: 0.02`
- `error_over_time: 5`

当前默认模型文件名：

- `policy/loco_mode/model/policy_29dof.pt`
- `policy/beyond_mimic/model/dance_763.onnx`
- `policy/beyond_mimic2/model/dance_763.onnx`

## 说明

- 这个工作区现在只服务于 C++ 板载部署链路。
- 板载部署时不要复用桌面机 `x86_64` 的构建产物，也不要复用桌面版 ONNX Runtime / LibTorch。
- 请在机器人上使用 `aarch64` 对应依赖进行编译和运行。
- 首次部署前建议先阅读 `deploy_real/deploycpp/README.md` 里的详细板载说明。
