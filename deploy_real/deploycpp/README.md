# RoboMimic_Deploy G1 板载 C++ 部署说明

这个目录是 `RoboMimic_Deploy` 的 C++ 实机部署程序。按照今天的实现，`deploycpp` 已经支持 5 个状态：

```text
Passive
FixedPose
LocoMode
BeyondMimic
BeyondMimic2
```

当前策略链路是：

```text
Passive -> FixedPose -> LocoMode -> BeyondMimic / BeyondMimic2
```

并支持回切：

```text
BeyondMimic -> LocoMode
BeyondMimic2 -> LocoMode
任意活动状态 -> Passive
```

策略后端分工如下：

```text
LocoMode      -> LibTorch / TorchScript
BeyondMimic   -> ONNX Runtime
BeyondMimic2  -> ONNX Runtime
```

本文面向机器人内部 Orin NX 板载运行。重点原则只有一条：**Orin NX 是 ARM/aarch64，不能直接复用你本地电脑上的 x86_64 构建产物，也不能直接复用 x86_64 版 ONNX Runtime 或 LibTorch。**

推荐方式：把源码和依赖放到机器人内部，在机器人上原生编译 `Release` 版本。

---

## 1. 板载目录规划

本文默认机器人用户名是：

```text
unitree
```

默认板载项目路径：

```text
/home/unitree/RoboMimic_Deploy
```

默认板载 `LibTorch` 路径：

```text
/home/unitree/RoboMimic_Deploy/deploy_real/deploycpp/libtorch
```

默认板载 `ONNX Runtime` 路径示例：

```text
/home/unitree/onnxruntime-linux-aarch64-1.19.2
```

如果你最后放在别的位置，也可以，只要后面的 `PROJECT_ROOT`、`ONNXRUNTIME_ROOT`、`LIBTORCH_ROOT` 环境变量改对即可。

本文同时默认你当前在外部电脑上准备代码的本地目录是：

```text
/home/abc/robot/RoboMimic_Deployc/RoboMimic_Deploy
```

也就是说：

- 本地准备目录：`/home/abc/robot/RoboMimic_Deployc/RoboMimic_Deploy`
- 机器人板载落地目录：`/home/unitree/RoboMimic_Deploy`

---

## 2. 需要传到机器人里的内容

### 必须传

建议直接传整个项目目录：

```text
/home/abc/robot/RoboMimic_Deployc/RoboMimic_Deploy
```

至少要包含这些路径：

```text
RoboMimic_Deploy/deploy_real/deploycpp
RoboMimic_Deploy/deploy_real/config/real.yaml
RoboMimic_Deploy/policy/passive/config/Passive.yaml
RoboMimic_Deploy/policy/fixedpose/config/FixedPose.yaml
RoboMimic_Deploy/policy/loco_mode/config/LocoMode.yaml
RoboMimic_Deploy/policy/loco_mode/model/*.pt
RoboMimic_Deploy/policy/beyond_mimic/config/BeyondMimic.yaml
RoboMimic_Deploy/policy/beyond_mimic/model/*.onnx
RoboMimic_Deploy/policy/beyond_mimic2/config/BeyondMimic2.yaml
RoboMimic_Deploy/policy/beyond_mimic2/model/*.onnx
```

### 板载还需要准备的依赖

机器人上还需要：

```text
unitree_sdk2 C++ SDK，且是 ARM/aarch64 可用版本
ONNX Runtime C/C++ aarch64 release 包
LibTorch aarch64 release 包
yaml-cpp
cmake
g++
```

### 不要直接传本地 x86_64 产物

这些不能直接给 Orin NX 用：

```text
RoboMimic_Deploy/deploy_real/deploycpp/build
RoboMimic_Deploy/deploy_real/deploycpp/build_release
/home/abc/onnxruntime-linux-x64-1.19.2
任意本地电脑上的 x86_64 libtorch
```

---

## 3. 从本地电脑打包并传到机器人

在本地电脑执行：

```bash
cd /home/abc/robot/RoboMimic_Deployc
tar \
  --exclude='RoboMimic_Deploy/deploy_real/deploycpp/build' \
  --exclude='RoboMimic_Deploy/deploy_real/deploycpp/build_release' \
  -czf RoboMimic_Deploy_deploycpp.tar.gz \
  RoboMimic_Deploy
```

传到机器人：

```bash
scp /home/abc/robot/RoboMimic_Deployc/RoboMimic_Deploy_deploycpp.tar.gz unitree@ROBOT_IP:/home/unitree/
```

在机器人上解压：

```bash
cd /home/unitree
tar -xzf RoboMimic_Deploy.tar.gz
```

---

## 4. 在机器人上确认架构

在机器人 Orin NX 上执行：

```bash
uname -m
```

期望输出：

```text
aarch64
```

如果输出是 `x86_64`，说明你还在外部电脑上，不是在机器人内部。

---

## 5. 在机器人上准备 ONNX Runtime aarch64

如果机器人能联网，可以在机器人上下载：

```bash
cd /home/unitree
wget https://github.com/microsoft/onnxruntime/releases/download/v1.19.2/onnxruntime-linux-aarch64-1.19.2.tgz
tar -xzf onnxruntime-linux-aarch64-1.19.2.tgz
```

设置路径：

```bash
export ONNXRUNTIME_ROOT=/home/unitree/onnxruntime-linux-aarch64-1.19.2
```

检查：

```bash
test -f "$ONNXRUNTIME_ROOT/include/onnxruntime_cxx_api.h" && echo CXX_HEADER_OK
test -f "$ONNXRUNTIME_ROOT/lib/libonnxruntime.so" && echo LIB_OK
```

期望输出：

```text
CXX_HEADER_OK
LIB_OK
```

如果没有这两个输出，不要继续编译。

---

## 6. 在机器人上准备 LibTorch aarch64

你已经明确说过，板载 `LibTorch` 会放在：

```text
/home/unitree/RoboMimic_Deploy/deploy_real/deploycpp/libtorch
```

所以本文后面统一用：

```bash
export LIBTORCH_ROOT=/home/unitree/RoboMimic_Deploy/deploy_real/deploycpp/libtorch
```

这个目录应该是 `libtorch` 解压后的根目录，里面至少要有：

```text
lib/
include/
share/cmake/Torch/TorchConfig.cmake
```

检查命令：

```bash
test -f "$LIBTORCH_ROOT/share/cmake/Torch/TorchConfig.cmake" && echo TORCH_CONFIG_OK
test -d "$LIBTORCH_ROOT/lib" && echo TORCH_LIB_DIR_OK
test -d "$LIBTORCH_ROOT/include" && echo TORCH_INCLUDE_DIR_OK
```

期望输出：

```text
TORCH_CONFIG_OK
TORCH_LIB_DIR_OK
TORCH_INCLUDE_DIR_OK
```

如果你手里拿到的是压缩包，比如 `libtorch-cxx11-abi-shared-with-deps-*.zip` 或 `*.tar.gz`，要先在机器人上解压到这个目录。

---

## 7. 在机器人上准备 unitree_sdk2

机器人上必须能找到这些头文件和库：

```text
unitree/idl/hg/LowCmd_.hpp
unitree/idl/hg/LowState_.hpp
unitree/robot/channel/channel_publisher.hpp
unitree/robot/channel/channel_subscriber.hpp
libunitree_sdk2.a 或 libunitree_sdk2.so
libddsc.so
libddscxx.so
```

检查：

```bash
find /usr/local/include /usr/include -path '*unitree/idl/hg/LowCmd_.hpp' 2>/dev/null
find /usr/local/lib /usr/lib -name 'libunitree_sdk2*' -o -name 'libddsc.so*' -o -name 'libddscxx.so*' 2>/dev/null
```

如果找不到，先把 `unitree_sdk2` 安装好，再继续。

---

## 8. 安装基础编译依赖

在机器人上执行：

```bash
sudo apt update
sudo apt install -y build-essential cmake libyaml-cpp-dev
```

检查：

```bash
cmake --version
g++ --version
pkg-config --modversion yaml-cpp
```

---

## 9. 在机器人上重新编译 deploycpp

设置环境变量：

```bash
export PROJECT_ROOT=/home/unitree/RoboMimic_Deploy
export ONNXRUNTIME_ROOT=/home/unitree/onnxruntime-linux-aarch64-1.19.2
export LIBTORCH_ROOT=/home/unitree/RoboMimic_Deploy/deploy_real/deploycpp/libtorch
```

建议使用 `Release` 构建，实机实时性会比默认 `Debug` 更合适：

```bash
cd "$PROJECT_ROOT/deploy_real/deploycpp"
rm -rf build_release
cmake -S . -B build_release \
  -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT="$ONNXRUNTIME_ROOT" \
  -DLIBTORCH_ROOT="$LIBTORCH_ROOT"
cmake --build build_release -j4
```

编译成功后会生成：

```text
build_release/robomimic_deploycpp
```

如果你计划直接启用板载后台自启动，这一步完成后还会用到仓库内的：

```text
deploy_real/deploycpp/systemd/robomimic-launcher.service
deploy_real/deploycpp/systemd/install_robomimic_launcher_service.sh
```

---

## 10. 配置机器人板载运行网卡

程序读取：

```text
$PROJECT_ROOT/deploy_real/config/real.yaml
```

当前本地电脑连机器人时，这里可能是：

```yaml
net: enp88s0
```

这是外部电脑的网卡名，不一定适用于机器人内部 Orin NX。

在机器人上查看网卡：

```bash
ip link
```

然后编辑：

```bash
nano "$PROJECT_ROOT/deploy_real/config/real.yaml"
```

把 `net:` 改成机器人内部用于 Unitree DDS 通信的实际网卡，例如：

```yaml
net: eth0
```

具体名字以机器人上 `ip link` 输出为准。

---

## 11. 当前按键和状态机

当前状态迁移规则如下。

### 从 Passive

```text
Start -> FixedPose
Select -> 退出程序
```

### 从 FixedPose

只有在 `FixedPose` 完成后才允许切策略：

```text
R1 + A -> LocoMode
L1 + Y -> BeyondMimic
R1 + B -> BeyondMimic2
F1     -> Passive
```

### 从 LocoMode

```text
L1 + Y -> BeyondMimic
R1 + B -> BeyondMimic2
Start  -> FixedPose
F1     -> Passive
```

### 从 BeyondMimic

```text
R1 + A -> LocoMode
Start  -> FixedPose
F1     -> Passive
```

### 从 BeyondMimic2

```text
R1 + A -> LocoMode
Start  -> FixedPose
F1     -> Passive
```

---

## 12. 板载检查流程

所有命令都在机器人上执行：

```bash
cd "$PROJECT_ROOT/deploy_real/deploycpp"
```

### 12.1 检查配置

```bash
./build_release/robomimic_deploycpp --check-config "$PROJECT_ROOT"
```

期望看到：

```text
net=你的机器人板载网卡名
num_joints=29
lowcmd_topic=rt/lowcmd
lowstate_topic=rt/lowstate
control_dt=0.02
loco_model=.../policy/loco_mode/model/你的模型.pt
beyond_mimic2_model=.../policy/beyond_mimic2/model/你的模型.onnx
```

重点确认：

```text
net=
loco_model=
beyond_mimic2_model=
```

### 12.2 检查 BeyondMimic 的 ONNX 模型接口

```bash
./build_release/robomimic_deploycpp --check-onnx "$PROJECT_ROOT"
```

期望输出：

```text
model=.../policy/beyond_mimic/model/你的模型.onnx
inputs=obs,time_step
outputs=actions,joint_pos,joint_vel,body_pos_w,body_quat_w,body_lin_vel_w,body_ang_vel_w
actions_size=29
joint_pos_size=29
joint_vel_size=29
body_quat_w_size=56
```

### 12.3 检查 LocoMode 的 LibTorch 模型接口

```bash
./build_release/robomimic_deploycpp --check-torch "$PROJECT_ROOT"
```

期望输出：

```text
model=.../policy/loco_mode/model/你的模型.pt
actions_size=29
```

### 12.4 检查数学函数

```bash
./build_release/robomimic_deploycpp --check-math
```

期望输出：

```text
quat_identity_ok=1
yaw_quat_ok=1
matrix_identity_ok=1
```

### 12.5 检查离线 FSM

```bash
./build_release/robomimic_deploycpp --check-fsm "$PROJECT_ROOT"
```

期望输出包含：

```text
initial_state=Passive
start_to_fixed_pose=1
early_loco_blocked=1
fixed_pose_complete_after_steps=100
fixed_pose_to_loco=1
fixed_pose_to_beyond_mimic=1
fixed_pose_to_beyond_mimic2=1
beyond_mimic_to_loco=1
beyond_mimic_to_fixed_pose=1
beyond_mimic_to_passive=1
loco_to_beyond_mimic2=1
beyond_mimic2_to_passive=1
```

### 12.6 检查 low_state 通信

```bash
./build_release/robomimic_deploycpp --probe-lowstate "$PROJECT_ROOT"
```

期望输出：

```text
low_state_connected=1
num_joints=29
remote_decode_ok=1
q0=...
dq0=...
quat=...
gyro=...
buttons=...
```

如果 5 秒内没有 `low_state_connected=1`，优先检查：

```text
real.yaml 里的 net 是否正确
unitree_sdk2 / DDS 环境是否正常
机器人是否处于可通信状态
```

---

## 13. 板载运行

首次板载运行仍然建议吊起机器人。

```bash
cd "$PROJECT_ROOT/deploy_real/deploycpp"
./build_release/robomimic_deploycpp --run "$PROJECT_ROOT"
```

启动后应该看到：

```text
state=Passive
```

---

## 14. 板载后台 launcher 与 systemd 自启动

如果你不想每次都手动执行：

```bash
./build_release/robomimic_deploycpp --run "$PROJECT_ROOT"
```

建议直接在机器人上启用后台 `launcher` 服务。

### 14.1 服务文件路径

仓库内已经提供板载版 `systemd` 服务文件：

```text
$PROJECT_ROOT/deploy_real/deploycpp/systemd/robomimic-launcher.service
```

其中关键路径已经按机器人板载目录写死为：

```text
User=unitree
WorkingDirectory=/home/unitree/RoboMimic_Deploy/deploy_real/deploycpp
ExecStart=/home/unitree/RoboMimic_Deploy/deploy_real/deploycpp/build_release/robomimic_deploycpp --launcher /home/unitree/RoboMimic_Deploy
```

如果你最后没有把项目放在 `/home/unitree/RoboMimic_Deploy`，需要先改这个服务文件再安装。

### 14.2 安装方式

在机器人上执行：

```bash
cd "$PROJECT_ROOT/deploy_real/deploycpp/systemd"
chmod +x install_robomimic_launcher_service.sh
./install_robomimic_launcher_service.sh
```

安装脚本会完成：

- 拷贝 `robomimic-launcher.service` 到 `/etc/systemd/system/`
- `daemon-reload`
- `enable`
- `restart`
- 打印当前服务状态

### 14.3 查看服务状态和日志

看服务状态：

```bash
sudo systemctl status robomimic-launcher.service --no-pager
```

实时看日志：

```bash
journalctl -u robomimic-launcher.service -f
```

### 14.4 启用自启动后的使用流程

`launcher` 服务起来后，后台会常驻等待，不直接发送运动命令。

推荐按键流程：

1. 机器人按原厂流程开机并进入可控状态
2. 后台 `launcher` 已启动并收到 `low_state`
3. 按一次 `L2 + R2` 完成武装
4. 在武装窗口内按一次 `X`
5. `runner` 被拉起，并进入：

```text
state=Passive
```

6. 再按 `Start` 进入 `FixedPose`
7. 按原有策略键切换到 `LocoMode / BeyondMimic / BeyondMimic2`
8. 按 `Select` 正常退出 `runner`，后台 `launcher` 继续待机

如果 `runner` 连续异常退出，`launcher` 会按有限重启策略退避；超过阈值后进入锁定，需要重新按键确认。

---

## 15. 推荐真机验证路径

这是今天这版实现对应的完整手动验证路径。

### 第一段：验证 LocoMode 和 BeyondMimic

1. 按 `Start`

期望输出：

```text
transition=Passive->FixedPose
```

2. 等待 `FixedPose` 完成

期望输出：

```text
fixed_pose_step=100/100
fixed_pose_complete=1
```

3. 按 `R1 + A`

期望输出：

```text
transition=FixedPose->LocoMode
```

4. 在 `LocoMode` 下推动左摇杆/右摇杆，确认机器人有速度响应

如果机器性能不够，可能会看到：

```text
control_loop_overrun=1
```

这表示当前运行机器没有在 `20ms` 内跑完整个控制回路，优先用机器人板载 `Release` 构建确认最终效果。

5. 按 `L1 + Y`

期望输出：

```text
transition=LocoMode->BeyondMimic
beyond_mimic_anchor_frame=1
beyond_mimic_anchor_frame=2
beyond_mimic_policy_frame=3
```

6. 按 `F1`

期望输出：

```text
transition=BeyondMimic->Passive
```

### 第二段：验证 LocoMode 和 BeyondMimic2

1. 再按一次 `Start`

期望输出：

```text
transition=Passive->FixedPose
```

2. 等待 `FixedPose` 完成

期望输出：

```text
fixed_pose_complete=1
```

3. 按 `R1 + A`

期望输出：

```text
transition=FixedPose->LocoMode
```

4. 按 `R1 + B`

期望输出：

```text
transition=LocoMode->BeyondMimic2
```

5. 按 `F1`

期望输出：

```text
transition=BeyondMimic2->Passive
```

6. 按 `Select`

期望输出：

```text
select_exit=1
damping_sent=1
exit
```

---

## 16. 换模型时要改哪里

### BeyondMimic

修改：

```text
$PROJECT_ROOT/policy/beyond_mimic/config/BeyondMimic.yaml
```

例如：

```yaml
onnx_path: "dance_763.onnx"
motion_length: 28.9
```

模型文件要存在于：

```text
$PROJECT_ROOT/policy/beyond_mimic/model/
```

### BeyondMimic2

修改：

```text
$PROJECT_ROOT/policy/beyond_mimic2/config/BeyondMimic2.yaml
```

例如：

```yaml
onnx_path: "dance_763.onnx"
```

模型文件要存在于：

```text
$PROJECT_ROOT/policy/beyond_mimic2/model/
```

### LocoMode

修改：

```text
$PROJECT_ROOT/policy/loco_mode/config/LocoMode.yaml
```

例如：

```yaml
policy_path: "policy_29dof.pt"
```

模型文件要存在于：

```text
$PROJECT_ROOT/policy/loco_mode/model/
```

换模型后建议重新检查：

```bash
cd "$PROJECT_ROOT/deploy_real/deploycpp"
./build_release/robomimic_deploycpp --check-config "$PROJECT_ROOT"
./build_release/robomimic_deploycpp --check-onnx "$PROJECT_ROOT"
./build_release/robomimic_deploycpp --check-torch "$PROJECT_ROOT"
./build_release/robomimic_deploycpp --check-fsm "$PROJECT_ROOT"
```

---

## 17. 常见问题

### 1. `--check-torch` 报找不到 Torch

优先检查：

```bash
echo "$LIBTORCH_ROOT"
test -f "$LIBTORCH_ROOT/share/cmake/Torch/TorchConfig.cmake" && echo OK
```

### 2. 进入 `LocoMode` 一直打印 `control_loop_overrun=1`

这通常表示当前运行机器没有在 `control_dt=0.02` 的预算内跑完一圈控制。优先确认：

```text
是否在机器人板载上运行
是否使用 Release 构建
LibTorch 是否是正确的 aarch64 版本
```

### 3. `--probe-lowstate` 失败

优先检查：

```text
real.yaml 里的 net 是否正确
机器人 DDS 通信是否正常
unitree_sdk2 是否安装完整
```

### 4. 进不了策略，只能停在 FixedPose

先确认：

```text
fixed_pose_complete=1 是否已经打印
按键组合是否正确
remote_decode_ok 是否为 1
```

### 5. `systemd` 服务启动了，但按键没反应

优先检查：

```text
journalctl -u robomimic-launcher.service -f 是否能看到 low_state_connected=1
real.yaml 里的 net 是否是机器人板载通信网卡
是否先按了 L2+R2，再按 X
服务文件里的 ExecStart 和 WorkingDirectory 是否还是 /home/unitree/RoboMimic_Deploy
```

### 6. `systemd` 服务启动失败

优先检查：

```text
build_release/robomimic_deploycpp 是否已经在机器人上编译成功
robomimic-launcher.service 里的路径是否和实际板载路径一致
sudo systemctl status robomimic-launcher.service --no-pager
journalctl -u robomimic-launcher.service -n 100 --no-pager
```
