# 免终端按键启动方案

## Summary

目标不是把当前 `deploycpp` 改成“开机就直接接管机器人运动”，而是把启动控制做成一套更保守的两层结构：

- 第 1 层是常驻后台的 `launcher`
- 第 2 层是现有的 `robomimic_deploycpp --run "$PROJECT_ROOT"` 主控制程序

`launcher` 只负责监听 `low_state`、解码遥控器、管理子进程生命周期，不发送 `low_cmd`。真正的运动控制仍然只由 `runner` 持有。

目标体验是：

1. 机器人按原厂流程开机并进入可控状态。
2. 后台 `launcher` 已经常驻并等待人工确认。
3. 操作者先执行一次武装手势。
4. 再按一次启动确认键。
5. `runner` 被拉起并从 `Passive` 开始运行。

这套方案的核心原则是：

- 启动必须是两阶段确认，不能由单个裸按键直接触发。
- `launcher` 和 `runner` 必须严格分离，避免双控制环争抢控制权。
- `runner` 异常退出允许有限次数自动恢复，但不允许无限重启。

---

## Design Goals

- 免终端启动：不依赖人工登录和手敲命令。
- 保守安全：任何启动动作都必须有明确人工确认。
- 单一控制权：只有 `runner` 可以发送运动命令。
- 可恢复：偶发异常退出后允许自动恢复。
- 可诊断：现场必须能通过日志快速判断卡在哪个状态。

非目标：

- 不改变机器人原厂开机流程。
- 不替代原厂 `debug mode` 进入方式。
- 不修改主程序现有状态机键位语义。
- 不让 `launcher` 直接接管关节控制或发送姿态命令。

---

## Proposed Architecture

启动结构改成 `launcher + runner`：

- `launcher`
  - 开机自启，常驻后台。
  - 初始化 DDS。
  - 仅订阅 `low_state`。
  - 解码遥控器按键。
  - 管理 `runner` 子进程的拉起、退出、退避和锁定。
- `runner`
  - 继续复用现有 `robomimic_deploycpp --run "$PROJECT_ROOT"`。
  - 仍然负责 FSM、模型推理和 `low_cmd` 发送。
  - 启动后默认进入 `Passive`。

`launcher` 建议作为现有可执行文件的新增模式实现，例如：

```bash
robomimic_deploycpp --launcher "$PROJECT_ROOT"
```

这样可避免额外二进制、路径和依赖管理复杂化。

---

## Launcher State Machine

`launcher` 不应是“监听到按键就起进程”的脚本，而应是一个显式状态机。推荐定义以下状态：

- `Idle`
  - 后台待机。
  - 已初始化 DDS。
  - 仅监听 `low_state` 和遥控器。
  - 未武装，任何启动确认键都无效。

- `Armed`
  - 已满足启动前提。
  - 等待一次明确的启动确认。
  - 只在短时间窗口内有效。

- `Starting`
  - 正在拉起 `runner`。
  - 屏蔽重复触发。

- `Running`
  - `runner` 已存活。
  - `launcher` 只监控，不发送 `low_cmd`。

- `Backoff`
  - `runner` 异常退出后等待自动恢复。
  - 在退避计时期间不接受新的启动请求。

- `Locked`
  - 自动恢复预算耗尽。
  - 不再自动重启。
  - 必须重新执行人工确认流程后才能再次启动。

推荐状态流转：

```text
Idle
  -> Armed      : low_state 稳定 + 收到武装手势
Armed
  -> Starting   : 收到启动确认键上升沿
  -> Idle       : 武装超时 / low_state 丢失
Starting
  -> Running    : runner 成功拉起
  -> Backoff    : 启动失败
Running
  -> Idle       : runner 正常退出
  -> Backoff    : runner 异常退出
  -> Idle       : low_state 丢失
Backoff
  -> Starting   : 退避结束且仍允许自动恢复
  -> Locked     : 超过重启阈值
  -> Idle       : low_state 丢失
Locked
  -> Armed      : 操作者重新完成武装流程
```

---

## Arming And Start Gesture

这里不建议把“进入原厂 `debug mode`”直接等价为“允许启动 `runner`”，因为 `launcher` 未必能从 `low_state` 中稳定读到一个明确的 `debug mode` 标志。

更稳的定义是：只有当 `launcher` 能观测到一组可验证条件时，才允许进入 `Armed`。

推荐的 `Idle -> Armed` 条件：

- 已持续收到稳定 `low_state`
- 当前没有 `runner` 在运行
- 操作者执行了一次专用武装手势

推荐手势设计：

- `L2 + R2`：武装手势
- `X`：启动确认键

具体语义：

- `L2 + R2` 只负责让 `launcher` 进入 `Armed`
- `X` 只在 `Armed` 状态下有效
- `X` 只认上升沿，不接受长按重复触发
- `Armed` 必须有超时，例如 `5s`
- `Armed` 超时后自动回 `Idle`
- `low_state` 一旦丢失，立即撤销 `Armed`

推荐的人机流程：

1. 机器人按原厂流程开机并进入可控状态。
2. `launcher` 在后台稳定接收到 `low_state`。
3. 操作者按一次 `L2 + R2` 完成武装。
4. `launcher` 进入 `Armed`。
5. 操作者在武装窗口内按一次 `X`。
6. `launcher` 拉起 `runner`。

这样启动是“两阶段确认”，不是裸 `X` 触发，误启动风险明显更低。

---

## Runner Exit Policy

`runner` 的退出语义建议分为三类：

- 正常退出
  - 退出码 `0`
  - `launcher` 回到 `Idle`
  - 等待下一次人工武装和启动确认

- 可恢复异常
  - 非 `0` 退出
  - 尚未超过自动恢复预算
  - 进入 `Backoff`

- 锁定异常
  - 短时间内连续失败次数超过阈值
  - 进入 `Locked`
  - 必须重新人工确认

推荐的自动恢复策略：

- 最大自动重启次数：`3`
- 统计窗口：`60s`
- 退避时间：`1s -> 3s -> 10s`
- 第 `4` 次仍失败：进入 `Locked`

再加两个约束：

- 如果 `runner` 存活时间小于 `2s`，计为快速失败
- 在 `Running` 或 `Backoff` 期间如果 `low_state` 丢失，取消自动恢复并回 `Idle`

推荐行为总结：

- 正常退出：不自动重启
- 偶发异常：有限次自动重启
- 连续异常：锁定等待人工重新确认

这个策略的目标是处理偶发故障，而不是掩盖持续性故障。无限自动重启不应出现在机器人主控制程序的启动路径里。

---

## Ownership And Safety Semantics

这部分必须定死：

- `launcher` 绝不发送 `low_cmd`
- `launcher` 不进入 FSM
- `launcher` 不生成姿态、轨迹或策略命令
- `runner` 是唯一持有运动控制权的进程

主程序的现有运行语义保持不变：

- 启动后默认 `Passive`
- `Start -> FixedPose`
- 其它策略切换逻辑不改
- `F1 -> Passive`
- `Select -> 正常退出 runner`

主程序退出后的安全语义继续沿用当前实现：

- 正常退出前发阻尼/安全态
- 异常路径优先尝试发一次阻尼/安全态

`launcher` 只负责“是否拉起进程”，不负责“机器人应该如何运动”。

---

## Shared Remote Decode

`launcher` 和 `runner` 不能各自维护一套按键位表，必须共用一套遥控器解码来源。

共享解码层至少应包含：

- `L2`
- `R2`
- `R1`
- `L1`
- `Start`
- `Select`
- `F1`
- `A`
- `B`
- `X`
- `Y`

要求：

- 统一由一处代码定义位映射
- `launcher` 与主程序共用同一份解码逻辑
- 位定义对齐仓库内已有 Python 遥控器实现和现有 C++ 使用语义

否则会出现“武装键位和主程序实际读取键位不一致”的隐患。

---

## Systemd Integration

推荐在目标机器上增加一个 `systemd` 服务，开机自动运行：

```bash
robomimic_deploycpp --launcher "$PROJECT_ROOT"
```

仓库内建议直接提供一个系统服务文件，例如：

```text
deploy_real/deploycpp/systemd/robomimic-launcher.service
```

如果当前机器路径保持为：

```text
/home/abc/RoboMimic_Deploy
```

则服务文件中的关键字段应固定为：

```text
User=abc
WorkingDirectory=/home/abc/RoboMimic_Deploy/deploy_real/deploycpp
ExecStart=/home/abc/RoboMimic_Deploy/deploy_real/deploycpp/build_release/robomimic_deploycpp --launcher /home/abc/RoboMimic_Deploy
```

`systemd` 的职责：

- 开机自启 `launcher`
- `launcher` 自身异常退出时自动拉起
- 提供统一日志入口

`systemd` 不应直接管理 `runner`，因为 `runner` 的生命周期应由 `launcher` 管控。

换句话说：

- `systemd` 管 `launcher`
- `launcher` 管 `runner`

不要让 `systemd` 直接把 `runner` 设成“崩了就无限重启”的服务。

---

## Logging And Observability

`launcher` 必须输出稳定、可机读的日志字段，至少包括：

- `low_state_connected=1/0`
- `launcher_state=Idle|Armed|Starting|Running|Backoff|Locked`
- `arm_reason=l2_r2_combo`
- `start_trigger=x_rising_edge`
- `runner_pid=...`
- `runner_exit_code=...`
- `restart_attempt=1/2/3`
- `backoff_seconds=...`
- `lock_reason=too_many_failures|low_state_lost`

建议行为：

- 只有状态变化时打印状态日志，避免刷屏
- 子进程启动、退出、重启、锁定必须打印
- `low_state` 首次连上和丢失都要打印

这样现场无论是看终端、串口还是 `journalctl`，都能明确判断系统当前卡在哪一层。

---

## Acceptance Criteria

### 1. 正常链路

- 开机后 `launcher` 常驻
- 收到稳定 `low_state`
- 按 `L2 + R2` 后进入 `Armed`
- 在超时前按 `X`，进入 `Starting -> Running`
- `runner` 启动后打印 `state=Passive`
- 按 `Start` 能进入 `FixedPose`
- 按 `Select` 后 `runner` 正常退出，`launcher` 回 `Idle`

### 2. 误触保护

- 未武装时按 `X` 不会启动 `runner`
- `Armed` 超时后按 `X` 不会启动 `runner`
- `Running` 状态下再次按 `X` 不会重复拉起
- `Backoff` 状态下人工乱按不会打断退避策略

### 3. 异常恢复

- 人为制造 `runner` 异常退出
- 验证自动恢复最多执行 `3` 次
- 验证退避时间符合 `1s -> 3s -> 10s`
- 第 `4` 次仍失败后进入 `Locked`
- `Locked` 后必须重新执行武装和确认流程

### 4. 信号丢失

- 在 `Armed` 期间丢失 `low_state`，自动回 `Idle`
- 在 `Running` 期间丢失 `low_state`，取消自动恢复并回 `Idle`
- 在 `Backoff` 期间丢失 `low_state`，终止退避并回 `Idle`

---

## Open Implementation Notes

实现时建议保持以下边界：

- `main.cpp`
  - 新增 `--launcher`
- `RobotIO` / 遥控器解码
  - 抽出共享解码函数
  - 补齐 `L2/R2`
- 新增 launcher 模块
  - 只做监听、状态机和子进程管理
- 现有 `--run`
  - 尽量不改控制语义

实现优先级建议：

1. 先统一遥控器解码
2. 再实现 `launcher` 状态机
3. 再接入子进程拉起和退避逻辑
4. 最后补 `systemd` 服务和真机验收

---

## Final Recommendation

推荐采用“保守安全版”：

- 常驻 `launcher`
- `L2 + R2` 武装
- `X` 上升沿启动确认
- `runner` 有限次数自动恢复
- 连续失败后锁定

这比“进入 debug mode 后裸按 `X` 就启动”更稳，也比“异常退出就无限自动重启”更符合机器人控制场景。
