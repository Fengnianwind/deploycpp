# AnyAdapter Deploy Design

## Goal

在现有 `RoboMimic_Deploy` 部署框架中新增一个独立的 `AnyAdapter` 策略目录与状态机入口，保留 `BeyondMimic` / `BeyondMimic2` 不变，使训练侧导出的三输入 ONNX 可以同时在 `deploy_mujoco` 和 `deploy_real` 中切换部署。

## Architecture

- 新增 `policy/anyadapter` 目录，包含独立的 `AnyAdapter.py`、运行时辅助模块、配置文件和模型目录。
- 复用当前 `BeyondMimic` 的 tracking 观测构造方式，外部维护 `history`，并按训练 README 中固定的 10 维顺序构建 `balance_features`。
- 由于当前部署侧不具备训练端同等完整的接触/足端状态，本次先实现可部署近似版特征：
  - 严格保留特征顺序与 shape
  - 能从现有信号直接获得的量正常计算
  - 无法可靠恢复的量使用稳定近似或零值退化

## Components

- `policy/anyadapter/runtime.py`
  - 历史缓冲更新
  - tracking 观测拼接
  - 10 维 balance feature 组装
  - motion `.npz` 帧索引与参考数据读取辅助
- `policy/anyadapter/AnyAdapter.py`
  - 加载三输入 ONNX
  - 维护 `history`
  - 读取 motion `.npz`
  - 生成 `adaptive_action`
- `FSM` / `common.utils`
  - 新增 `FSMStateName.SKILL_ANYADAPTER`
  - 将 `FSMCommand.SKILL_2` 绑定到新状态
- `deploy_real` / `deploy_mujoco`
  - `R1+Y` 触发 `AnyAdapter`

## Data Flow

1. 用户在 `LocoMode` 下触发 `R1+Y`
2. FSM 切到 `AnyAdapter`
3. `AnyAdapter` 从 motion `.npz` 读取当前时间步的参考关节和参考 torso 姿态
4. 部署侧构建 154 维 tracking `obs`
5. 部署侧维护 `(1, H, obs_dim + action_dim)` 的 `history`
6. 部署侧构建 `(1, 10)` 的 `balance_features`
7. ONNX 输出 `adaptive_action`
8. 动作映射回 29 维关节目标并下发

## Error Handling

- 缺少 ONNX 或 motion 文件时，初始化直接抛出清晰异常
- ONNX 输入输出名不匹配时，启动时报错
- motion 时间步超过长度时，钳制到最后一帧

## Testing

- 新增纯 Python 单元测试覆盖：
  - tracking 观测拼接长度与顺序
  - history 缓冲滚动行为
  - balance feature 的 10 维顺序与 double-support 逻辑
- 运行 `python -m unittest discover -s tests -v`
