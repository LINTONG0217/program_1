# BSP / Module / APP 三层架构总览

## 目标

- BSP：硬件相关（板级支持、驱动、适配）
- Module：算法与中间能力（定位、融合、控制、通信）
- APP：业务入口与任务流程

## 新目录

- [BSP](../BSP)
- [Module](../Module)
- [APP](../APP)
- [docs/BSP](BSP)
- [docs/Module](Module)
- [docs/APP](APP)

## 入口方式

统一入口为 [main.py](../main.py)，按 [config.py](../config.py) 里的 `APP_PROFILE` 运行。

## 命名原则

- BSP：强调“硬件职责”，例如 `motor_actuator`、`board_runtime`
- Module：强调“算法职责”，例如 `pose_estimation`、`imu_attitude_fusion`
- APP：强调“业务场景”，例如 `single`、`competition`、`dual_slave`

## 当前状态

当前仓库已经完成一轮物理裁剪：

- 已移除旧的 OpenART 摄像机兼容空壳与旧调试入口
- 主线开发集中在当前仍保留的 BSP / Module / APP 文件
- 历史映射仅保留在 `legacy/compatibility_map.md` 供追溯

当前建议使用的 APP 入口：

- `APP/single.py`
- `APP/competition.py`
- `APP/dual_slave.py`
- `APP/test_uwb_two_anchor_localization.py`
