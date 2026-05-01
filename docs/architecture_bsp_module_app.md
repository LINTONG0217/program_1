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
- APP：强调“业务场景”，例如 `app_openart_single`、`app_dual_master`

## 兼容策略

当前采用“新文件名 + 兼容映射”的重构方式：

- 旧代码不立即删除，避免一次性改动过大导致不可运行。
- 新开发统一在 BSP/Module/APP 三层文件进行。
- 稳定后可再做第二阶段：把旧文件迁入 legacy 并物理裁剪。
