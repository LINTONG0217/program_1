# RM湖南大学风格项目整理说明

## 统一入口

项目主程序已统一放入 APP 目录：

- [APP/main.py](APP/main.py)
- [APP/run.py](APP/run.py)

通过 [Module/config.py](Module/config.py) 中的 `APP_PROFILE` 选择运行模式（`single` / `competition`）。

## 推荐模式

- `single`：单车（仅雷达）
- `competition`：比赛模式（仅雷达）

## 分层对应（RM常用）

- APP层：`APP/main.py`、`APP/run.py`、`APP/single.py`、`APP/competition.py`
- Module层：`Module/ydlidar_receiver.py`、`Module/task_controller_lidar_push.py`、`Module/pose_estimation.py`、`Module/imu_attitude_fusion.py`、`Module/wheel_odometry.py`、`Module/omni_pid_navigation.py`、`Module/task_controller_basic.py`、`Module/task_controller_competition.py`、`Module/dual_robot_comm.py`、`Module/rm_binary_protocol.py`、`Module/linked_chassis.py`、`Module/system_menu.py`
- BSP层：`BSP/board_runtime.py`、`BSP/motor_actuator.py`、`BSP/omni_chassis_driver.py`、`BSP/device_adapters.py`、`BSP/obstacle_sensors.py`、`BSP/grid_display.py`

## 本次精简

旧主程序与根目录功能文件已并入三层目录并删除，根目录仅保留文档、部署脚本与工程文件。
