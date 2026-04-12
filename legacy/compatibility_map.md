# 兼容层映射（已完成删除）

旧文件已清理完成，以下映射仅用于追溯。

## BSP

- motor.py -> BSP/motor_actuator.py
- chassis.py -> BSP/omni_chassis_driver.py
- main_control.py -> BSP/board_runtime.py

## Module

- imu_fusion.py -> Module/imu_attitude_fusion.py
- odometry.py -> Module/wheel_odometry.py
- estimation.py -> Module/pose_estimation.py
- robot_system.py -> Module/robot_runtime.py
- controller.py -> Module/task_controller_basic.py
- vision.py -> Module/uart_vision_receiver.py
- vision_openart.py -> Module/openart_perception.py
- global_localization.py -> Module/global_pose_initializer.py

## APP

- main_openart.py -> APP/app_openart_single.py
- main_openart_debug.py -> APP/app_openart_debug.py

## 当前主入口

- APP/main.py
- APP/run.py
- APP/single.py
- APP/debug.py
