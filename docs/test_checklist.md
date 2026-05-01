# 现场测试打勾清单

## 0. 上电前

- [ ] 电池电压正常，急停可用
- [ ] 轮子离地空转检查无卡滞
- [ ] OpenArt 串口线连接正确

## 1. 底盘动作测试（先做）

文件：APP/test_chassis.py

- [ ] 前进方向正确
- [ ] 后退方向正确
- [ ] 右平移方向正确
- [ ] 左平移方向正确
- [ ] 顺时针/逆时针旋转都正常

失败处理：

- 方向反：调整 Module/config.py 中 `MOTOR_REVERSE`
- 单轮不动：检查电机线、驱动口、对应引脚

## 2. 视觉调试测试

文件：APP/debug.py（`APP_PROFILE=debug`）

- [ ] 串口持续输出 JSON
- [ ] `object` 能随目标移动变化
- [ ] `zone` 可稳定识别
- [ ] `field` / `field_corner` 可识别黄色边界和角点

失败处理：

- 识别不稳：调 Module/config.py 的阈值（`OPENART_*_THRESHOLD`）
- 无输出：检查波特率、TX/RX、视觉端脚本是否运行

## 3. 主流程单车测试

文件：APP/single.py（`APP_PROFILE=single`）

- [ ] 上电启动正常
- [ ] 无异常抖动、无乱冲
- [ ] 目标可被搜索并接近

失败处理：

- 抖动：降低速度参数（`APPROACH_MAX_SPEED`、`PUSH_SPEED`）
- 搜索慢：提高 `SEARCH_ROT_SPEED`

## 4. 开机全局定位测试（俯视+可见角点）

- [ ] 启动日志出现 `openart global init`
- [ ] 初始位置与真实位置接近
- [ ] 朝向符号正确（不镜像）

失败处理：

- 无法初始化：提高 `OPENART_GLOBAL_LOC_MAX_TRIES`
- 坐标方向反：调整 `OPENART_PIXEL_TO_ROBOT_X_SIGN / Y_SIGN`
- 比例偏差：微调 `OPENART_PIXEL_TO_M_X / Y`

## 5. 任务闭环测试（追物→推区）

- [ ] 搜索目标稳定
- [ ] 对齐目标区有效
- [ ] 推送动作连贯
- [ ] 结束后能恢复下一轮

失败处理：

- 推不动/偏航：降低 `PUSH_SPEED`，加大 `PID_PUSH_ALIGN_P`
- 过冲：降低 `PID_DIST_P` 或提高 `CENTER_DEADBAND`

## 6. 稳定性回归（至少3轮）

- [ ] 连续运行无死机
- [ ] 串口无明显掉线
- [ ] 定位误差不持续累积

判定建议：

- 6/6 全通过：可进赛前参数固化
- 4~5 通过：按失败处理项复测
- ≤3 通过：先回到第1步硬件排查
