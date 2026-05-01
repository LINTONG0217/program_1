# 双车搬运综合测试清单

适用范围：

- 主车：`UWB + IMU + 编码器 + 雷达 + OpenArt mini + 无线通信`
- 从车：`UWB + IMU + 编码器 + 无线通信`
- 目标任务：双车协同夹持物体并推出 `3.2m x 2.8m` 场地

推荐测试顺序：

1. 底盘
2. IMU / 编码器
3. 雷达
4. UWB / 两锚定位
5. 双车无线通信
6. OpenArt mini
7. 单车综合
8. 双车综合

---

## 1. 底盘测试

测试文件：

- `APP/test_chassis.py`
- `APP/test_straight.py`
- `APP/test_straight_force_trim.py`

检查项：

- 小车前进、后退、横移、旋转都能执行
- 四个轮子转向与预期一致
- 直线行驶时明显跑偏可通过 trim 调整

正常现象：

- 前进时车头基本保持稳定
- 横移时不会变成斜前或斜后
- 原地旋转时不会明显平移

异常排查：

- 某个轮子反转：检查 `MOTOR_REVERSE`
- 明显跑偏：检查 `WHEEL_TRIM`、`WHEEL_TRIM_FWD`、`WHEEL_TRIM_REV`
- 指令一发车不动：检查电机驱动接线与 PWM 引脚

---

## 2. IMU / 编码器测试

相关模块：

- `BSP/board_runtime.py`
- `Module/pose_estimation.py`

建议测试入口：

- `APP/test_uwb_two_anchor_localization.py`

检查项：

- 原地静止时 `yaw` 基本稳定
- 手动缓慢转动车体时 `yaw` 连续变化
- 车向前走时编码器速度有变化

正常现象：

- `imu yaw fusion enabled: True`
- 原地不动时 `yaw` 不会剧烈跳变
- 轻微漂移可以接受，但不能快速乱跳

异常排查：

- `yaw` 方向反了：检查 IMU 安装方向
- `yaw` 不变：检查 IMU 初始化、串口输出和 `UWB_TWO_ANCHOR_USE_IMU`
- 编码器无数据：检查编码器引脚、`ENCODER_REVERSE`、硬件连接

---

## 3. 雷达测试

测试文件：

- `APP/test_lidar.py`
- `APP/test_lidar_track.py`

检查项：

- 串口是否能持续收到扫描数据
- 前方 90 度左右扇区内的物体是否能被稳定发现
- 物体靠近时距离值是否变小

正常现象：

- 串口输出有 `dist=...mm ang=...deg`
- 物体在前方移动时角度能跟着变化
- 没有目标时不是一直卡死在旧值

异常排查：

- 一直没数据：检查 `LIDAR_UART_ID`、`LIDAR_BAUDRATE`、`LIDAR_TX_PIN`、`LIDAR_RX_PIN`
- 角度左右反了：检查 `LIDAR_ANGLE_SIGN`
- 前方目标扫不到：检查 `LIDAR_FRONT_MIN_DEG`、`LIDAR_FRONT_MAX_DEG`

---

## 4. UWB / 两锚定位测试

测试文件：

- `APP/test_uwb_raw.py`
- `APP/test_uwb_distance.py`
- `APP/test_uwb_pose.py`
- `APP/test_uwb_two_anchor_localization.py`

检查项：

- 两个基站测距都能更新
- 出现稳定的 `[2A FIX]` 或综合输出中的坐标更新
- 小车移动时 `x, y` 随之变化

正常现象：

- `id0=...m id1=...m` 持续刷新
- 位姿不会在两个镜像位置来回跳
- 轻微噪声可以接受，但整体趋势应正确

异常排查：

- 只有一个基站有值：检查基站 ID 和 AT 配置
- 坐标跳镜像：检查 `UWB_TWO_ANCHOR_PREFERRED_SIDE`
- 全局位置更新很慢：检查 `UWB_RANGE_POLL_MS`、`UWB_RANGE_MAX_AGE_MS`

---

## 5. 双车无线通信测试

测试文件：

- `APP/test_dual_uart_link.py`
- 从车运行：`APP/dual_slave.py`

检查项：

- 主车能发，从车能收
- 从车丢主车后会停止而不是乱跑
- 主车进入协同推送时，从车跟随距离会变小

正常现象：

- 无线链路建立后，从车能持续收到主车状态
- 主车移动时，从车能跟在旁边
- 主车停下时，从车也能停下

异常排查：

- 完全收不到：检查 `CAR_LINK_UART_ID`、波特率、无线模块供电
- 从车乱冲：检查主车状态时间戳、`SLAVE_MAX_MASTER_AGE_MS`
- 跟随方向反了：检查 `COOP_FORMATION_LATERAL_OFFSET_M` 正负

---

## 6. OpenArt mini 测试

测试文件：

- `openart/openart_near_ball.py`

主车接收模块：

- `Module/openart_mini_receiver.py` 中的 `OpenArtMiniReceiver`

检查项：

- OpenArt mini 是否通过串口输出标签
- 输出标签是否和主控配置映射一致
- 近距离识别是否稳定

正常现象：

- 串口输出 `label=...` 或 `none`
- 标签和物体种类一致
- 同一物体连续几帧标签基本不变

异常排查：

- 总是 `none`：扩大 `ROI`，重调颜色阈值
- 标签错乱：提高 `STABLE_FRAMES`，收紧 `THRESHOLDS`
- 主控不响应：检查 `OPENART_MINI_ENABLE` 和 UART 号

---

## 7. 单车综合测试

测试文件：

- `APP/test_uwb_two_anchor_localization.py`

检查项：

- 小车能扫描到物体
- 能生成局部栅格图并规划路径
- 接近后触发 OpenArt 识别
- 能按规则把物体向对应边推动

正常现象：

- 串口输出中有 `state=search_object`
- 找到目标后变成 `approach_object`
- 识别后进入 `transport_to_edge`

异常排查：

- 一直停在 `search_object`：先查雷达和目标检测
- 一直靠近但不识别：检查 `OPENART_RECOGNIZE_DISTANCE_M`
- 识别后不运输：检查 `OPENART_TARGET_EDGE_MAP`

---

## 8. 双车综合测试

主车：

- `APP/test_uwb_two_anchor_localization.py`

从车：

- `APP/dual_slave.py`

检查项：

- 从车能稳定跟在主车侧边
- 主车识别完目标后进入协同搬运
- 搬运过程中地图持续更新
- 物体位置变化时主车能继续修正运输方向
- 场地内清空后进入停止状态

正常现象：

- 主车输出 `state=transport_to_edge`
- 从车在主车一侧维持较小横向偏置
- 物体推出边界后主车重新搜索下一个目标
- 全场清空后主车进入 `match_done`

异常排查：

- 搬运时两车分得太开：调 `COOP_PUSH_LATERAL_OFFSET_M`
- 从车顶到物体前后侧：调 `COOP_PUSH_FORWARD_OFFSET_M`
- 物体明明还在场内却误判完成：当前轻量追踪链路已不使用栅格 track TTL，优先检查 OpenArt/雷达目标是否稳定输出。
- 场地已经空了还不停：调 `MISSION_EMPTY_CONFIRM_MS`

---

## 建议现场顺序

第一次联调整体建议：

1. 先单车验证底盘、IMU、编码器、雷达、UWB
2. 再让从车单独验证 UWB 与无线链路
3. 然后验证主车识别和单车推送
4. 最后再开双车协同搬运

不要一上来全功能一起跑。

最常见的问题不是算法，而是：

- 串口号不对
- 波特率不一致
- IMU 安装方向错误
- 电机正反与编码器方向不一致
- OpenArt 标签名和主控映射表不一致
