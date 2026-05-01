# config.py
# 硬件引脚配置和参数设置

# --- RM湖南大学风格：统一入口配置 ---
# 推荐运行 APP/main.py，并通过 APP_PROFILE 选择业务模式。
# 瘦身后推荐：
# - single（主线）
# - debug（标定/调试）
# 其余模式已归档，不在 slim mode 中启用。
# 可选：
# - single
# - debug
# - competition
# - dual_slave
# Slim APP build: only the competition runtime is kept in APP/.
# Hardware/manual test scripts live under tests/hardware/.
APP_PROFILE = "competition"

# --- 实际主控硬件引脚定义 ---
# 4 路电机驱动，使用 PWM + DIR 方式。
MOTOR_FL_PINS = ("D4", "D5")     # 左前：PWM_D4_DIR_D5
MOTOR_FR_PINS = ("D6", "D7")     # 右前：PWM_D6_DIR_D7
MOTOR_BL_PINS = ("C30", "C31")   # 左后：PWM_C30_DIR_C31
MOTOR_BR_PINS = ("C28", "C29")   # 右后：PWM_C28_DIR_C29

# 4 路编码器
ENCODER_FL_PINS = ("C0", "C1")
ENCODER_FR_PINS = ("C2", "C3")
ENCODER_BL_PINS = ("D13", "D14")
ENCODER_BR_PINS = ("D15", "D16")
ENCODER_PINS = {
	"fl": ENCODER_FL_PINS,
	"fr": ENCODER_FR_PINS,
	"bl": ENCODER_BL_PINS,
	"br": ENCODER_BR_PINS,
}

# 按键 / IMU / LCD / SPI / ticker
NAV_KEY_PIN = "C14"
BOOT_WAIT_C14_ENABLE = True
TEST_WAIT_C14_ENABLE = True
# 运行期按 C14 切换：运行 <-> 暂停（暂停时底盘 stop）
RUNTIME_C14_TOGGLE_ENABLE = True
TEST_RUNTIME_C14_TOGGLE_ENABLE = True
LCD_CS_PIN = "B29"
LCD_RST_PIN = "B31"
LCD_DC_PIN = "B5"
LCD_SPI_INDEX = 2
CONTROL_TICKER_ID = 1
CONTROL_PERIOD_MS = 10

# --- 硬件标定 / 方向修正 ---
MOTOR_REVERSE = {
	"fl": False,
	"fr": True,
	"bl": False,
	"br": True,
}
ENCODER_REVERSE = {
	"fl": False,
	"fr": False,
	"bl": False,
	"br": False,
}
ENCODER_CPR = 1024
IMU_GYRO_SCALE = 1.0
IMU_MAG_SCALE = 1.0
IMU_CLASS_NAME = "IMU963RA"
IMU_PINS = ("C10", "C12", "C13")
GYRO_RAW_TO_DPS = 0.07
MAG_RAW_TO_UNIT = 0.0003333333
GYRO_CALIBRATION_WARMUP = 20
GYRO_CALIBRATION_SAMPLES = 100

# 启动阶段硬件初始化开关（用于排查“看起来卡住”的情况）
# - 若串口只停在某行不动，可先把 IMU/LCD 暂时关掉确认是否被初始化阻塞。
IMU_CALIBRATION_ENABLE = True
LCD_ENABLE = True
PULSE_PER_METER = 2387.32414637843
IMU_FUSION_Q_ANGLE = 0.001
IMU_FUSION_Q_GYRO = 0.003
IMU_FUSION_R_ANGLE = 0.03
MAG_CALIBRATION_ENABLE = False
MAG_CALIBRATION_DURATION_S = 5.0

# --- 定点导航 / 全向控制参数 ---
NAV_TARGET_X = 2.0
NAV_TARGET_Y = 1.0
NAV_DEBOUNCE_MS = 20
NAV_REACHED_THRESHOLD_M = 0.05
NAV_LOCK_YAW_DEG = 0.0
NAV_MAX_LINEAR_CMD = 5500
NAV_MAX_ROTATE_CMD = 2500
NAV_WHEEL_PWM_LIMIT = 10000
NAV_POS_KP = 2600.0
NAV_POS_KI = 0.0
NAV_POS_KD = 120.0
NAV_VEL_KP = 1800.0
NAV_VEL_KI = 0.0
NAV_VEL_KD = 40.0
NAV_YAW_KP = 38.0
NAV_YAW_KI = 0.0
NAV_YAW_KD = 2.0
NAV_YAW_RATE_KP = 280.0
NAV_YAW_RATE_KI = 0.0
NAV_YAW_RATE_KD = 8.0
NAV_WHEEL_DAMP = 20.0
NAV_BODY_VEL_LIMIT = 1.2
NAV_BODY_RATE_LIMIT = 180.0
NAV_WHEEL_FEEDFORWARD = 1.0
NAV_DEBUG_ENABLE = True
NAV_DEBUG_PRINT_MS = 250
DUAL_NAV_REMOTE_ENABLE = True

# PWM 频率
PWM_FREQ = 13000
PWM_MAX = 10000

# --- 视觉模块配置 ---
VISION_BACKEND = "uart"  # openart_local / uart
VISION_UART_ID = 5       # 串口ID（实测收到数据：debug_uart_scan 扫描到 UART7 有数据）
VISION_BAUDRATE = 115200 # 波特率
VISION_TX_PIN = 4    # 主控丝印引脚
VISION_RX_PIN = 5   # 主控丝印引脚

# 串口视觉调试输出（VisionReceiver 内部的 VISION_RX 打印）
VISION_DEBUG_PRINT_RX = False

# 双 OpenART 链路状态打印：仅在启用双 OpenART 时建议开启。
VISION_DUAL_STATUS_PRINT = False
VISION_DUAL_STATUS_PRINT_MS = 1000

# 双 OpenART（近场+远场）配置：近场使用 VISION_*；远场使用 VISION2_*
# 近场优先：先看近场是否有球，没有再看远场（见 DualVisionReceiver）。
# 注意：若你暂时还没接远场 OpenART，可先保持 VISION2_* 为 None；
# 这时系统会自动退化为“仅近场”，不会因为远场未配置而崩溃。
VISION_DUAL_ENABLE = False
VISION2_UART_ID = 7
VISION2_BAUDRATE = 115200
VISION2_TX_PIN = "D22"
VISION2_RX_PIN = "D23"

# 图像尺寸，需要与视觉模块输出一致
FRAME_WIDTH = 320
FRAME_HEIGHT = 240

# OpenART 本地视觉识别配置
OPENART_SENSOR_PIXFORMAT = "RGB565"
OPENART_SENSOR_FRAMESIZE = "QVGA"
OPENART_SKIP_FRAMES_MS = 1500
OPENART_AUTO_WHITEBAL = False
OPENART_AUTO_GAIN = False

# LAB 阈值，需现场标定
# 网球（偏黄绿）初始推荐：先保证能稳定检出，再逐步收紧
OPENART_OBJECT_THRESHOLD = (35, 92, -35, 18, 18, 90)
OPENART_ZONE_THRESHOLD = (20, 80, -70, -10, 10, 70)
# 场地边界（10cm黄色胶带）阈值：需现场标定
OPENART_FIELD_THRESHOLD = (40, 95, -15, 25, 35, 95)
OPENART_MIN_PIXELS = 120
OPENART_MIN_AREA = 120
OPENART_MERGE_BLOBS = True

# OpenArt 俯视单角点全局定位参数
OPENART_GLOBAL_LOC_ENABLE = True
OPENART_GLOBAL_LOC_MAX_TRIES = 40
OPENART_VISIBLE_WORLD_CORNER = "BL"  # 视野中开机可见的场地角点：BL/BR/TL/TR
OPENART_PIXEL_TO_M_X = 0.0085         # 初始推荐值，现场按已知位移微调
OPENART_PIXEL_TO_M_Y = 0.0085         # 初始推荐值，现场按已知位移微调
OPENART_PIXEL_TO_ROBOT_X_SIGN = 1.0   # 若x方向反了改为-1
OPENART_PIXEL_TO_ROBOT_Y_SIGN = -1.0  # 俯视常见图像y向下，机器人y向上时用-1
OPENART_CAM_TO_ROBOT_X_M = 0.0        # 相机中心到机器人参考点在机器人坐标系x偏移(m)
OPENART_CAM_TO_ROBOT_Y_M = 0.0        # 相机中心到机器人参考点在机器人坐标系y偏移(m)
OPENART_GLOBAL_CLAMP_TO_FIELD = True

# OpenART 串口调试输出
OPENART_DEBUG_PRINT_JSON = True
OPENART_DEBUG_PRINT_MS = 120

# --- 双车通信配置 ---
CAR_LINK_BACKEND = "uart"  # "uart" / "nrf"

CAR_LINK_UART_ID = 2
CAR_LINK_BAUDRATE = 115200
CAR_LINK_TX_PIN = 6
CAR_LINK_RX_PIN = 7

VISION_OFFSET_X_SIGN = -1
CAR_LINK_NRF_PAYLOAD_SIZE = 32
CAR_LINK_PROTO_MAX_PAYLOAD = 22
CAR_LINK_NRF_CHANNEL = 76
VISION2_OFFSET_X_SIGN = -1
CAR_LINK_NRF_SPI_BAUDRATE = 4000000
CAR_LINK_NRF_CE_PIN = None
CAR_LINK_NRF_CSN_PIN = None
CAR_LINK_NRF_SCK_PIN = None
CAR_LINK_NRF_MOSI_PIN = None
CAR_LINK_NRF_MISO_PIN = None
CAR_LINK_NRF_PIPE_TX = b"\xe7\xe7\xe7\xe7\xe7"
CAR_LINK_NRF_PIPE_RX = b"\xc2\xc2\xc2\xc2\xc2"
CAR_LINK_NRF_RETRY_DELAY_US = 4000
CAR_LINK_NRF_RETRY_COUNT = 10

CAR_LINK_SEND_MS = 60
CAR_LINK_TIMEOUT_MS = 300
CAR_LINK_STATUS_MS = 80
CAR_LINK_STATUS_TIMEOUT_MS = 250
CAR_LINK_STARTUP_GRACE_MS = 1500
DUAL_REQUIRE_SLAVE_READY = True
CAR_LINK_HEARTBEAT_MS = 100
CAR_LINK_MAX_QUEUE = 8

# 主车也回传自身状态（用于双向位姿共享）
CAR_LINK_MASTER_STATUS_MS = 120

# --- UWB(DW3000) 位姿输入（可选） ---
# 若你的 DW3000 定位模块能通过串口输出位姿，可启用此项让系统直接使用 UWB 位姿。
UWB_POSE_ENABLE = False
UWB_UART_ID = 1
UWB_BAUDRATE = 115200
UWB_TX_PIN = None
UWB_RX_PIN = None

# 输出格式：
# - "csv": 形如 "x,y,yaw\n"（单位：米/米/度）
# - "json": 形如 '{"x":0.12,"y":0.34,"yaw":90.0}\n'
UWB_POSE_FORMAT = "csv"

# 若 UWB 超过该时间未更新，则不再覆盖里程计（ms）
UWB_POSE_MAX_AGE_MS = 250

# 副车相对主车的横向偏置。
# 并排推时，副车在主车右侧可设为正值，在左侧设为负值。
SLAVE_LATERAL_BIAS = 18
SLAVE_ROTATE_SCALE = 1.0
SLAVE_SPEED_SCALE = 1.0

# 视觉协议中的目标名
OBJECT_LABEL = "object"
ZONE_LABEL = "zone"

# --- 避障传感器配置 ---
OBSTACLE_LEFT_PIN = 21
OBSTACLE_FRONT_PIN = 22
OBSTACLE_RIGHT_PIN = 23
OBSTACLE_ACTIVE_LEVEL = 0   # 红外避障模块常见为低电平触发

# 超声波版本可选配置
ULTRASONIC_TRIG_PIN = None
ULTRASONIC_ECHO_PIN = None
OBSTACLE_DISTANCE_CM = 18

# --- PID 参数 ---
# 横向居中 PID
PID_CENTER_P = 0.45
PID_CENTER_I = 0.0
PID_CENTER_D = 0.08

# 前进 PID，根据目标大小逼近
PID_DIST_P = 0.90
PID_DIST_I = 0.0
PID_DIST_D = 0.05

# 推送前对齐 PID，根据区域和物体的相对位置进行横移
PID_PUSH_ALIGN_P = 0.55
PID_PUSH_ALIGN_I = 0.0
PID_PUSH_ALIGN_D = 0.05

# --- 阈值 ---
TARGET_OBJECT_SIZE = 280     # 到达物体附近时的像素宽度阈值（更近停车，增大以靠得更近）
CENTER_DEADBAND = 8          # 画面中心死区
ZONE_ALIGN_DEADBAND = 12     # 物体和目标区在图像中重合误差

# 目标确认：连续 N 帧检测到“可信目标”才进入靠近，避免误检导致突然直走/乱冲
TARGET_CONFIRM_FRAMES = 3

# 前进最小目标尺寸：目标太小（远处噪声/黄线碎片）时禁止向前，只允许横移/搜索
APPROACH_FORWARD_MIN_SIZE = 28

# 进入推球的最小尺寸（competition 直推也会受此约束）
PUSH_ENTER_MIN_SIZE = 80

# 推球阶段是否必须持续看到目标。
# True 更安全：目标丢失就先停住，避免“看不到球还一直直走”。
# 若你的球会钻到车底导致视觉必丢，可改为 False 允许短时间盲推。
PUSH_REQUIRE_TARGET = True

SEARCH_ROT_SPEED = 18        # 搜索物体时的旋转速度（自转运动模糊时可适当降低）
SEARCH_ZONE_ROT_SPEED = 18   # 搜索目标区时的旋转速度

# 搜索策略：转一小段 -> 停一下再看（减少运动模糊，提高搜球检出率）
SEARCH_PULSE_ENABLE = True
SEARCH_ROTATE_MS = 160
SEARCH_PAUSE_MS = 120
APPROACH_MIN_SPEED = 12      # 靠近物体的最小前进速度
APPROACH_MAX_SPEED = 32      # 靠近物体的最大前进速度（降低过冲）
MAX_LATERAL_SPEED = 35       # 平移修正最大速度
ALIGN_FORWARD_SPEED = 18     # 对齐目标区时保持轻推的速度
PUSH_SPEED = 55              # 推物体前进速度
PUSH_TIME_MS = 2200          # 推送持续时间

# 推球结束后：短暂后退回到场内，再继续找下一个球
AFTER_PUSH_RETREAT_ENABLE = True
AFTER_PUSH_RETREAT_SPEED = -22
AFTER_PUSH_RETREAT_MS = 550

# 仅停到球前（不进入推送状态）
OBJECT_STOP_MODE = False
# 到达判定余量：size >= TARGET_OBJECT_SIZE - margin 时允许停车
APPROACH_STOP_SIZE_MARGIN = 2
# 停车时的中心误差容忍（像素）
APPROACH_CENTER_STOP_DEADBAND = 6
# 靠近时进入刹车区间（像素差阈值），在此区间内按比例减速
APPROACH_BRAKE_DIST = 60
# 刹车时允许的最小前进速度（像素速度单位，与底盘命令刻度相关）
APPROACH_BRAKE_MIN_SPEED = 4
# 连续帧数门槛：检测到连续 N 帧满足停止条件才真正停止，避免单帧误触发
APPROACH_STOP_FRAMES = 3
# 接近控制指令低通（0~1，越小越平滑）
APPROACH_CMD_SMOOTH_ALPHA = 0.60

# 任务循环
FRAME_TIMEOUT_MS = 500       # 超过该时间未收到视觉帧则视为丢失
LOOP_DELAY_MS = 50           # 主循环间隔

# 安全：允许使用“上一帧视觉”的最大年龄。超过则进入 stale 保护。
# 如果设置过小，串口抖动/帧率下降会频繁触发 stale，表现为卡在 search 或动作忽停忽转。
# stale 期间控制器会把缓存目标标记为 held 并禁止直走，因此这里适当放宽更稳。
VISION_MAX_AGE_MS = 800

# 安全：目标偏离中心太多时，先只做横移居中，不允许向前冲（减少左前/右前斜冲失控）。
APPROACH_STRAFE_ONLY_PX = 140

# 更严格的前进门槛：只有当目标基本居中时才允许向前（否则只横移找中心）。
# 建议 40~80；数值越小越“稳”，但更容易一直横移。
APPROACH_FORWARD_CENTER_PX = 60

# 横移方向符号：如果出现“球在左但车还在往左横移/越移越偏”，把它改成 -1。
VISION_OFFSET_X_SIGN = 1

# 双视觉时远场相机可能是镜像/安装方向不同，可单独设置符号。
# 若不确定，先保持与近场一致；当发现 source=far 时方向反了，再改成 -1。
VISION2_OFFSET_X_SIGN = 1

# 丢目标保持：用于抗抖（偶发掉帧时不中断跟踪）。
# 建议保持很小：50~120ms。
TARGET_LOST_HOLD_MS = 80

# --- LCD 网格显示参数 ---
LCD_GRID_COLS = 10
LCD_GRID_ROWS = 10
LCD_WIDTH = 240
LCD_HEIGHT = 240
LCD_GRID_MARGIN = 10
LCD_BG_COLOR = 0x0000
LCD_GRID_COLOR = 0x7BEF
LCD_CAR_COLOR = 0xF800
LCD_TEXT_COLOR = 0xFFFF

# 基于底盘速度的屏幕估计缩放
LCD_SPEED_TO_CELL = 0.0025

# --- 工程化底盘控制参数 ---
CHASSIS_CMD_DEADBAND = 3
CHASSIS_MAX_VX_STEP = 8
CHASSIS_MAX_VY_STEP = 8
CHASSIS_MAX_VW_STEP = 10
CHASSIS_COMMAND_TIMEOUT_MS = 150
CHASSIS_ENABLE_CLOSED_LOOP = False
WHEEL_PID_P = 0.35
WHEEL_PID_I = 0.02
WHEEL_PID_D = 0.01
WHEEL_PID_LIMIT = 35
WHEEL_SPEED_FILTER = 0.35
# 单轮修正系数：用于补偿电机/减速箱个体差异，提升直线与横移精度
WHEEL_TRIM = {
	"fl": 1.00,
	"fr": 1.00,
	"bl": 1.00,
	"br": 1.00,
}
# 前进/后退可分别标定（只在纯直行 vx 场景下生效）
WHEEL_TRIM_FWD = {
	"fl": 1.00,
	"fr": 0.78,
	"bl": 1.00,
	"br": 0.78,
}
WHEEL_TRIM_REV = {
	"fl": 1.00,
	"fr": 0.90,
	"bl": 1.00,
	"br": 0.90,
}
CHASSIS_DEBUG_PRINT_ENABLE = False
CHASSIS_DEBUG_PRINT_MS = 200

# --- 里程计 / 状态估计参数 ---
ODOM_LINEAR_SCALE = 0.0012
ODOM_ANGULAR_SCALE = 0.0008
ODOM_GYRO_BLEND = 0.65
POSE_MAX_DT_MS = 100

# --- 场地坐标系 / 开机全局定位（初始化位姿注入） ---
# 坐标系建议：以场地某一角为(0,0)，x沿3.2m方向，y沿2.8m方向，yaw为朝向x轴的角度(度)。
FIELD_SIZE_X_M = 3.2
FIELD_SIZE_Y_M = 2.8

# 若主车每次都从固定起点/朝向出发，可以直接开启该配置实现“开机全局定位”。
# 若起点不固定，则需要视觉地标(角标/AprilTag等)才能自动求出(x,y)。
POSE_INIT_ENABLE = False
POSE_INIT_X = 0.0
POSE_INIT_Y = 0.0
POSE_INIT_YAW_DEG = 0.0

# --- 二进制协议参数 ---
PROTO_MAGIC_1 = 0xA5
PROTO_MAGIC_2 = 0x5A
PROTO_VERSION = 1
PROTO_MAX_PAYLOAD = 64

# --- 比赛版避障参数 ---
AVOID_REVERSE_SPEED = -25
AVOID_STRAFE_SPEED = 35
AVOID_ROTATE_SPEED = 25
AVOID_ACTION_MS = 450
OBJECT_LOCK_SIZE = 55        # 目标太小时优先继续追踪，不强制避障绕行

# --- 比赛流程控制参数（双车协同 + 回发车区） ---
COMPETITION_DUAL_ENABLE = False

# 临时止血：competition 控制器初始化在某些固件上会触发 ValueError(expected 4)。
# 开启该开关将强制使用基础 SmartCarController，保证主循环可跑，用于现场验证双路视觉/UART。
# 待问题定位修复后可改回 False 以启用 CompetitionController。
COMPETITION_FORCE_BASIC_CONTROLLER = False

# 放置区判定（默认场地中心 1m x 1m）
CENTER_ZONE_SIZE_M = 1.0
MIN_DROP_SPACING_M = 0.10
MAX_DROPS_TRACKED = 16

# 每次搜球前先回到场地中心：
# - 使用 IMU yaw 锁定航向；
# - 使用四轮编码器里程计估算 x/y 和累计行驶距离；
# - 到中心后再进入 OpenART 搜球。
CENTER_FIRST_ENABLE = True
CENTER_TARGET_X = FIELD_SIZE_X_M * 0.5
CENTER_TARGET_Y = FIELD_SIZE_Y_M * 0.5
CENTER_LOCK_CURRENT_YAW = True
CENTER_LOCK_YAW_DEG = 0.0
CENTER_NAV_KP = 90.0
CENTER_NAV_MAX_SPEED = 38.0
CENTER_NAV_MIN_SPEED = 10.0
CENTER_NAV_TOLERANCE_M = 0.08
CENTER_NAV_STABLE_MS = 250
CENTER_NAV_TIMEOUT_MS = 10000
CENTER_YAW_KP = 1.2
CENTER_YAW_MAX_SPEED = 24.0
CENTER_YAW_DEADBAND_DEG = 1.5
CENTER_YAW_TOLERANCE_DEG = 5.0

# --- 硬件手动测试：陀螺仪锁航向 ---
HEADING_LOCK_CALIBRATE_IMU = True
HEADING_LOCK_TEST_FORWARD_SPEED = 45
HEADING_LOCK_FORWARD_SIGN = -1.0
HEADING_LOCK_YAW_KP = 1.2
HEADING_LOCK_YAW_MAX_SPEED = 22
# 如果车被扭偏后越修越偏，把这个值改成 -1.0。
HEADING_LOCK_YAW_SIGN = 1.0

# 发车区判定（默认左下角 0.5m x 0.5m）
START_ZONE_X_MIN = 0.0
START_ZONE_X_MAX = 0.5
START_ZONE_Y_MIN = 0.0
START_ZONE_Y_MAX = 0.5

# 回发车区控制
HOME_TARGET_X = 0.25
HOME_TARGET_Y = 0.25
RETURN_HOME_ENABLE = False
RETURN_HOME_KP = 120.0
RETURN_HOME_MAX_SPEED = 45
RETURN_HOME_TIMEOUT_MS = 8000
NEXT_TASK_WAIT_MS = 600

# 辅助车禁触碰保护：限制协同推送时横移幅度
ASSIST_SAFE_LATERAL_LIMIT = 15

# 辅助车几何保护（更严格）
# push阶段禁止正向推进，确保辅助车不从物体正后方顶推
SLAVE_PUSH_ALLOW_VX_MAX = 0
# push阶段限制横移方向：与 SLAVE_LATERAL_BIAS 同侧（右侧>0，左侧<0）
SLAVE_PUSH_REQUIRE_BIAS_SIDE = True
# push阶段角速度限幅，减少辅助车前端扫碰
SLAVE_PUSH_MAX_ROTATE = 20
# 中心搬运区附近的额外保护带（米）：进入后进一步压低速度
ASSIST_CENTER_KEEP_OUT_MARGIN_M = 0.25

# --- 协同推送（主从并排推球）---
# 开启后：副车 push 阶段允许有限前进 + 主车可用UWB位姿对副车做队形修正
COOP_PUSH_ENABLE = True
COOP_SLAVE_PUSH_ALLOW_VX_MAX = 28

# 队形保持：目标是让副车相对主车保持固定横向偏置（米）
# 正值代表“副车在主车右侧”，负值代表“副车在主车左侧”（以主车朝向坐标系为准）
COOP_FORMATION_ENABLE = True
COOP_FORMATION_LATERAL_OFFSET_M = 0.35
COOP_FORMATION_KP_VY = 80.0
COOP_FORMATION_MAX_VY = 15

# 多物体清场结束判定：连续未看到目标达到该时间，判定“场上已清空”
MISSION_EMPTY_CONFIRM_MS = 5000

# 清场策略：比赛中允许不依赖zone，接近目标后直接协同推送出界
COMPETITION_DIRECT_PUSH_NO_ZONE = True
# 判定“目标已出界”的图像边缘阈值（像素）
OBJECT_OUT_MARGIN_PX = 14
# 推送阶段连续丢失目标多少帧后，认定该目标已被推出或不可见
# 数值越小越“不会盲推”；若误触发太频繁再增大。
PUSH_LOST_CONFIRM_FRAMES = 2

# 任务结束前，主从都在发车区内持续稳定时间（毫秒）
MISSION_RETURN_STABLE_MS = 1200
