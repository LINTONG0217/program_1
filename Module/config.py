# config.py
# 硬件引脚配置和参数设�?

# --- RM湖南大学风格：统一入口配置 ---
# 推荐运行 APP/main.py，并通过 APP_PROFILE 选择业务模式�?
# 瘦身后推荐：
# - single（主线）
# - competition（比赛模式）
# 其余模式已归档，不在 slim mode 中启用�?
# 可选：
# - single
# - competition
# - dual_slave
APP_PROFILE = "test_lidar_track"

# Vehicle preset switch:
# - master_test
# - slave_test
# - master_competition
# - slave_competition
# - manual (disable preset override)
VEHICLE_PRESET = "manual"

# --- 实际主控硬件引脚定义 ---
# 4 路电机驱动，使用 PWM + DIR 方式�?
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
# 运行期按 C14 切换：运�?<-> 暂停（暂停时底盘 stop�?
RUNTIME_C14_TOGGLE_ENABLE = True
TEST_RUNTIME_C14_TOGGLE_ENABLE = True
MOTOR_OUTPUT_ENABLE = True
MOTOR_SPEED_LIMIT_PERCENT = 25
MOTOR_MAX_SPEED_STEP_PERCENT = 4
MOTOR_DIRECT_DUTY_LIMIT = 2500
TEST_CHASSIS_DIRECT_DUTY = 1000
TEST_CHASSIS_MOTOR_DURATION_MS = 2000
TEST_CHASSIS_STEP_GAP_MS = 1000
TEST_CHASSIS_MOVE_SPEED = 15
TEST_CHASSIS_ROTATE_SPEED = 12
TEST_CHASSIS_MOVE_DURATION_MS = 2000
LCD_CS_PIN = "B29"
LCD_RST_PIN = "B31"
LCD_DC_PIN = "B5"
LCD_SPI_INDEX = 2
CONTROL_TICKER_ID = 1
CONTROL_PERIOD_MS = 10

# --- 硬件标定 / 方向修正 ---
MOTOR_REVERSE = {
	"fl": True,
	"fr": False,
	"bl": True,
	"br": False,
}
ENCODER_REVERSE = {
	"fl": False,
	"fr": False,
	"bl": False,
	"br": False,
}
ENCODER_CPR = 1024
MOTOR_GEAR_RATIO = 100.0
ENCODER_MOTOR_CPR = ENCODER_CPR
ENCODER_OUTPUT_CPR = ENCODER_MOTOR_CPR * MOTOR_GEAR_RATIO
# Set the real wheel diameter and enable DERIVE_ODOM_SCALE_FROM_MOTOR when
# encoder counts are measured on the motor shaft and should include gearbox ratio.
WHEEL_DIAMETER_M = 0.06
DERIVE_ODOM_SCALE_FROM_MOTOR = False
IMU_GYRO_SCALE = 1.0
IMU_MAG_SCALE = 1.0
GYRO_RAW_TO_DPS = 0.07
MAG_RAW_TO_UNIT = 0.0003333333
GYRO_CALIBRATION_WARMUP = 20
GYRO_CALIBRATION_SAMPLES = 100

# 启动阶段硬件初始化开关（用于排查“看起来卡住”的情况�?
# - 若串口只停在某行不动，可先把 IMU/LCD 暂时关掉确认是否被初始化阻塞�?
IMU_CALIBRATION_ENABLE = True
LCD_ENABLE = True
PULSE_PER_METER_CALIBRATED = 2387.32414637843
PULSE_PER_METER_DERIVED = ENCODER_OUTPUT_CPR / (3.141592653589793 * WHEEL_DIAMETER_M)
PULSE_PER_METER = PULSE_PER_METER_DERIVED if DERIVE_ODOM_SCALE_FROM_MOTOR else PULSE_PER_METER_CALIBRATED
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

# --- 比赛场地 ---
FIELD_WIDTH_M = 3.2
FIELD_HEIGHT_M = 2.8
FIELD_EXIT_MARGIN_M = 0.08
FIELD_TRACK_MARGIN_M = 0.22

# PWM 频率
PWM_FREQ = 13000
PWM_MAX = 10000

# --- 感知模块配置 ---
# 当前工程已精简为：仅保�?YDLIDAR T-MINI PLUS 雷达感知�?

# 图像尺寸（控制器仍以“frame/object”抽象工作，lidar 会映射为等效像素�?
FRAME_WIDTH = 320
FRAME_HEIGHT = 240

# --- YDLIDAR T-MINI PLUS 配置 ---
LIDAR_UART_ID = 7
LIDAR_BAUDRATE = 230400
LIDAR_TX_PIN = "D22"
LIDAR_RX_PIN = "D23"

# 启动扫描：部�?YDLIDAR（含 T-MINI PLUS）需要主控发送指令后才开始持续输�?AA 55 扫描包�?
# - STOP: A5 65
# - SCAN: A5 60
LIDAR_SEND_START_CMD = True
LIDAR_START_WITH_STOP = True
LIDAR_START_DELAY_MS = 120

# UART 兼容/探测�?
# - �?sys.platform == "mimxrt"（常�?OpenMV/OpenART 风格固件）上，UART 往往不支持传 tx/rx，且 UART 号可能不�?5�?
# - 开�?AUTO_FALLBACK 后会尝试多个 UART id，并通过 PROBE_MS 选取“真的在收数据”的那一路�?
LIDAR_UART_AUTO_FALLBACK = False
LIDAR_UART_PROBE_MS = 150

# 雷达角度窗口（单位：度，前方�?0，左正右负）
LIDAR_FRONT_MIN_DEG = 25
LIDAR_FRONT_MAX_DEG = 65
LIDAR_IGNORE_ANGLE_RANGES = ((25, 31), (59, 65))

# 雷达安装修正
LIDAR_ANGLE_OFFSET_DEG = 0.0
LIDAR_ANGLE_SIGN = 1.0

# 有效测距区间（mm�?
LIDAR_MIN_DISTANCE_MM = 40
LIDAR_MAX_DISTANCE_MM = 700
LIDAR_TRACK_TARGET_DISTANCE_MM = 80
LIDAR_TRACK_DISTANCE_DEADBAND_MM = 55
LIDAR_TRACK_MIN_SIZE = 90
LIDAR_TRACK_CENTER_DEG = 45.0
LIDAR_TRACK_ANGLE_DEADBAND_DEG = 3.0
LIDAR_TRACK_ALIGN_FIRST_DEG = 8.0
LIDAR_TRACK_VX = 16
LIDAR_TRACK_REVERSE_VX = 0
LIDAR_TRACK_VY_PER_DEG = 2.2
LIDAR_TRACK_MAX_VY = 20
LIDAR_TRACK_SEARCH_VW = 10
LIDAR_TRACK_LOOP_MS = 20
LIDAR_TRACK_FILTER_ALPHA = 0.35
LIDAR_TRACK_HOLD_MS = 450

# 将角度映射为控制器使用的 offset_x（像素等效量�?
LIDAR_OFFSET_X_PER_DEG = 4.0

# 将距离映射为控制器使用的 size（越近越大）
LIDAR_SIZE_SCALE = 60000.0
LIDAR_SIZE_MAX = 420
LIDAR_HOLD_MS = 120
LIDAR_SCAN_TIMEOUT_MS = 260
LIDAR_KEEP_SCAN_POINTS = False

# --- 雷达栅格建图 / A* ---
LIDAR_GRID_WIDTH_M = 2.4
LIDAR_GRID_HEIGHT_M = 2.4
LIDAR_GRID_RESOLUTION_M = 0.08
LIDAR_GRID_FRONT_MARGIN_M = 1.6
LIDAR_GRID_REAR_MARGIN_M = 0.8
LIDAR_GRID_INFLATION_M = 0.12
LIDAR_OBJECT_INFLATION_M = 0.05
LIDAR_GOAL_STANDOFF_M = 0.28
LIDAR_CLUSTER_GAP_DEG = 7.0
LIDAR_CLUSTER_GAP_MM = 180.0
LIDAR_CLUSTER_MIN_POINTS = 3
LIDAR_PLAN_INTERVAL_MS = 260
LIDAR_PLAN_PRINT_MS = 220

# --- 栅格路径跟踪 ---
GRID_FOLLOW_MAX_SPEED = 28
GRID_FOLLOW_MIN_SPEED = 10
GRID_FOLLOW_MAX_STRAFE = 24
GRID_FOLLOW_MAX_YAW = 18
GRID_FOLLOW_YAW_P = 0.22
GRID_FOLLOW_LOOKAHEAD_POINTS = 3
GRID_FOLLOW_ARRIVE_XY_M = 0.10
GRID_APPROACH_OBJECT_M = 0.38
MATCH_TRACK_MATCH_DIST_M = 0.32
MATCH_TRACK_TTL_MS = 2200
MATCH_EXIT_CONFIRM_MS = 1200
COOP_TRANSPORT_MIN_VX = 12
COOP_TRANSPORT_FALLBACK_VX = 16
COOP_TRANSPORT_MIN_MS = 900
COOP_TRANSPORT_TIMEOUT_MS = 5000
COOP_DELIVER_EDGE_NEAR_M = 0.22
COOP_DELIVER_CROSS_MARGIN_M = 0.25
COOP_DELIVER_CONFIRM_MS = 350
COOP_DELIVER_LOST_CONFIRM_MS = 650
COOP_BLIND_PUSH_MAX_MS = 900

# 目标尺寸阈值：�?lidar 而言等效于“推送触发距离�?
# 经验值：LIDAR_SIZE_SCALE / TARGET_OBJECT_SIZE �?触发距离(mm)
# 例如 60000/260 �?230mm
TARGET_OBJECT_SIZE = 260

# lidar 模式建议：不依赖 zone，达到阈值后直接进入推送流�?
COMPETITION_DIRECT_PUSH_NO_ZONE = True

# 推送阶段更稳一�?
PUSH_REQUIRE_TARGET = False

# --- OpenArt mini 近距离识别 ---
# 串口输出推荐格式：
# - label=red,conf=0.92
# - class=green,score=0.87
# - red
OPENART_MINI_ENABLE = True
OPENART_MINI_UART_ID = 5
OPENART_MINI_BAUDRATE = 115200
OPENART_MINI_TX_PIN = "D20"
OPENART_MINI_RX_PIN = "D21"
OPENART_MINI_TIMEOUT_MS = 900
OPENART_RECOGNIZE_DISTANCE_M = 0.42
OPENART_EDGE_PUSH_TIME_MS = 1500
OPENART_EDGE_PUSH_SPEED = 22
OPENART_EDGE_FORWARD_SPEED = 30
OPENART_DEFAULT_EDGE = "right"
OPENART_TARGET_EDGE_MAP = {
	"red": "left",
	"green": "right",
	"blue": "top",
	"yellow": "bottom",
	"tennis": "top",
	"tennis_ball": "top",
	"volleyball": "top",
	"standard_ball": "top",
	"standard_volleyball": "top",
	"标准网球": "top",
	"标准球": "top",
	"toy_sandbag": "left",
	"sandbag": "left",
	"bag": "left",
	"玩具沙袋": "left",
	"沙袋": "left",
	"teddy": "right",
	"teddy_bear": "right",
	"bear": "right",
	"泰迪熊": "right",
	"小熊": "right",
}

# --- 双车通信配置 ---

CAR_LINK_UART_ID = 2
CAR_LINK_BAUDRATE = 115200
CAR_LINK_TX_PIN = 6
CAR_LINK_RX_PIN = 7

CAR_LINK_PROTO_MAX_PAYLOAD = 22

CAR_LINK_SEND_MS = 60
CAR_LINK_TIMEOUT_MS = 300
CAR_LINK_STATUS_MS = 80
CAR_LINK_STATUS_TIMEOUT_MS = 250
CAR_LINK_STARTUP_GRACE_MS = 1500
DUAL_REQUIRE_SLAVE_READY = True
CAR_LINK_HEARTBEAT_MS = 100
CAR_LINK_MAX_QUEUE = 8

# 双车串口测试角色："master" 或 "slave"
CAR_LINK_TEST_ROLE = "master"

# 主车也回传自身状态（用于双向位姿共享�?
CAR_LINK_MASTER_STATUS_MS = 120
CAR_LINK_ROLE = "master"

# --- UWB(DW3000) 位姿输入（可选） ---
# 若你�?DW3000 定位模块能通过串口输出位姿，可启用此项让系统直接使�?UWB 位姿�?
UWB_POSE_ENABLE = True
UWB_UART_ID = 4
UWB_BAUDRATE = 115200
UWB_TX_PIN = "D0"
UWB_RX_PIN = "D1"

# 输出格式�?
# - "csv": 形如 "x,y,yaw\n"（单位：�?�?度）
# - "json": 形如 '{"x":0.12,"y":0.34,"yaw":90.0}\n'
UWB_POSE_FORMAT = "csv"

# �?UWB 超过该时间未更新，则不再覆盖里程计（ms�?
UWB_POSE_MAX_AGE_MS = 180

# --- Two-anchor UWB test defaults ---
# Used by APP/test_uwb_two_anchor_localization.py only.
UWB_ANCHOR_0_ID = 0
UWB_ANCHOR_0_X_M = 0.0
UWB_ANCHOR_0_Y_M = 0.0
UWB_ANCHOR_1_ID = 1
UWB_ANCHOR_1_X_M = 3.2
UWB_ANCHOR_1_Y_M = 0.0
UWB_RANGE_MAX_AGE_MS = 260
UWB_TWO_ANCHOR_INTERSECTION_TOL_M = 0.10
UWB_TWO_ANCHOR_PREFERRED_SIDE = 1
UWB_TAG_OFFSET_FORWARD_M = 0.0
UWB_TAG_OFFSET_LATERAL_M = 0.0
UWB_RANGE_POLL_ENABLE = True
UWB_RANGE_POLL_MS = 100
UWB_RANGE_CMD_TEMPLATE = "AT+DISTANCE\r\n"
UWB_TWO_ANCHOR_LIDAR_ENABLE = False
UWB_TWO_ANCHOR_PRINT_MS = 200
UWB_TWO_ANCHOR_INIT_ENABLE = True
UWB_TWO_ANCHOR_INIT_X_M = 0.25
UWB_TWO_ANCHOR_INIT_Y_M = 0.25
UWB_TWO_ANCHOR_INIT_YAW_DEG = 0.0
UWB_TWO_ANCHOR_USE_IMU = True

# 副车相对主车的横向偏置�?
# 并排推时，副车在主车右侧可设为正值，在左侧设为负值�?
SLAVE_LATERAL_BIAS = 18
SLAVE_ROTATE_SCALE = 1.0
SLAVE_SPEED_SCALE = 1.0
DUAL_COORD_PRINT_ENABLE = True
DUAL_COORD_PRINT_MS = 250

# 视觉协议中的目标�?
OBJECT_LABEL = "object"
ZONE_LABEL = "zone"

# --- 避障传感器配�?---
OBSTACLE_LEFT_PIN = 21
OBSTACLE_FRONT_PIN = 22
OBSTACLE_RIGHT_PIN = 23
OBSTACLE_ACTIVE_LEVEL = 0   # 红外避障模块常见为低电平触发

# 超声波版本可选配�?
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

# 推送前对齐 PID，根据区域和物体的相对位置进行横�?
PID_PUSH_ALIGN_P = 0.55
PID_PUSH_ALIGN_I = 0.0
PID_PUSH_ALIGN_D = 0.05

# --- 阈�?---
# TARGET_OBJECT_SIZE 在上方已定义，供雷达感知映射与控制逻辑共用
CENTER_DEADBAND = 8          # 画面中心死区
ZONE_ALIGN_DEADBAND = 12     # 物体和目标区在图像中重合误差

# 目标确认：连�?N 帧检测到“可信目标”才进入靠近，避免误检导致突然直走/乱冲
TARGET_CONFIRM_FRAMES = 3

# 前进最小目标尺寸：目标太小（远处噪�?黄线碎片）时禁止向前，只允许横移/搜索
APPROACH_FORWARD_MIN_SIZE = 28

# 进入推球的最小尺寸（competition 直推也会受此约束�?
PUSH_ENTER_MIN_SIZE = 80

# 推球阶段是否必须持续看到目标�?
# True 更安全：目标丢失就先停住，避免“看不到球还一直直走”�?
# 若你的球会钻到车底导致视觉必丢，可改�?False 允许短时间盲推�?
PUSH_REQUIRE_TARGET = False

SEARCH_ROT_SPEED = 18        # 搜索物体时的旋转速度（自转运动模糊时可适当降低�?
SEARCH_ZONE_ROT_SPEED = 18   # 搜索目标区时的旋转速度

# 搜索策略：转一小段 -> 停一下再看（减少运动模糊，提高搜球检出率�?
SEARCH_PULSE_ENABLE = True
SEARCH_ROTATE_MS = 160
SEARCH_PAUSE_MS = 120
APPROACH_MIN_SPEED = 12      # 靠近物体的最小前进速度
APPROACH_MAX_SPEED = 32      # 靠近物体的最大前进速度（降低过冲）
MAX_LATERAL_SPEED = 35       # 平移修正最大速度
ALIGN_FORWARD_SPEED = 18     # 对齐目标区时保持轻推的速度
PUSH_SPEED = 55              # 推物体前进速度
PUSH_TIME_MS = 2200          # 推送持续时�?

# 推球结束后：短暂后退回到场内，再继续找下一个球
AFTER_PUSH_RETREAT_ENABLE = True
AFTER_PUSH_RETREAT_SPEED = -22
AFTER_PUSH_RETREAT_MS = 550

# 仅停到球前（不进入推送状态）
OBJECT_STOP_MODE = False
# 到达判定余量：size >= TARGET_OBJECT_SIZE - margin 时允许停�?
APPROACH_STOP_SIZE_MARGIN = 2
# 停车时的中心误差容忍（像素）
APPROACH_CENTER_STOP_DEADBAND = 6
# 靠近时进入刹车区间（像素差阈值），在此区间内按比例减�?
APPROACH_BRAKE_DIST = 60
# 刹车时允许的最小前进速度（像素速度单位，与底盘命令刻度相关�?
APPROACH_BRAKE_MIN_SPEED = 4
# 连续帧数门槛：检测到连续 N 帧满足停止条件才真正停止，避免单帧误触发
APPROACH_STOP_FRAMES = 3
# 接近控制指令低通（0~1，越小越平滑�?
APPROACH_CMD_SMOOTH_ALPHA = 0.60

# 任务循环
FRAME_TIMEOUT_MS = 500       # 超过该时间未收到视觉帧则视为丢失
LOOP_DELAY_MS = 50           # 主循环间�?

# 安全：允许使用“上一帧视觉”的最大年龄。超过则进入 stale 保护�?
# 如果设置过小，串口抖�?帧率下降会频繁触�?stale，表现为卡在 search 或动作忽停忽转�?
# stale 期间控制器会把缓存目标标记为 held 并禁止直走，因此这里适当放宽更稳�?
VISION_MAX_AGE_MS = 800

# 安全：目标偏离中心太多时，先只做横移居中，不允许向前冲（减少左前/右前斜冲失控）�?
APPROACH_STRAFE_ONLY_PX = 140

# 更严格的前进门槛：只有当目标基本居中时才允许向前（否则只横移找中心）�?
# 建议 40~80；数值越小越“稳”，但更容易一直横移�?
APPROACH_FORWARD_CENTER_PX = 60

# 横移方向符号：如果出现“球在左但车还在往左横�?越移越偏”，把它改成 -1�?
VISION_OFFSET_X_SIGN = 1

# 双视觉时远场相机可能是镜�?安装方向不同，可单独设置符号�?
# 若不确定，先保持与近场一致；当发�?source=far 时方向反了，再改�?-1�?
VISION2_OFFSET_X_SIGN = 1

# 丢目标保持：用于抗抖（偶发掉帧时不中断跟踪）�?
# 建议保持很小�?0~120ms�?
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

# 基于底盘速度的屏幕估计缩�?
LCD_SPEED_TO_CELL = 0.0025

# --- 工程化底盘控制参�?---
CHASSIS_CMD_DEADBAND = 3
CHASSIS_VY_SIGN = -1
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
# 单轮修正系数：用于补偿电�?减速箱个体差异，提升直线与横移精度
WHEEL_TRIM = {
	"fl": 1.00,
	"fr": 1.00,
	"bl": 1.00,
	"br": 1.00,
}
# 前进/后退可分别标定（只在纯直�?vx 场景下生效）
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

# --- 里程�?/ 状态估计参�?---
ODOM_LINEAR_SCALE_CALIBRATED = 0.0012
ODOM_LINEAR_SCALE_DERIVED = 1.0 / PULSE_PER_METER
ODOM_LINEAR_SCALE = ODOM_LINEAR_SCALE_DERIVED if DERIVE_ODOM_SCALE_FROM_MOTOR else ODOM_LINEAR_SCALE_CALIBRATED
ODOM_ANGULAR_SCALE = 0.0008
ODOM_GYRO_BLEND = 0.65
POSE_MAX_DT_MS = 100

# --- 场地坐标�?/ 开机全局定位（初始化位姿注入�?---
# 坐标系建议：以场地某一角为(0,0)，x�?.2m方向，y�?.4m方向，yaw为朝向x轴的角度(�?�?
FIELD_SIZE_X_M = 3.2
FIELD_SIZE_Y_M = 2.4

# 若主车每次都从固定起�?朝向出发，可以直接开启该配置实现“开机全局定位”�?
# 若起点不固定，则需要视觉地�?角标/AprilTag�?才能自动求出(x,y)�?
POSE_INIT_ENABLE = False
POSE_INIT_X = 0.0
POSE_INIT_Y = 0.0
POSE_INIT_YAW_DEG = 0.0

# --- 二进制协议参�?---
PROTO_MAGIC_1 = 0xA5
PROTO_MAGIC_2 = 0x5A
PROTO_VERSION = 1
PROTO_MAX_PAYLOAD = 64

# --- 比赛版避障参�?---
AVOID_REVERSE_SPEED = -25
AVOID_STRAFE_SPEED = 35
AVOID_ROTATE_SPEED = 25
AVOID_ACTION_MS = 450
OBJECT_LOCK_SIZE = 55        # 目标太小时优先继续追踪，不强制避障绕�?

# --- 比赛流程控制参数（双车协�?+ 回发车区�?---
COMPETITION_DUAL_ENABLE = True

# 临时止血：competition 控制器初始化在某些固件上会触�?ValueError(expected 4)�?
# lidar 模式建议关闭该开关，以启�?CompetitionController 的“无 zone 直推”逻辑�?
# 注意：由�?mimxrt 固件内存极其受限，强行加载全�?Controller 会导�?76KB alloc MemoryError�?
COMPETITION_FORCE_BASIC_CONTROLLER = True

# 放置区判定（默认场地中心 1m x 1m�?
CENTER_ZONE_SIZE_M = 1.0
MIN_DROP_SPACING_M = 0.10
MAX_DROPS_TRACKED = 16

# 发车区判定（默认左下�?0.5m x 0.5m�?
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
# push阶段禁止正向推进，确保辅助车不从物体正后方顶�?
SLAVE_PUSH_ALLOW_VX_MAX = 0
# push阶段限制横移方向：与 SLAVE_LATERAL_BIAS 同侧（右�?0，左�?0�?
SLAVE_PUSH_REQUIRE_BIAS_SIDE = True
# push阶段角速度限幅，减少辅助车前端扫碰
SLAVE_PUSH_MAX_ROTATE = 20
# 中心搬运区附近的额外保护带（米）：进入后进一步压低速度
ASSIST_CENTER_KEEP_OUT_MARGIN_M = 0.25

# --- 协同推送（主从并排推球�?--
# 开启后：副�?push 阶段允许有限前进 + 主车可用UWB位姿对副车做队形修正
COOP_PUSH_ENABLE = True
COOP_SLAVE_PUSH_ALLOW_VX_MAX = 28

# 队形保持：目标是让副车相对主车保持固定横向偏置（米）
# 正值代表“副车在主车右侧”，负值代表“副车在主车左侧”（以主车朝向坐标系为准�?
COOP_FORMATION_ENABLE = True
COOP_FORMATION_FORWARD_OFFSET_M = 0.00
COOP_FORMATION_LATERAL_OFFSET_M = 0.35
COOP_PUSH_FORWARD_OFFSET_M = 0.04
COOP_PUSH_LATERAL_OFFSET_M = 0.22
COOP_FORMATION_KP_VY = 80.0
COOP_FORMATION_MAX_VY = 15
SLAVE_POSITION_KP_VX = 90.0
SLAVE_YAW_KP = 1.3
SLAVE_FEEDFORWARD_SCALE = 0.85
SLAVE_FOLLOW_MAX_VX = 35
SLAVE_FOLLOW_MAX_VY = 35
SLAVE_FOLLOW_MAX_VW = 28
SLAVE_TARGET_DEADBAND_M = 0.03
SLAVE_YAW_DEADBAND_DEG = 4.0
SLAVE_MAX_MASTER_AGE_MS = 400
LCD_POSE_TEXT_MS = 120

# 多物体清场结束判定：连续未看到目标达到该时间，判定“场上已清空�?
MISSION_EMPTY_CONFIRM_MS = 5000
MISSION_FIELD_EMPTY_MARGIN_M = 0.03
MISSION_FINAL_SWEEP_ROT_SPEED = 10
MISSION_FINAL_SWEEP_MIN_DEG = 330
MISSION_FINAL_SWEEP_MAX_MS = 9000
MISSION_FINAL_EMPTY_CONFIRM_MS = 5000

# 清场策略：比赛中允许不依赖zone，接近目标后直接协同推送出�?
COMPETITION_DIRECT_PUSH_NO_ZONE = True
# 判定“目标已出界”的图像边缘阈值（像素�?
OBJECT_OUT_MARGIN_PX = 14
# 推送阶段连续丢失目标多少帧后，认定该目标已被推出或不可�?
# 数值越小越“不会盲推”；若误触发太频繁再增大�?
PUSH_LOST_CONFIRM_FRAMES = 2

# 任务结束前，主从都在发车区内持续稳定时间（毫秒）
MISSION_RETURN_STABLE_MS = 1200


try:
	from Module.vehicle_config_presets import apply_preset as _apply_vehicle_preset

	_apply_vehicle_preset(globals(), globals().get("VEHICLE_PRESET", "manual"))
except Exception as _vehicle_preset_error:
	try:
		print("vehicle preset skipped:", _vehicle_preset_error)
	except Exception:
		pass
