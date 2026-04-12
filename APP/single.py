"""单车雷达主程序（仅保留 YDLIDAR T-MINI PLUS）。"""

import time
from BSP.board_runtime import MainControl
from BSP.motor_actuator import Motor
from BSP.omni_chassis_driver import OmniChassis
from Module import config
from Module.robot_runtime import DisplayLayer, DriveLayer, EstimationLayer, RobotSystem, TaskLayer
from Module.task_controller_lidar_push import LidarSmartPushController
from Module.ydlidar_receiver import YDLidarReceiver


def build_chassis():
    motor_fl = Motor(*config.MOTOR_FL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fl", False))
    motor_fr = Motor(*config.MOTOR_FR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fr", False))
    motor_bl = Motor(*config.MOTOR_BL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("bl", False))
    motor_br = Motor(*config.MOTOR_BR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("br", False))
    for name, motor, pins in (
        ("fl", motor_fl, config.MOTOR_FL_PINS),
        ("fr", motor_fr, config.MOTOR_FR_PINS),
        ("bl", motor_bl, config.MOTOR_BL_PINS),
        ("br", motor_br, config.MOTOR_BR_PINS),
    ):
        print(
            "motor init:",
            name,
            "pins=", pins,
            "backend=", "seekfree" if getattr(motor, "controller", None) is not None else "machine.PWM",
            "reverse=", getattr(motor, "reverse", False),
        )
    return OmniChassis(motor_fl, motor_fr, motor_bl, motor_br, config=config)


def build_vision(cfg):
    print(
        "vision backend: lidar uart={} baud={} tx={} rx={}".format(
            getattr(cfg, "LIDAR_UART_ID", None),
            getattr(cfg, "LIDAR_BAUDRATE", 230400),
            getattr(cfg, "LIDAR_TX_PIN", None),
            getattr(cfg, "LIDAR_RX_PIN", None),
        )
    )
    return YDLidarReceiver(
        getattr(cfg, "LIDAR_UART_ID", None),
        getattr(cfg, "LIDAR_BAUDRATE", 230400),
        getattr(cfg, "LIDAR_TX_PIN", None),
        getattr(cfg, "LIDAR_RX_PIN", None),
        cfg,
    )


def main():
    chassis = build_chassis()
    vision = build_vision(config)
    controller = LidarSmartPushController(chassis, vision, config)
    control_board = MainControl(config)
    control_board.init()

    uwb_pose_source = None
    if bool(getattr(config, "UWB_POSE_ENABLE", False)):
        try:
            from Module.uwb_pose_receiver import UWBPoseReceiver

            receiver = UWBPoseReceiver(
                config.UWB_UART_ID,
                config.UWB_BAUDRATE,
                tx_pin=getattr(config, "UWB_TX_PIN", None),
                rx_pin=getattr(config, "UWB_RX_PIN", None),
                fmt=getattr(config, "UWB_POSE_FORMAT", "csv"),
            )

            def uwb_pose_source():
                receiver.read_pose()
                return receiver.get_pose()

            print("uwb pose: enabled")
        except Exception as e:
            print("uwb pose unavailable:", e)
            uwb_pose_source = None

    system = RobotSystem(
        control_board,
        DriveLayer(chassis, control_board),
        EstimationLayer(config, control_board, pose_source=uwb_pose_source),
        TaskLayer(controller),
        display_layer=DisplayLayer(control_board),
    )

    if bool(getattr(config, "BOOT_WAIT_C14_ENABLE", True)):
        print("press C14 to start")
        while not control_board.nav_key.is_pressed():
            time.sleep_ms(20)
        # 去抖 + 等待松手，避免长按触发误动作
        time.sleep_ms(60)
        while control_board.nav_key.is_pressed():
            time.sleep_ms(20)
        print("C14 start")

    print("lidar mode start")
    allow_toggle = bool(getattr(config, "RUNTIME_C14_TOGGLE_ENABLE", True))
    running = True
    key_was_pressed = False
    try:
        while True:
            if allow_toggle:
                key_pressed = control_board.nav_key.is_pressed()
                if key_pressed and not key_was_pressed:
                    running = not running
                    if running:
                        print("C14: resume")
                    else:
                        print("C14: pause")
                        chassis.stop()
                    time.sleep_ms(60)
                key_was_pressed = key_pressed

            if control_board and control_board.ticker.ready():
                if running:
                    system.step()
                else:
                    if system.display:
                        system.display.update(system.estimation.last_pose, chassis)
            else:
                time.sleep_ms(1)
    finally:
        chassis.stop()
