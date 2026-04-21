import time

from Module import config
from Module.dual_car_coop import build_robot_status


def build_chassis():
    from BSP.motor_actuator import Motor
    from BSP.omni_chassis_driver import OmniChassis

    motor_fl = Motor(*config.MOTOR_FL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fl", False))
    motor_fr = Motor(*config.MOTOR_FR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fr", False))
    motor_bl = Motor(*config.MOTOR_BL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("bl", False))
    motor_br = Motor(*config.MOTOR_BR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("br", False))
    return OmniChassis(motor_fl, motor_fr, motor_bl, motor_br, config=config)


def build_vision(cfg):
    from Module.ydlidar_receiver import YDLidarReceiver

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


def build_obstacle_devices(cfg):
    obstacle = None
    ultrasonic = None

    try:
        from BSP.obstacle_sensors import IRSensorArray

        obstacle = IRSensorArray(
            cfg.OBSTACLE_LEFT_PIN,
            cfg.OBSTACLE_FRONT_PIN,
            cfg.OBSTACLE_RIGHT_PIN,
            active_level=cfg.OBSTACLE_ACTIVE_LEVEL,
        )
    except Exception as e:
        print("obstacle sensor unavailable:", e)

    trig = getattr(cfg, "ULTRASONIC_TRIG_PIN", None)
    echo = getattr(cfg, "ULTRASONIC_ECHO_PIN", None)
    if trig is not None and echo is not None:
        try:
            from BSP.obstacle_sensors import UltrasonicSensor

            ultrasonic = UltrasonicSensor(trig, echo)
        except Exception as e:
            print("ultrasonic unavailable:", e)

    return obstacle, ultrasonic


def build_uwb_pose(cfg):
    if not bool(getattr(cfg, "UWB_POSE_ENABLE", False)):
        return None
    try:
        from Module.uwb_pose_receiver import UWBPoseReceiver

        receiver = UWBPoseReceiver(
            cfg.UWB_UART_ID,
            cfg.UWB_BAUDRATE,
            tx_pin=getattr(cfg, "UWB_TX_PIN", None),
            rx_pin=getattr(cfg, "UWB_RX_PIN", None),
            fmt=getattr(cfg, "UWB_POSE_FORMAT", "csv"),
        )
        print("uwb pose: enabled")

        def pose_source():
            receiver.read_pose()
            return receiver.get_pose()

        return pose_source
    except Exception as e:
        print("uwb pose unavailable:", e)
        return None


def build_car_link(cfg):
    if not bool(getattr(cfg, "COMPETITION_DUAL_ENABLE", False)):
        return None
    try:
        from Module.uart_car_link import UartCarLink

        print("Initializing Seekfree Wireless UART Link...")
        return UartCarLink(
            getattr(cfg, "CAR_LINK_UART_ID", 2),
            getattr(cfg, "CAR_LINK_BAUDRATE", 115200),
            tx_pin=getattr(cfg, "CAR_LINK_TX_PIN", None),
            rx_pin=getattr(cfg, "CAR_LINK_RX_PIN", None),
        )
    except Exception as e:
        print("Failed to initialize Seekfree Link:", e)
        return None


def build_basic_display(cfg):
    if not bool(getattr(cfg, "LCD_ENABLE", True)):
        return None
    try:
        from BSP.board_runtime import LCDGridDisplay

        display = LCDGridDisplay(cfg)
        display.init()
        return display
    except Exception as e:
        print("basic lcd unavailable:", e)
        return None


def build_master_status(controller, pose, chassis, cfg):
    frame = getattr(controller, "_last_frame", None)
    target_visible = bool(isinstance(frame, dict) and frame.get("object"))
    push_states = ("align_push", "push", "coop_push")
    extra = {
        "target_visible": target_visible,
        "coop_push_active": bool(getattr(controller, "state", "") in push_states),
        "formation_enabled": bool(getattr(cfg, "COOP_FORMATION_ENABLE", True)),
    }
    return build_robot_status("master", pose=pose, chassis=chassis, state=getattr(controller, "state", None), extra=extra)


def maybe_print_pose(tag, pose, last_ms_holder, cfg):
    if not bool(getattr(cfg, "DUAL_COORD_PRINT_ENABLE", True)):
        return
    if not pose:
        return
    now = time.ticks_ms()
    interval = int(getattr(cfg, "DUAL_COORD_PRINT_MS", 250))
    if time.ticks_diff(now, last_ms_holder.get("value", 0)) < interval:
        return
    last_ms_holder["value"] = now
    print(
        "[{} POSE] x={:.3f} y={:.3f} yaw={:.1f}".format(
            tag,
            float(pose.get("x", 0.0)),
            float(pose.get("y", 0.0)),
            float(pose.get("yaw", 0.0)),
        )
    )


def wait_c14_start_with_pin():
    if not bool(getattr(config, "BOOT_WAIT_C14_ENABLE", True)):
        return
    try:
        from machine import Pin

        pull_up = getattr(Pin, "PULL_UP_47K", None)
        if pull_up is not None:
            key_pin = Pin(config.NAV_KEY_PIN, Pin.IN, pull_up)
        else:
            key_pin = Pin(config.NAV_KEY_PIN, Pin.IN)

        print("press C14 to start")
        while key_pin.value() != 0:
            time.sleep_ms(20)
        time.sleep_ms(60)
        while key_pin.value() == 0:
            time.sleep_ms(20)
        print("C14 start")
    except Exception:
        pass


def run_basic_mode():
    try:
        import gc

        gc.collect()
    except Exception:
        pass

    from Module.task_controller_lidar_push import LidarSmartPushController

    chassis = build_chassis()
    vision = build_vision(config)
    uwb_pose_source = build_uwb_pose(config)
    basic_display = build_basic_display(config)

    try:
        import gc

        gc.collect()
    except Exception:
        pass

    print("CONTROLLER: SmartCarController (forced, minimal)")
    controller = LidarSmartPushController(chassis, vision, config)
    car_link = build_car_link(config)

    wait_c14_start_with_pin()

    last_sync = 0
    pose_print_ms = {"value": 0}
    while True:
        try:
            controller.update()
            pose = uwb_pose_source() if callable(uwb_pose_source) else None

            if basic_display is not None:
                basic_display.update(chassis, pose)
            maybe_print_pose("MASTER", pose, pose_print_ms, config)

            if car_link:
                if car_link.uart and car_link.uart.any():
                    remote_msg = car_link.receive()
                    if remote_msg:
                        controller.remote_data = car_link.get_remote_data()

                now = time.ticks_ms()
                if time.ticks_diff(now, last_sync) >= getattr(config, "CAR_LINK_SEND_MS", 100):
                    last_sync = now
                    car_link.send(build_master_status(controller, pose, chassis, config))

        except Exception as e:
            import sys

            print("CRASH IN UPDATE", repr(e))
            sys.print_exception(e)
            raise
        time.sleep_ms(config.LOOP_DELAY_MS)


def run_full_mode():
    from BSP.board_runtime import MainControl
    from Module.robot_runtime import DisplayLayer, DriveLayer, EstimationLayer, RobotSystem, TaskLayer
    from Module.task_controller_lidar_push import LidarSmartPushController

    chassis = build_chassis()
    vision = build_vision(config)
    build_obstacle_devices(config)

    control_board = MainControl(config)
    control_board.init()

    uwb_pose_source = build_uwb_pose(config)
    controller = LidarSmartPushController(chassis, vision, config)
    system = RobotSystem(
        control_board,
        DriveLayer(chassis, control_board),
        EstimationLayer(config, control_board, pose_source=uwb_pose_source),
        TaskLayer(controller),
        display_layer=DisplayLayer(control_board),
        communication_layer=None,
    )
    car_link = build_car_link(config)

    if bool(getattr(config, "BOOT_WAIT_C14_ENABLE", True)):
        print("press C14 to start")
        while not control_board.nav_key.is_pressed():
            time.sleep_ms(20)
        time.sleep_ms(60)
        while control_board.nav_key.is_pressed():
            time.sleep_ms(20)
        print("C14 start")

    allow_toggle = bool(getattr(config, "RUNTIME_C14_TOGGLE_ENABLE", True))
    running = True
    key_was_pressed = False
    last_sync = 0
    pose_print_ms = {"value": 0}
    print("competition mode start")

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
                    pose = system.step()
                    maybe_print_pose("MASTER", pose, pose_print_ms, config)

                    if car_link:
                        if car_link.uart and car_link.uart.any():
                            remote_msg = car_link.receive()
                            if remote_msg:
                                controller.remote_data = car_link.get_remote_data()

                        now = time.ticks_ms()
                        if time.ticks_diff(now, last_sync) >= getattr(config, "CAR_LINK_SEND_MS", 100):
                            last_sync = now
                            car_link.send(build_master_status(controller, pose, chassis, config))
                else:
                    if system.display:
                        system.display.update(system.estimation.last_pose, chassis)
            else:
                time.sleep_ms(1)
    finally:
        chassis.stop()


def main():
    try:
        print("competition: build=2026-04-16 dual-coop")
    except Exception:
        pass

    if bool(getattr(config, "COMPETITION_FORCE_BASIC_CONTROLLER", False)):
        run_basic_mode()
        return
    run_full_mode()


if __name__ == "__main__":
    main()
