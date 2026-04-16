import sys
import time

from Module import config


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


def main():
    try:
        print("competition: build=2026-04-04 basic-fallback")
    except Exception:
        pass
    force_basic = bool(getattr(config, "COMPETITION_FORCE_BASIC_CONTROLLER", False))
    is_lidar_backend = True
    if force_basic:
        # Ultra-minimal path: import controller as early as possible (heap is largest now)
        # and avoid heavy runtime/display/control modules to prevent MemoryError.
        try:
            import gc

            gc.collect()
        except Exception:
            pass

        from Module.task_controller_lidar_push import LidarSmartPushController as SmartCarController

        local_chassis = build_chassis()
        # In forced-basic mode we deliberately skip dual-link/obstacle/UWB/display layers.
        chassis = local_chassis
        vision = build_vision(config)
        try:
            import gc

            gc.collect()
        except Exception:
            pass
        print("CONTROLLER: SmartCarController (forced, minimal)")
        controller = SmartCarController(chassis, vision, config)

        try:
            from Module.uart_car_link import UartCarLink
            print("Initializing Seekfree Wireless UART Link...")
            car_link = UartCarLink(getattr(config, "CAR_LINK_UART_ID", 2), getattr(config, "CAR_LINK_BAUDRATE", 115200))
        except Exception as e:
            print("Failed to initialize Seekfree Link:", e)
            car_link = None

        if bool(getattr(config, "BOOT_WAIT_C14_ENABLE", True)):
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

        last_sync = 0
        while True:
            try:
                controller.update()
                
                # --- Seekfree Wireless Dual Car Sync ---
                if car_link:
                    # 1. 鎺ユ敹鍙︿竴鍙ц溅鐨勭姸鎬?                    if car_link.uart and car_link.uart.any():
                        remote_msg = car_link.receive()
                        if remote_msg:
                            # 濡傛灉闇€瑕佹牴鎹彟涓€鍙ц溅鐨勭姸鎬佸仛鍐崇瓥锛屽彲浠ュ湪鎺у埗鍣ㄩ噷闈㈠鐞?                            controller.remote_data = car_link.get_remote_data()
                            # debug only
                            # print("<- Remote:", controller.remote_data)
                            
                    # 2. 瀹氭湡鍙戦€佽嚜宸辩殑鐘舵€佸埌鍙︿竴鍙ц溅
                    now = time.ticks_ms()
                    if time.ticks_diff(now, last_sync) >= getattr(config, "CAR_LINK_SEND_MS", 100):
                        last_sync = now
                        my_status = {
                            "state": controller.state,
                            "target_visible": bool(getattr(controller, "_last_frame", {}).get("object"))
                        }
                        car_link.send(my_status)
                        
            except Exception as e:
                import sys
                print("CRASH IN UPDATE", repr(e))
                sys.print_exception(e)
                raise
            time.sleep_ms(config.LOOP_DELAY_MS)
        return

    # Full competition runtime (heavier).
    from BSP.board_runtime import MainControl
    from Module.robot_runtime import DisplayLayer, DriveLayer, EstimationLayer, RobotSystem, TaskLayer

    local_chassis = build_chassis()
    chassis = local_chassis
    communication = None
    vision = build_vision(config)
    obstacle_sensor, ultrasonic = build_obstacle_devices(config)

    controller = None
    control_board = MainControl(config)
    control_board.init()

    uwb_pose_source = build_uwb_pose(config)

    system_holder = {"system": None}

    def pose_provider():
        system = system_holder["system"]
        if not system:
            return None
        return system.estimation.last_pose

    def in_start_zone(pose):
        if not pose:
            return False
        x = float(pose.get("x", 0.0))
        y = float(pose.get("y", 0.0))
        return (
            float(getattr(config, "START_ZONE_X_MIN", 0.0)) <= x <= float(getattr(config, "START_ZONE_X_MAX", 0.5))
            and float(getattr(config, "START_ZONE_Y_MIN", 0.0)) <= y <= float(getattr(config, "START_ZONE_Y_MAX", 0.5))
        )

    def remote_home_fn():
        status = getattr(chassis, "last_status", None)
        if not status:
            return False
        remote_pose = status.get("status", {}).get("pose")
        return in_start_zone(remote_pose)

    if not force_basic:
        from Module.task_controller_lidar_push import LidarSmartPushController

        print("CONTROLLER: LidarSmartPushController")
        controller = LidarSmartPushController(chassis, vision, config)

    system = RobotSystem(
        control_board,
        DriveLayer(chassis, control_board),
        EstimationLayer(config, control_board, pose_source=uwb_pose_source),
        TaskLayer(controller),
        display_layer=DisplayLayer(control_board),
        communication_layer=None,
    )
    system_holder["system"] = system

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
                    system.step()
                else:
                    if system.display:
                        system.display.update(system.estimation.last_pose, chassis)
            else:
                time.sleep_ms(1)
    finally:
        chassis.stop()


if __name__ == "__main__":
    main()
