import gc
import time

from machine import Pin

from BSP.motor_actuator import Motor
from BSP.omni_chassis_driver import OmniChassis
from Module import config
from Module.mahony_ahrs import MahonyYawAHRS
from Module.openart_mini_receiver import OpenArtMiniReceiver


def clamp(value, low, high):
    return max(low, min(high, value))


def wait_c14_start():
    if not bool(getattr(config, "TEST_WAIT_C14_ENABLE", True)):
        return
    pin_name = getattr(config, "NAV_KEY_PIN", "C14")
    pull_up = getattr(Pin, "PULL_UP_47K", None)
    if pull_up is not None:
        try:
            key = Pin(pin_name, Pin.IN, pull_up)
        except Exception:
            key = Pin(pin_name, Pin.IN)
    else:
        key = Pin(pin_name, Pin.IN)

    print("========================================")
    print("Press {} to start Lidar Tracking Test".format(pin_name))
    print("========================================")
    while key.value() != 0:
        time.sleep_ms(20)
    time.sleep_ms(60)
    while key.value() == 0:
        time.sleep_ms(20)


class RuntimeStopKey:
    def __init__(self):
        self.enabled = bool(getattr(config, "TEST_RUNTIME_C14_STOP_ENABLE", True))
        self.pin = None
        self.down_ms = None
        self.hold_ms = int(getattr(config, "TEST_RUNTIME_C14_STOP_HOLD_MS", 700))
        self.printed = False
        if not self.enabled:
            return
        pin_name = getattr(config, "NAV_KEY_PIN", "C14")
        pull_up = getattr(Pin, "PULL_UP_47K", None)
        try:
            if pull_up is not None:
                self.pin = Pin(pin_name, Pin.IN, pull_up)
            else:
                self.pin = Pin(pin_name, Pin.IN)
        except Exception:
            self.pin = None

    def should_stop(self):
        if not self.enabled or self.pin is None:
            return False
        try:
            pressed = self.pin.value() == 0
        except Exception:
            return False
        now = time.ticks_ms()
        if not pressed:
            self.down_ms = None
            self.printed = False
            return False
        if self.down_ms is None:
            self.down_ms = now
            return False
        if not self.printed and time.ticks_diff(now, self.down_ms) > 120:
            print("[STOP] keep holding C14 to exit...")
            self.printed = True
        return time.ticks_diff(now, self.down_ms) >= self.hold_ms


def build_chassis():
    motor_fl = Motor(*config.MOTOR_FL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fl", False))
    motor_fr = Motor(*config.MOTOR_FR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fr", True))
    motor_bl = Motor(*config.MOTOR_BL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("bl", False))
    motor_br = Motor(*config.MOTOR_BR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("br", True))
    return OmniChassis(motor_fl, motor_fr, motor_bl, motor_br, config=config)


def build_lidar():
    if not bool(getattr(config, "HYBRID_LIDAR_ENABLE", False)):
        print("[INFO] Lidar disabled by HYBRID_LIDAR_ENABLE=False")
        return None
    try:
        from Module.ydlidar_receiver import YDLidarReceiver
        return YDLidarReceiver(
            uart_id=getattr(config, "LIDAR_UART_ID", 5),
            baudrate=getattr(config, "LIDAR_BAUDRATE", 230400),
            tx_pin=getattr(config, "LIDAR_TX_PIN", 4),
            rx_pin=getattr(config, "LIDAR_RX_PIN", 5),
            config=config,
        )
    except Exception as e:
        print("[WARN] Lidar disabled:", e)
        if bool(getattr(config, "LIDAR_TRACK_REQUIRE_LIDAR", False)):
            raise
        return None


def build_openart():
    if not bool(getattr(config, "HYBRID_OPENART_ENABLE", True)):
        return None
    return OpenArtMiniReceiver(config)


def build_imu():
    if not bool(getattr(config, "LIDAR_TRACK_MAHONY_ENABLE", True)):
        return None
    try:
        from BSP.board_runtime import IMUDevice
    except Exception:
        return None
    imu = IMUDevice(config)
    if bool(getattr(config, "LIDAR_TRACK_IMU_CALIBRATE", False)):
        imu.calibrate()
    return imu


def angle_error_deg(target, current):
    err = float(target) - float(current)
    while err > 180.0:
        err -= 360.0
    while err <= -180.0:
        err += 360.0
    return err


def valid_openart_result(result):
    if not isinstance(result, dict):
        return False
    label = str(result.get("label", "")).strip().lower()
    if label in ("", "none", "null", "no", "openart_near_ball_start"):
        return False
    if bool(result.get("held", False)) and not bool(getattr(config, "HYBRID_OPENART_ALLOW_HELD", False)):
        return False
    conf = float(result.get("confidence", result.get("conf", 1.0)) or 0.0)
    if conf < float(getattr(config, "HYBRID_OPENART_MIN_CONF", 0.0)):
        return False
    pixels = result.get("pixels", result.get("area", None))
    if pixels is not None:
        try:
            if float(pixels) < float(getattr(config, "HYBRID_OPENART_MIN_PIXELS", 80)):
                return False
        except Exception:
            pass
    return True


def openart_track_command(result):
    label = str(result.get("label", "")).strip()
    pixels = float(result.get("pixels", result.get("area", 0)) or 0)
    cx = result.get("cx", result.get("x", None))
    if cx is None:
        vx = float(getattr(config, "HYBRID_OPENART_FORWARD_SPEED", getattr(config, "OPENART_TRACK_FORWARD_SPEED", 20)))
        return (vx, 0.0), "[OPENART] label={} no_cx | vx={:.0f}".format(label, vx)

    cx = float(cx)
    center_x = float(getattr(config, "OPENART_TRACK_CENTER_X", 160))
    err = cx - center_x
    deadband = float(getattr(config, "OPENART_TRACK_DEADBAND_PX", 18))
    center_first_px = float(getattr(config, "HYBRID_OPENART_CENTER_FIRST_PX", 38))
    forward_center_px = float(getattr(config, "HYBRID_OPENART_FORWARD_CENTER_PX", 22))
    push_center_px = float(getattr(config, "HYBRID_OPENART_PUSH_CENTER_PX", 12))
    abs_err = abs(err)

    vy = 0.0
    if abs_err > deadband:
        vy = -err * float(getattr(config, "HYBRID_OPENART_VY_PER_PX", getattr(config, "OPENART_TRACK_VY_PER_PX", 0.18)))
        limit = float(getattr(config, "HYBRID_OPENART_MAX_VY", getattr(config, "OPENART_TRACK_MAX_VY", 24)))
        vy = clamp(vy, -limit, limit)

    if abs_err > center_first_px:
        vx = float(getattr(config, "HYBRID_OPENART_ALIGN_VX", 4))
    elif abs_err > forward_center_px:
        vx = float(getattr(config, "HYBRID_OPENART_CENTERING_VX", 18))
    elif pixels >= float(getattr(config, "HYBRID_OPENART_PUSH_PIXELS", getattr(config, "OPENART_TRACK_PUSH_PIXELS", 2200))) and abs_err <= push_center_px:
        vx = float(getattr(config, "HYBRID_OPENART_PUSH_SPEED", getattr(config, "OPENART_TRACK_PUSH_SPEED", 34)))
    else:
        vx = float(getattr(config, "HYBRID_OPENART_FORWARD_SPEED", getattr(config, "OPENART_TRACK_FORWARD_SPEED", 20)))
    return (vx, vy), "[OPENART] label={} cx={:.0f} err={:.0f} pixels={:.0f} | vx={:.0f} vy={:.0f}".format(label, cx, err, pixels, vx, vy)


def lidar_track_command(obj):
    distance = float(obj.get("distance_mm", 0))
    angle = float(obj.get("angle_deg", 0))
    size = float(obj.get("size", 0))

    max_distance = float(getattr(config, "LIDAR_TRACK_MAX_DISTANCE_MM", getattr(config, "LIDAR_MAX_DISTANCE_MM", 700)))
    if distance > max_distance:
        return None, "[SKIP] far D:{:.0f}mm A:{:.1f} S:{:.0f}".format(distance, angle, size)

    min_size = float(getattr(config, "LIDAR_TRACK_MIN_SIZE", 80))
    if size < min_size:
        return None, "[SKIP] weak D:{:.0f}mm A:{:.1f} S:{:.0f}".format(distance, angle, size)

    target_dist = float(getattr(config, "LIDAR_TRACK_TARGET_DISTANCE_MM", 280))
    dist_deadband = float(getattr(config, "LIDAR_TRACK_DISTANCE_DEADBAND_MM", 45))
    front_min = float(getattr(config, "LIDAR_FRONT_MIN_DEG", 25.0))
    front_max = float(getattr(config, "LIDAR_FRONT_MAX_DEG", 65.0))
    center_angle = float(getattr(config, "LIDAR_TRACK_CENTER_DEG", (front_min + front_max) * 0.5))
    angle_error = angle - center_angle
    angle_deadband = float(getattr(config, "LIDAR_TRACK_ANGLE_DEADBAND_DEG", 3.0))

    vx = 0.0
    vy = 0.0

    if distance > target_dist + dist_deadband:
        vx = float(getattr(config, "LIDAR_TRACK_VX", 18))
    elif distance < target_dist - dist_deadband:
        vx = float(getattr(config, "LIDAR_TRACK_REVERSE_VX", -8))

    if abs(angle_error) > angle_deadband:
        vy = -angle_error * float(getattr(config, "LIDAR_TRACK_VY_PER_DEG", 2.0))
        limit = float(getattr(config, "LIDAR_TRACK_MAX_VY", 18))
        vy = clamp(vy, -limit, limit)
        if abs(angle_error) > float(getattr(config, "LIDAR_TRACK_ALIGN_FIRST_DEG", 5.0)):
            vx *= float(getattr(config, "LIDAR_TRACK_ALIGN_VX_SCALE", 0.35))

    if abs(vx) <= 0 and abs(vy) <= 0:
        return (0.0, 0.0), "[HOLD] D:{:.0f}mm A:{:.1f} err:{:.1f} S:{:.0f}".format(distance, angle, angle_error, size)
    return (vx, vy), "[LIDAR] D:{:.0f}mm A:{:.1f} err:{:.1f} S:{:.0f} | vx={:.0f} vy={:.0f}".format(distance, angle, angle_error, size, vx, vy)


def search(chassis):
    vw = float(getattr(config, "HYBRID_SEARCH_VW", getattr(config, "LIDAR_TRACK_SEARCH_VW", 18)))
    direction = float(getattr(config, "HYBRID_SEARCH_DIRECTION", 1.0))
    chassis.move(0, 0, vw * direction)
    return vw * direction


def smooth_object(rx, obj):
    if not isinstance(obj, dict):
        return None
    last = getattr(rx, "filtered_track_obj", None)
    if not isinstance(last, dict):
        return dict(obj)

    alpha = float(getattr(config, "LIDAR_TRACK_FILTER_ALPHA", 0.35))
    alpha = clamp(alpha, 0.0, 1.0)
    out = dict(obj)
    for key in ("distance_mm", "angle_deg", "size"):
        try:
            out[key] = float(last.get(key, obj.get(key, 0))) * (1.0 - alpha) + float(obj.get(key, 0)) * alpha
        except Exception:
            pass
    return out


def update_yaw(imu, ahrs, last_ms):
    if imu is None or ahrs is None:
        return None, last_ms
    now = time.ticks_ms()
    dt = time.ticks_diff(now, last_ms) / 1000.0 if last_ms else float(getattr(config, "LIDAR_TRACK_LOOP_MS", 20)) / 1000.0
    try:
        data = imu.read()
    except Exception:
        return None, now
    gyro = data.get("gyro") if isinstance(data, dict) else None
    mag = data.get("mag") if isinstance(data, dict) else None
    gyro_z = gyro[2] if isinstance(gyro, (tuple, list)) and len(gyro) >= 3 else None
    return ahrs.update(gyro_z, mag, dt), now


def yaw_hold_vw(current_yaw, lock_yaw):
    if current_yaw is None or lock_yaw is None:
        return 0.0
    error = angle_error_deg(lock_yaw, current_yaw)
    vw = error * float(getattr(config, "LIDAR_TRACK_YAW_HOLD_P", 0.45))
    limit = float(getattr(config, "LIDAR_TRACK_YAW_HOLD_MAX_VW", 12))
    return clamp(vw, -limit, limit)


def main():
    gc.collect()
    print("[INIT] Starting OpenArt gate...")
    openart = build_openart()

    print("[INIT] Starting Lidar...")
    rx = build_lidar()
    gc.collect()

    print("[INIT] Starting Mahony AHRS...")
    imu = build_imu()
    ahrs = MahonyYawAHRS(
        kp=getattr(config, "LIDAR_TRACK_MAHONY_KP", 1.4),
        ki=getattr(config, "LIDAR_TRACK_MAHONY_KI", 0.02),
    ) if imu is not None else None

    print("[INIT] Starting Chassis...")
    chassis = build_chassis()
    gc.collect()

    chassis.cfg.CHASSIS_MAX_VX_STEP = int(getattr(config, "LIDAR_TRACK_MAX_VX_STEP", 45))
    chassis.cfg.CHASSIS_MAX_VY_STEP = int(getattr(config, "LIDAR_TRACK_MAX_VY_STEP", 45))
    chassis.cfg.CHASSIS_MAX_VW_STEP = int(getattr(config, "LIDAR_TRACK_MAX_VW_STEP", 35))

    wait_c14_start()
    print("[RUN] Hybrid tracking started. OpenArt first, Lidar fallback.")

    last_print = 0
    last_imu_ms = 0
    yaw_lock = None
    stop_key = RuntimeStopKey()
    try:
        while True:
            if stop_key.should_stop():
                print("[STOP] C14 held, exit tracking loop.")
                break

            now = time.ticks_ms()
            yaw, last_imu_ms = update_yaw(imu, ahrs, last_imu_ms)
            result = openart.get_latest(timeout_ms=int(getattr(config, "HYBRID_OPENART_TIMEOUT_MS", 180))) if openart is not None else None
            cmd = None
            message = None
            source = None

            if valid_openart_result(result):
                cmd, message = openart_track_command(result)
                source = "openart"
            elif rx is not None:
                frame = rx.read_frame()
                obj = frame.get("object") if isinstance(frame, dict) else None

                if obj is None:
                    last_obj = getattr(rx, "last_track_obj", None)
                    last_ms = int(getattr(rx, "last_track_ms", 0) or 0)
                    hold_ms = int(getattr(config, "LIDAR_TRACK_HOLD_MS", getattr(config, "LIDAR_HOLD_MS", 120)))
                    if isinstance(last_obj, dict) and time.ticks_diff(now, last_ms) <= hold_ms:
                        obj = last_obj

                if obj:
                    obj = smooth_object(rx, obj)
                    rx.filtered_track_obj = obj
                    rx.last_track_obj = obj
                    rx.last_track_ms = now
                    cmd, message = lidar_track_command(obj)
                    source = "lidar"
            else:
                message = "[SEARCH] no openart target, lidar disabled"

            if cmd is not None:
                if yaw_lock is None and yaw is not None:
                    yaw_lock = yaw
                if source == "openart" and not bool(getattr(config, "HYBRID_OPENART_YAW_HOLD_ENABLE", False)):
                    vw = 0.0
                    if yaw is not None:
                        yaw_lock = yaw
                else:
                    vw = yaw_hold_vw(yaw, yaw_lock)
                if time.ticks_diff(now, last_print) > 200:
                    yaw_text = "" if yaw is None else " yaw={:.1f} vw={:.1f}".format(yaw, vw)
                    print(message + yaw_text)
                    last_print = now
                chassis.move(cmd[0], cmd[1], vw)
            else:
                yaw_lock = None
                if time.ticks_diff(now, last_print) > 300:
                    if message:
                        print(message)
                    else:
                        print("[SEARCH] sweep for nearest target")
                    last_print = now
                search(chassis)

            time.sleep_ms(int(getattr(config, "LIDAR_TRACK_LOOP_MS", 10)))

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("Error:", e)
    finally:
        chassis.stop()
        print("Chassis stopped.")


if __name__ == "__main__":
    main()
