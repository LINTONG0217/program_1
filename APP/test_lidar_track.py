import gc
import time

from machine import Pin

from BSP.motor_actuator import Motor
from BSP.omni_chassis_driver import OmniChassis
from Module import config
from Module.ydlidar_receiver import YDLidarReceiver


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


def build_chassis():
    motor_fl = Motor(*config.MOTOR_FL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fl", False))
    motor_fr = Motor(*config.MOTOR_FR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fr", True))
    motor_bl = Motor(*config.MOTOR_BL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("bl", False))
    motor_br = Motor(*config.MOTOR_BR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("br", True))
    return OmniChassis(motor_fl, motor_fr, motor_bl, motor_br, config=config)


def build_lidar():
    return YDLidarReceiver(
        uart_id=getattr(config, "LIDAR_UART_ID", 5),
        baudrate=getattr(config, "LIDAR_BAUDRATE", 230400),
        tx_pin=getattr(config, "LIDAR_TX_PIN", 4),
        rx_pin=getattr(config, "LIDAR_RX_PIN", 5),
        config=config,
    )


def track_command(obj):
    distance = float(obj.get("distance_mm", 0))
    angle = float(obj.get("angle_deg", 0))
    size = float(obj.get("size", 0))

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
            vx = 0.0

    if abs(vx) <= 0 and abs(vy) <= 0:
        return (0.0, 0.0), "[HOLD] D:{:.0f}mm A:{:.1f} err:{:.1f} S:{:.0f}".format(distance, angle, angle_error, size)
    return (vx, vy), "[TRACK] D:{:.0f}mm A:{:.1f} err:{:.1f} S:{:.0f} | vx={:.0f} vy={:.0f}".format(distance, angle, angle_error, size, vx, vy)


def search(chassis):
    chassis.move(0, 0, float(getattr(config, "LIDAR_TRACK_SEARCH_VW", 18)))


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


def main():
    gc.collect()
    print("[INIT] Starting Lidar...")
    rx = build_lidar()

    print("[INIT] Starting Chassis...")
    chassis = build_chassis()
    gc.collect()

    chassis.cfg.CHASSIS_MAX_VX_STEP = 30
    chassis.cfg.CHASSIS_MAX_VY_STEP = 30
    chassis.cfg.CHASSIS_MAX_VW_STEP = 30

    wait_c14_start()
    print("[RUN] Tracking started! Put the ball in front of the Lidar.")

    last_print = 0
    try:
        while True:
            now = time.ticks_ms()
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
                cmd, message = track_command(obj)
                if time.ticks_diff(now, last_print) > 200:
                    print(message)
                    last_print = now
                if cmd is None:
                    search(chassis)
                else:
                    chassis.move(cmd[0], cmd[1], 0)
            else:
                if time.ticks_diff(now, last_print) > 300:
                    print("[SEARCH] no target")
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
