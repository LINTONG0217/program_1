"""OpenART ball tracking and push-out field test.

Flow:
- Search for the ball using OpenART UART frames.
- Track/approach the ball until it is close enough.
- Push forward with IMU yaw lock until OpenART reports object_out_of_field.
"""

import time
from machine import Pin, unique_id
from seekfree import IMU963RA

from BSP.motor_actuator import Motor
from BSP.omni_chassis_driver import OmniChassis
from Module import config
from Module.uart_vision_receiver import VisionReceiver


SEARCH_SPEED = 16
TRACK_MIN_SPEED = 10
TRACK_MAX_SPEED = 32
TRACK_KP_SIZE = 0.14
TRACK_STRAFE_KP = 0.18
TRACK_STRAFE_MAX = 24
CENTER_DEADBAND_PX = 8
PUSH_ENTER_SIZE = 135
PUSH_SPEED = 42
PUSH_STRAFE_KP = 0.10
PUSH_STRAFE_MAX = 12
PUSH_MAX_MS = 4500
PUSH_OUT_CONFIRM_FRAMES = 2
TARGET_LOST_STOP_MS = 450
GYRO_RAW_TO_DPS = 0.07
YAW_KP = 1.4
YAW_KD = 0.12
YAW_MAX_SPEED = 24
YAW_DEADBAND_DEG = 1.0
PRINT_MS = 200


def clamp(value, low, high):
    return max(low, min(high, value))


def angle_error(target, current):
    err = float(target) - float(current)
    while err > 180.0:
        err -= 360.0
    while err < -180.0:
        err += 360.0
    return err


def wait_c14_start():
    if not bool(getattr(config, "TEST_WAIT_C14_ENABLE", True)):
        return
    pin_name = getattr(config, "NAV_KEY_PIN", "C14")
    key = Pin(pin_name, Pin.IN)
    pull_up = getattr(Pin, "PULL_UP_47K", None)
    if pull_up is not None:
        try:
            key = Pin(pin_name, Pin.IN, pull_up)
        except Exception:
            pass
    print("press {} to start OpenART track/push-out test".format(pin_name))
    while key.value() != 0:
        time.sleep_ms(20)
    time.sleep_ms(60)
    while key.value() == 0:
        time.sleep_ms(20)
    print("{} start".format(pin_name))


def build_chassis():
    motor_fl = Motor(*config.MOTOR_FL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fl", False))
    motor_fr = Motor(*config.MOTOR_FR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fr", False))
    motor_bl = Motor(*config.MOTOR_BL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("bl", False))
    motor_br = Motor(*config.MOTOR_BR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("br", False))
    chassis = OmniChassis(motor_fl, motor_fr, motor_bl, motor_br, config=config)
    cfg = getattr(chassis, "cfg", None)
    if cfg:
        cfg.CHASSIS_CMD_DEADBAND = 0
        cfg.CHASSIS_MAX_VX_STEP = 6
        cfg.CHASSIS_MAX_VY_STEP = 6
        cfg.CHASSIS_MAX_VW_STEP = 8
        cfg.CHASSIS_ENABLE_CLOSED_LOOP = False
    chassis.attach_feedback(encoders=None, imu=None)
    return chassis


def imu_raw(imu):
    try:
        imu.read()
    except Exception:
        pass
    try:
        return imu.get()
    except Exception:
        return None


def gyro_z(imu):
    raw = imu_raw(imu)
    if raw and len(raw) > 5:
        return float(raw[5]) * GYRO_RAW_TO_DPS, raw
    return 0.0, raw


def yaw_lock_cmd(target_yaw, yaw, gz):
    err = angle_error(target_yaw, yaw)
    if abs(err) <= YAW_DEADBAND_DEG:
        return 0.0, err
    sign = float(getattr(config, "HEADING_LOCK_YAW_SIGN", 1.0))
    vw = (err * YAW_KP + gz * YAW_KD) * sign
    return clamp(vw, -YAW_MAX_SPEED, YAW_MAX_SPEED), err


def target_out(frame, target):
    if not target:
        return False
    if frame and bool(frame.get("object_out_of_field", False)):
        return True
    w = int(frame.get("frame", {}).get("w", config.FRAME_WIDTH)) if frame else config.FRAME_WIDTH
    h = int(frame.get("frame", {}).get("h", config.FRAME_HEIGHT)) if frame else config.FRAME_HEIGHT
    x = int(target.get("x", w // 2))
    y = int(target.get("y", h // 2))
    margin = int(getattr(config, "OBJECT_OUT_MARGIN_PX", 14))
    return x <= margin or x >= w - margin or y <= margin or y >= h - margin


def main():
    print("board uid:", unique_id())
    print("script version: manual_openart_track_push_out_v1")
    print(
        "vision uart id={} baud={} tx={} rx={}".format(
            config.VISION_UART_ID,
            config.VISION_BAUDRATE,
            config.VISION_TX_PIN,
            config.VISION_RX_PIN,
        )
    )

    chassis = build_chassis()
    imu = IMU963RA()
    vision = VisionReceiver(
        config.VISION_UART_ID,
        config.VISION_BAUDRATE,
        config.VISION_TX_PIN,
        config.VISION_RX_PIN,
        config.FRAME_WIDTH,
        config.FRAME_HEIGHT,
        debug_print=bool(getattr(config, "VISION_DEBUG_PRINT_RX", False)),
        label="pushout",
    )

    wait_c14_start()
    state = "search"
    yaw = 0.0
    yaw_lock = 0.0
    push_start_ms = None
    out_count = 0
    last_seen_ms = time.ticks_ms()
    last_ms = time.ticks_ms()
    last_print = last_ms

    try:
        while True:
            now = time.ticks_ms()
            dt = max(1, time.ticks_diff(now, last_ms)) / 1000.0
            last_ms = now
            gz, raw = gyro_z(imu)
            yaw = (yaw + gz * dt) % 360.0

            frame = vision.get_latest_frame(timeout_ms=getattr(config, "FRAME_TIMEOUT_MS", 500))
            target = frame.get("object") if frame else None
            if target and bool(target.get("held", False)) and state != "push":
                target = None
            if target:
                last_seen_ms = now

            if state == "search":
                if target:
                    chassis.stop()
                    yaw_lock = yaw
                    state = "track"
                    print("state -> track", target)
                else:
                    chassis.move(0, 0, SEARCH_SPEED)

            elif state == "track":
                if not target:
                    if time.ticks_diff(now, last_seen_ms) > TARGET_LOST_STOP_MS:
                        state = "search"
                        print("state -> search")
                    chassis.stop()
                else:
                    offset = int(target.get("offset_x", 0))
                    size = int(target.get("size", 0))
                    if size >= PUSH_ENTER_SIZE and abs(offset) <= CENTER_DEADBAND_PX * 3:
                        state = "push"
                        yaw_lock = yaw
                        push_start_ms = now
                        out_count = 0
                        print("state -> push", target)
                    else:
                        vx = clamp((PUSH_ENTER_SIZE - size) * TRACK_KP_SIZE, TRACK_MIN_SPEED, TRACK_MAX_SPEED)
                        vy = 0.0 if abs(offset) <= CENTER_DEADBAND_PX else clamp(offset * TRACK_STRAFE_KP, -TRACK_STRAFE_MAX, TRACK_STRAFE_MAX)
                        vw, _ = yaw_lock_cmd(yaw_lock, yaw, gz)
                        chassis.move(vx, vy, vw)

            elif state == "push":
                if target_out(frame, target):
                    out_count += 1
                else:
                    out_count = 0

                if out_count >= PUSH_OUT_CONFIRM_FRAMES:
                    chassis.stop()
                    print("push-out success", "target=", target, "frame=", frame)
                    break

                if push_start_ms is not None and time.ticks_diff(now, push_start_ms) >= PUSH_MAX_MS:
                    chassis.stop()
                    print("push timeout", "target=", target, "frame=", frame)
                    break

                offset = int(target.get("offset_x", 0)) if target else 0
                vy = 0.0 if abs(offset) <= CENTER_DEADBAND_PX else clamp(offset * PUSH_STRAFE_KP, -PUSH_STRAFE_MAX, PUSH_STRAFE_MAX)
                vw, _ = yaw_lock_cmd(yaw_lock, yaw, gz)
                chassis.move(PUSH_SPEED, vy, vw)

            if time.ticks_diff(now, last_print) >= PRINT_MS:
                last_print = now
                stat = vision.rx_stat()
                print(
                    "state={} yaw={:.1f} gz={:.1f} target={} out={} rx_ok={} crc_fail={} raw_gz={}".format(
                        state,
                        yaw,
                        gz,
                        target,
                        bool(frame and frame.get("object_out_of_field", False)),
                        stat.get("ok"),
                        stat.get("crc_fail"),
                        raw[5] if raw and len(raw) > 5 else None,
                    )
                )
            time.sleep_ms(20)
    finally:
        chassis.stop()
        print("OpenART track/push-out test done")


if __name__ == "__main__":
    main()
