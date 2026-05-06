"""One-object cycle smoke test: search -> approach -> push with yaw lock."""

import time
from machine import Pin, unique_id
from seekfree import IMU963RA, MOTOR_CONTROLLER

from Module import config
from Module.uart_vision_receiver import VisionReceiver


SEARCH_ROTATE_DUTY = 1600
FORWARD_DUTY = 2600
PUSH_DUTY = 3400
STRAFE_KP = 9.0
STRAFE_MAX = 1500
APPROACH_STOP_SIZE = 250
PUSH_MS = 2200
GYRO_RAW_TO_DPS = 0.07
YAW_KP = 110.0
YAW_KD = 18.0
YAW_SIGN = float(getattr(config, "HEADING_LOCK_YAW_SIGN", 1.0))
YAW_MAX_DUTY = 1300
YAW_DEADBAND_DEG = 0.8


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
    key = Pin(getattr(config, "NAV_KEY_PIN", "C14"), Pin.IN)
    print("press C14 to start one-object cycle")
    while key.value() != 0:
        time.sleep_ms(20)
    time.sleep_ms(60)
    while key.value() == 0:
        time.sleep_ms(20)


def build_motors():
    return [
        MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5, 13000),
        MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D6_DIR_D7, 13000),
        MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000),
        MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000),
    ]


def stop_motors(motors):
    for m in motors:
        m.duty(0)


def set_duties(motors, duties):
    reverses = [
        getattr(config, "MOTOR_REVERSE", {}).get("fl", False),
        getattr(config, "MOTOR_REVERSE", {}).get("fr", True),
        getattr(config, "MOTOR_REVERSE", {}).get("bl", False),
        getattr(config, "MOTOR_REVERSE", {}).get("br", True),
    ]
    for i in range(4):
        duty = -duties[i] if reverses[i] else duties[i]
        motors[i].duty(int(clamp(duty, -10000, 10000)))


def drive(motors, forward, strafe, rotate):
    drive_strafe = -strafe
    duties = [
        forward + drive_strafe + rotate,
        -forward + drive_strafe + rotate,
        forward - drive_strafe + rotate,
        -forward - drive_strafe + rotate,
    ]
    set_duties(motors, duties)
    return duties


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


def main():
    print("board uid:", unique_id())
    print("script version: manual_one_object_cycle_v3")
    motors = build_motors()
    imu = IMU963RA()
    vision = VisionReceiver(config.VISION_UART_ID, config.VISION_BAUDRATE, config.VISION_TX_PIN, config.VISION_RX_PIN, config.FRAME_WIDTH, config.FRAME_HEIGHT)
    wait_c14_start()
    state = "search"
    yaw = 0.0
    last_ms = time.ticks_ms()
    state_ms = last_ms
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

            if state == "search":
                drive(motors, 0, 0, SEARCH_ROTATE_DUTY)
                if target:
                    stop_motors(motors)
                    state = "approach"
                    state_ms = now
                    print("state -> approach", target)
            elif state == "approach":
                if not target:
                    stop_motors(motors)
                    state = "search"
                    state_ms = now
                    print("state -> search")
                else:
                    offset = int(target.get("offset_x", 0))
                    size = int(target.get("size", 0))
                    if size >= APPROACH_STOP_SIZE:
                        stop_motors(motors)
                        yaw = 0.0
                        state = "push"
                        state_ms = now
                        print("state -> push, heading reset to 0")
                    else:
                        strafe = clamp(offset * STRAFE_KP, -STRAFE_MAX, STRAFE_MAX)
                        drive(motors, FORWARD_DUTY, strafe, 0)
            elif state == "push":
                err = angle_error(0.0, yaw)
                rot = 0
                if abs(err) > YAW_DEADBAND_DEG:
                    rot = int(clamp(err * YAW_KP * YAW_SIGN + gz * YAW_KD, -YAW_MAX_DUTY, YAW_MAX_DUTY))
                drive(motors, PUSH_DUTY, 0, rot)
                if time.ticks_diff(now, state_ms) >= PUSH_MS:
                    stop_motors(motors)
                    print("one-object cycle done")
                    break

            if time.ticks_diff(now, last_print) >= 250:
                last_print = now
                print("state={} yaw={:.2f} gz={:.2f} target={} raw_gz={}".format(state, yaw, gz, target, raw[5] if raw and len(raw) > 5 else None))
            time.sleep_ms(20)
    finally:
        stop_motors(motors)
        print("one-object cycle stopped")


if __name__ == "__main__":
    main()
