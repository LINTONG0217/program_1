"""Push forward while locking yaw to the current 0-degree heading."""

import time
from machine import Pin, unique_id
from seekfree import IMU963RA, MOTOR_CONTROLLER

from Module import config


FORWARD_DUTIES = [3600, -3600, 3600, -3600]
PUSH_MS = 2500
TARGET_YAW_DEG = 0.0
GYRO_RAW_TO_DPS = 0.07
YAW_KP = 115.0
YAW_KD = 18.0
YAW_SIGN = float(getattr(config, "HEADING_LOCK_YAW_SIGN", 1.0))
YAW_MAX_DUTY = 1450
YAW_MIN_DUTY = 220
YAW_DEADBAND_DEG = 0.7


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
    print("press C14 to start cardinal push")
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
        duty = duties[i]
        if reverses[i]:
            duty = -duty
        motors[i].duty(int(clamp(duty, -10000, 10000)))


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
    print("script version: manual_push_cardinal_v2")
    motors = build_motors()
    imu = IMU963RA()
    wait_c14_start()
    yaw = 0.0
    last_ms = time.ticks_ms()
    start = last_ms
    last_print = last_ms
    try:
        while time.ticks_diff(time.ticks_ms(), start) < PUSH_MS:
            now = time.ticks_ms()
            dt = max(1, time.ticks_diff(now, last_ms)) / 1000.0
            last_ms = now
            gz, raw = gyro_z(imu)
            yaw = (yaw + gz * dt) % 360.0
            err = angle_error(TARGET_YAW_DEG, yaw)
            rot = 0
            if abs(err) > YAW_DEADBAND_DEG:
                rot = err * YAW_KP * YAW_SIGN + gz * YAW_KD
                if abs(rot) < YAW_MIN_DUTY:
                    rot = YAW_MIN_DUTY if rot > 0 else -YAW_MIN_DUTY
            rot = int(clamp(rot, -YAW_MAX_DUTY, YAW_MAX_DUTY))
            duties = [FORWARD_DUTIES[0] + rot, FORWARD_DUTIES[1] + rot, FORWARD_DUTIES[2] + rot, FORWARD_DUTIES[3] + rot]
            set_duties(motors, duties)
            if time.ticks_diff(now, last_print) >= 200:
                last_print = now
                print("yaw={:.2f} err={:.2f} gz={:.2f} rot={} raw_gz={} duties={}".format(yaw, err, gz, rot, raw[5] if raw and len(raw) > 5 else None, duties))
            time.sleep_ms(20)
    finally:
        stop_motors(motors)
        print("cardinal push done")


if __name__ == "__main__":
    main()
