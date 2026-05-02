"""Turn to 0/90/180/270 degree headings using raw motors and IMU."""

import time
from machine import Pin, unique_id
from seekfree import IMU963RA, MOTOR_CONTROLLER

from Module import config


TARGETS = (90.0, 180.0, 270.0, 0.0)
ROTATE_MIN_DUTY = 550
ROTATE_MAX_DUTY = 1800
YAW_KP = 45.0
YAW_KD = 12.0
YAW_SIGN = -1.0
TOLERANCE_DEG = 2.0
STABLE_MS = 350
GYRO_RAW_TO_DPS = 0.07


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
    print("press C14 to start cardinal turns")
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


def set_rotate(motors, duty):
    duty = int(clamp(duty, -ROTATE_MAX_DUTY, ROTATE_MAX_DUTY))
    reverses = [
        getattr(config, "MOTOR_REVERSE", {}).get("fl", False),
        getattr(config, "MOTOR_REVERSE", {}).get("fr", True),
        getattr(config, "MOTOR_REVERSE", {}).get("bl", False),
        getattr(config, "MOTOR_REVERSE", {}).get("br", True),
    ]
    for i, m in enumerate(motors):
        real_duty = -duty if reverses[i] else duty
        m.duty(real_duty)


def stop_motors(motors):
    for m in motors:
        m.duty(0)


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


def turn_to(motors, imu, yaw, target):
    print("turn target:", target)
    last_ms = time.ticks_ms()
    stable_since = None
    last_print = last_ms
    start_time = last_ms
    TIMEOUT_MS = 8000
    last_raw_gz = None
    stuck_count = 0
    while True:
        now = time.ticks_ms()
        dt = max(1, time.ticks_diff(now, last_ms)) / 1000.0
        last_ms = now
        gz, raw = gyro_z(imu)
        yaw = (yaw + gz * dt) % 360.0
        err = angle_error(target, yaw)
        duty = (err * YAW_KP * YAW_SIGN) + (gz * YAW_KD)
        if abs(err) > TOLERANCE_DEG and abs(duty) < ROTATE_MIN_DUTY:
            duty = ROTATE_MIN_DUTY if duty > 0 else -ROTATE_MIN_DUTY
        if abs(err) <= TOLERANCE_DEG:
            duty = 0
            if stable_since is None:
                stable_since = now
            elif time.ticks_diff(now, stable_since) >= STABLE_MS:
                set_rotate(motors, 0)
                print("target reached yaw={:.2f}".format(yaw))
                return 0.0
        else:
            stable_since = None
        set_rotate(motors, duty)
        # 死数据检测
        raw_gz = raw[5] if raw and len(raw) > 5 else None
        if raw_gz == last_raw_gz:
            stuck_count += 1
        else:
            stuck_count = 0
        last_raw_gz = raw_gz
        if stuck_count > 100:  # 约2秒没变
            set_rotate(motors, 0)
            print("IMU数据卡死，已强制停止 yaw={:.2f}".format(yaw))
            return 0.0
        if time.ticks_diff(now, last_print) >= 200:
            last_print = now
            print("yaw={:.2f} target={:.1f} err={:.2f} gz={:.2f} duty={} raw_gz={}".format(
                yaw, target, err, gz, int(duty), raw_gz))
        # 超时保护，防止死循环
        if time.ticks_diff(now, start_time) > TIMEOUT_MS:
            set_rotate(motors, 0)
            print("timeout, force stop at yaw={:.2f}".format(yaw))
            return 0.0
        time.sleep_ms(20)


def main():
    print("board uid:", unique_id())
    print("script version: manual_cardinal_turns_v1")
    motors = build_motors()
    imu = IMU963RA()
    wait_c14_start()
    yaw = 0.0
    print("initial heading set to 0 deg")
    try:
        for target in TARGETS:
            yaw = turn_to(motors, imu, yaw, target)
            # 每次到达目标后重置yaw，防止积分漂移
            time.sleep_ms(600)
    finally:
        stop_motors(motors)
        print("cardinal turns done")


if __name__ == "__main__":
    main()
