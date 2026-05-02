import time
from machine import Pin, unique_id
from seekfree import IMU963RA, MOTOR_CONTROLLER
from Module import config


# Raw chassis commands are intentionally kept identical to manual_chassis_raw.py.
FORWARD_DUTIES = [-3600, 3600, -3600, 3600]
BACKWARD_DUTIES = [3600, -3600, 3600, -3600]

# Heading-lock tuning. Rotation duty is added equally to all four raw motors,
# matching manual_chassis_raw.py rotate cw/ccw patterns.
FORWARD_ONLY_MS = 2000
TARGET_YAW_DEG = 0.0
GYRO_RAW_TO_DPS = 0.07
YAW_KP = 115.0
YAW_KD = 18.0
YAW_MIN_DUTY = 220
YAW_MAX_DUTY = 1450
YAW_DEADBAND_DEG = 0.7
YAW_SIGN = -1.0
GYRO_FILTER_ALPHA = 0.35
ROT_FILTER_ALPHA = 0.45
ROT_STEP_LIMIT = 260


def clamp(value, low, high):
    return max(low, min(high, value))


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

    print("press {} to start raw heading-lock test".format(pin_name))
    while key.value() != 0:
        time.sleep_ms(20)
    time.sleep_ms(60)
    while key.value() == 0:
        time.sleep_ms(20)
    print("{} start".format(pin_name))


def step(title, duties, duration_ms=1200):
    print("raw:", title, duties)
    motors[0].duty(duties[0])
    motors[1].duty(duties[1])
    motors[2].duty(duties[2])
    motors[3].duty(duties[3])
    time.sleep_ms(duration_ms)


def stop_motors():
    for motor in motors:
        motor.duty(0)


def add_rotate(forward_duties, rotate_duty):
    rot = int(clamp(rotate_duty, -YAW_MAX_DUTY, YAW_MAX_DUTY))
    return [
        int(clamp(forward_duties[0] + rot, -10000, 10000)),
        int(clamp(forward_duties[1] + rot, -10000, 10000)),
        int(clamp(forward_duties[2] + rot, -10000, 10000)),
        int(clamp(forward_duties[3] + rot, -10000, 10000)),
    ]


def set_duties(duties):
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
        motors[i].duty(duty)


def angle_error(target, current):
    err = float(target) - float(current)
    while err > 180.0:
        err -= 360.0
    while err < -180.0:
        err += 360.0
    return err


def limit_step(target, current, limit):
    delta = target - current
    if delta > limit:
        return current + limit
    if delta < -limit:
        return current - limit
    return target


def yaw_to_rotate_duty(err, gyro_z, last_rotate):
    if abs(err) <= YAW_DEADBAND_DEG:
        target = 0.0
    else:
        target = err * YAW_KP * YAW_SIGN + gyro_z * YAW_KD
        if abs(target) < YAW_MIN_DUTY:
            target = YAW_MIN_DUTY if target > 0 else -YAW_MIN_DUTY
    target = clamp(target, -YAW_MAX_DUTY, YAW_MAX_DUTY)
    filtered = last_rotate * (1.0 - ROT_FILTER_ALPHA) + target * ROT_FILTER_ALPHA
    duty = limit_step(filtered, last_rotate, ROT_STEP_LIMIT)
    if abs(duty) < 1:
        return 0
    return int(clamp(duty, -YAW_MAX_DUTY, YAW_MAX_DUTY))


def imu_read_raw(imu):
    try:
        imu.read()
    except Exception:
        pass
    try:
        return imu.get()
    except Exception:
        return None


def gyro_z_dps(imu):
    raw = imu_read_raw(imu)
    if raw and len(raw) > 5:
        return float(raw[5]) * GYRO_RAW_TO_DPS, raw
    return 0.0, raw


motor_fl = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5, 13000)
motor_fr = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D6_DIR_D7, 13000)
motor_bl = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000)
motor_br = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000)
motors = [motor_fl, motor_fr, motor_bl, motor_br]


print("board uid:", unique_id())
print("script version: raw_heading_lock_like_chassis_raw_v1")
print("raw motor channels ready")
wait_c14_start()

try:
    # This must behave exactly like manual_chassis_raw.py forward.
    step("forward probe", FORWARD_DUTIES, 1500)
    stop_motors()
    time.sleep_ms(500)

    imu = IMU963RA()
    print("imu ready")
    for i in range(3):
        print("imu sample{}:".format(i), imu_read_raw(imu))
        time.sleep_ms(50)

    print(
        "heading lock params:",
        "forward=", FORWARD_DUTIES,
        "forward_only_ms=", FORWARD_ONLY_MS,
        "kp=", YAW_KP,
        "kd=", YAW_KD,
        "min=", YAW_MIN_DUTY,
        "max=", YAW_MAX_DUTY,
        "deadband=", YAW_DEADBAND_DEG,
        "sign=", YAW_SIGN,
    )

    yaw = 0.0
    print("initial heading set to 0 deg")
    start_ms = time.ticks_ms()
    last_ms = start_ms
    last_print = start_ms
    filtered_gz = 0.0
    rotate = 0

    while True:
        now = time.ticks_ms()
        dt = max(1, time.ticks_diff(now, last_ms)) / 1000.0
        last_ms = now

        raw_gz, raw = gyro_z_dps(imu)
        filtered_gz = filtered_gz * (1.0 - GYRO_FILTER_ALPHA) + raw_gz * GYRO_FILTER_ALPHA
        yaw = (yaw + filtered_gz * dt) % 360.0
        err = angle_error(TARGET_YAW_DEG, yaw)

        if time.ticks_diff(now, start_ms) < FORWARD_ONLY_MS:
            mode = "FORWARD"
            rotate = 0
        else:
            mode = "LOCK"
            rotate = yaw_to_rotate_duty(err, filtered_gz, rotate)

        duties = add_rotate(FORWARD_DUTIES, rotate)
        set_duties(duties)

        if time.ticks_diff(now, last_print) >= 200:
            last_print = now
            print(
                "{} yaw={:.2f} err={:.2f} gz={:.2f} rot={} raw_gz={} duties={}".format(
                    mode,
                    yaw,
                    err,
                    filtered_gz,
                    rotate,
                    raw[5] if raw and len(raw) > 5 else None,
                    duties,
                )
            )
        time.sleep_ms(20)
finally:
    stop_motors()
    print("raw heading-lock test done")
