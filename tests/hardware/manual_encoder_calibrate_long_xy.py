"""Encoder center-position test: forward, then right strafe.

Run:
    python -m mpremote run tests/hardware/manual_encoder_calibrate_long_xy.py

Usage:
- Place the car at a known mark, facing the +X direction.
- Press C14 once. The car drives forward by the current encoder estimate.
- It stops and prints pulses. Measure the actual forward distance.
- Press C14 again. The car strafes right by the current encoder estimate.
- Measure the actual right distance.

Calibration formula:
    new_pulse_per_meter = old_pulse_per_meter * target_distance / actual_distance
"""

import time
from machine import Pin, unique_id
from seekfree import MOTOR_CONTROLLER

from Module import config

try:
    from BSP.board_runtime import MainControl
except Exception as e:
    MainControl = None
    MAINCONTROL_IMPORT_ERROR = e


NAMES = ("fl", "fr", "bl", "br")

FORWARD_TARGET_M = 0.70
RIGHT_TARGET_M = 1.10

# Thonny/offline calibration safety: this script can use a local PPM value even
# if the board still has an old /Module/config.py.
CALIBRATION_PULSE_PER_METER = 3325.625168175775

FORWARD_DUTIES = [6000, -6000, 6000, -6000]
RIGHT_DUTIES = [-3000, -3000, 3000, 3000]

SERIAL_LOG_ENABLE = True
PRINT_MS = 1000
PAUSE_BETWEEN_SEGMENTS_MS = 500
MAX_FORWARD_MS = 0
MAX_RIGHT_MS = 0
ENCODER_NO_PROGRESS_STOP_MS = 2500
ENCODER_NO_PROGRESS_MIN_PULSES = 20
RIGHT_CHUNK_M = 0.0
WAIT_C14_BETWEEN_CHUNKS = True
DUTY_RAMP_STEP = 240

# Use a soft heading lock during right strafe. Encoder mode avoids blocking on
# IMU gyro calibration while still damping chassis rotation.
HEADING_LOCK_ENABLE = True
HEADING_LOCK_MODE = "encoder"  # encoder / imu
HEADING_LOCK_GYRO_CALIBRATE_SAMPLES = 60
HEADING_LOCK_YAW_KP = 18.0
HEADING_LOCK_YAW_KD = 1.5
HEADING_LOCK_YAW_MIN_DUTY = 0
HEADING_LOCK_YAW_MAX_DUTY = 320
HEADING_LOCK_YAW_DEADBAND_DEG = 2.0
HEADING_LOCK_GYRO_FILTER_ALPHA = 0.20
HEADING_LOCK_ROT_FILTER_ALPHA = 0.15
HEADING_LOCK_ROT_STEP_LIMIT = 30
# 从日志看右移时误差持续变大且补偿打满，说明方向反了：改回 +1.0
RIGHT_HEADING_LOCK_YAW_SIGN = 1.0
ENCODER_HEADING_LOCK_SIGN = 1.0
ENCODER_HEADING_LOCK_KP = 6.0
ENCODER_HEADING_LOCK_DEADBAND_PULSE = 2.0
FORWARD_HEADING_LOCK_ENABLE = False
RIGHT_HEADING_LOCK_ENABLE = True
RUN_FORWARD_SEGMENT = True
RUN_RIGHT_SEGMENT = True


def clamp(value, low, high):
    return max(low, min(high, value))


def log(*args):
    if not SERIAL_LOG_ENABLE:
        return
    try:
        print(*args)
    except Exception:
        pass


def wait_c14_start(label):
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

    log("press {} to {}".format(pin_name, label))
    while key.value() != 0:
        time.sleep_ms(20)
    time.sleep_ms(60)
    while key.value() == 0:
        time.sleep_ms(20)
    log("{} start: {}".format(pin_name, label))


def make_key():
    pin_name = getattr(config, "NAV_KEY_PIN", "C14")
    key = Pin(pin_name, Pin.IN)
    pull_up = getattr(Pin, "PULL_UP_47K", None)
    if pull_up is not None:
        try:
            key = Pin(pin_name, Pin.IN, pull_up)
        except Exception:
            pass
    return key


def build_motors():
    return [
        MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5, 13000),
        MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D6_DIR_D7, 13000),
        MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000),
        MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000),
    ]


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


def add_rotate(duties, rotate_duty):
    rot = int(clamp(rotate_duty, -HEADING_LOCK_YAW_MAX_DUTY, HEADING_LOCK_YAW_MAX_DUTY))
    return [
        int(clamp(duties[0] + rot, -10000, 10000)),
        int(clamp(duties[1] + rot, -10000, 10000)),
        int(clamp(duties[2] + rot, -10000, 10000)),
        int(clamp(duties[3] + rot, -10000, 10000)),
    ]


def stop_motors(motors):
    for motor in motors:
        motor.duty(0)


def snapshot_counts(board):
    data = board.read_encoder_snapshot()
    return {name: int(data[name]["count"]) for name in NAMES}


def forward_pulse(delta):
    return avg_abs_pulse(delta)


def right_pulse(delta):
    # Observed RIGHT strafe counts are fl-, fr+, bl+, br-.
    return (-delta["fl"] + delta["fr"] + delta["bl"] - delta["br"]) * 0.25


def rotate_pulse(delta):
    return (delta["fl"] + delta["fr"] + delta["bl"] + delta["br"]) * 0.25


def avg_abs_pulse(delta):
    return sum(abs(int(delta[name])) for name in NAMES) * 0.25


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


def ramp_duties(targets, currents, step):
    return [int(limit_step(int(targets[i]), int(currents[i]), int(step))) for i in range(4)]


def gyro_z_dps(board):
    data = board.read_imu()
    gyro = data.get("gyro") if data else None
    if gyro and len(gyro) >= 3:
        return float(gyro[2])
    return 0.0


def calibrate_gyro_z(board):
    samples = int(HEADING_LOCK_GYRO_CALIBRATE_SAMPLES)
    if samples <= 0:
        return 0.0
    total = 0.0
    for _ in range(10):
        gyro_z_dps(board)
        time.sleep_ms(5)
    for _ in range(samples):
        total += gyro_z_dps(board)
        time.sleep_ms(5)
    return total / samples


def yaw_to_rotate_duty(err, gyro_z, last_rotate, yaw_sign=None):
    sign = float(yaw_sign if yaw_sign is not None else getattr(config, "HEADING_LOCK_YAW_SIGN", 1.0))
    if abs(err) <= HEADING_LOCK_YAW_DEADBAND_DEG:
        target = 0.0
    else:
        target = (err * HEADING_LOCK_YAW_KP + gyro_z * HEADING_LOCK_YAW_KD) * sign
        if abs(target) < HEADING_LOCK_YAW_MIN_DUTY:
            target = HEADING_LOCK_YAW_MIN_DUTY if target > 0 else -HEADING_LOCK_YAW_MIN_DUTY
    target = clamp(target, -HEADING_LOCK_YAW_MAX_DUTY, HEADING_LOCK_YAW_MAX_DUTY)
    filtered = last_rotate * (1.0 - HEADING_LOCK_ROT_FILTER_ALPHA) + target * HEADING_LOCK_ROT_FILTER_ALPHA
    duty = limit_step(filtered, last_rotate, HEADING_LOCK_ROT_STEP_LIMIT)
    if abs(duty) < 1:
        return 0
    return int(clamp(duty, -HEADING_LOCK_YAW_MAX_DUTY, HEADING_LOCK_YAW_MAX_DUTY))


def encoder_heading_to_rotate_duty(heading_pulse, last_rotate):
    if abs(heading_pulse) <= ENCODER_HEADING_LOCK_DEADBAND_PULSE:
        target = 0.0
    else:
        target = float(heading_pulse) * ENCODER_HEADING_LOCK_KP * ENCODER_HEADING_LOCK_SIGN
    target = clamp(target, -HEADING_LOCK_YAW_MAX_DUTY, HEADING_LOCK_YAW_MAX_DUTY)
    filtered = last_rotate * (1.0 - HEADING_LOCK_ROT_FILTER_ALPHA) + target * HEADING_LOCK_ROT_FILTER_ALPHA
    duty = limit_step(filtered, last_rotate, HEADING_LOCK_ROT_STEP_LIMIT)
    if abs(duty) < 1:
        return 0
    return int(clamp(duty, -HEADING_LOCK_YAW_MAX_DUTY, HEADING_LOCK_YAW_MAX_DUTY))


def print_calibration_hint(label, ppm, target_m, estimated_m, axis_pulses, delta):
    abs_pulses = avg_abs_pulse(delta)
    log("{} done".format(label))
    log("  target_m={:.3f}".format(target_m))
    log("  old_pulse_per_meter={:.6f}".format(ppm))
    log("  stop_axis_est_m={:.3f}".format(estimated_m))
    log("  axis_pulses={:.1f} axis_est_m={:.3f}".format(axis_pulses, abs(axis_pulses) / ppm))
    log("  avg_abs_pulses={:.1f} avg_abs_est_m={:.3f}".format(abs_pulses, abs_pulses / ppm))
    log("  total_counts={}".format(delta))
    log("  after measuring actual distance, use:")
    log("  new_pulse_per_meter = {:.6f} * {:.3f} / actual_{}_m".format(ppm, target_m, label.lower()))


def active_pulse_per_meter():
    override = globals().get("CALIBRATION_PULSE_PER_METER", None)
    if override is not None:
        try:
            return float(override)
        except Exception:
            pass
    return float(getattr(config, "PULSE_PER_METER", 1.0) or 1.0)


def run_segment(
    board,
    motors,
    label,
    duties,
    target_m,
    axis_fn,
    max_ms,
    abort_key=None,
    heading_lock=False,
    yaw_sign_override=None,
    chunk_m=0.0,
    wait_between_chunks=False,
):
    ppm = active_pulse_per_meter()
    start_ms = time.ticks_ms()
    last_ms = start_ms
    last_print = start_ms
    estimated_m = 0.0
    totals = {name: 0 for name in NAMES}
    last_progress_ms = start_ms
    last_progress_pulses = 0.0
    yaw = 0.0
    gyro_offset = 0.0
    filtered_gz = 0.0
    rotate = 0
    current_duties = [0, 0, 0, 0]

    heading_mode = str(HEADING_LOCK_MODE).lower()

    if bool(heading_lock) and heading_mode == "imu":
        log("{} heading lock: calibrating gyro, keep car still...".format(label))
        gyro_offset = calibrate_gyro_z(board)
        log("{} heading lock: gyro_offset={:.3f} sign={}".format(label, gyro_offset, yaw_sign_override if yaw_sign_override is not None else getattr(config, "HEADING_LOCK_YAW_SIGN", 1.0)))
    elif bool(heading_lock) and heading_mode == "encoder":
        log("{} heading lock: encoder mode sign={} kp={}".format(label, ENCODER_HEADING_LOCK_SIGN, ENCODER_HEADING_LOCK_KP))
    else:
        log("{} heading lock: disabled".format(label))
    next_stop_m = target_m
    if chunk_m > 0 and chunk_m < target_m:
        next_stop_m = chunk_m
    next_stop_pulse = next_stop_m * ppm
    target_pulse = target_m * ppm
    log("{} begin target_m={} chunk_m={} duties={} ppm={:.6f}".format(label, target_m, chunk_m, duties, ppm))
    log("{} target_axis_pulse={:.0f} max_ms={}".format(label, target_pulse, max_ms))
    try:
        while True:
            now = time.ticks_ms()
            dt = max(1, time.ticks_diff(now, last_ms)) / 1000.0
            last_ms = now

            step_counts = snapshot_counts(board)
            for name in NAMES:
                totals[name] += int(step_counts[name])

            axis = axis_fn(totals)
            abs_pulses = avg_abs_pulse(totals)
            estimated_m = abs(axis) / ppm

            run_duties = duties
            if bool(heading_lock) and heading_mode == "imu":
                gz = gyro_z_dps(board) - gyro_offset
                filtered_gz = filtered_gz * (1.0 - HEADING_LOCK_GYRO_FILTER_ALPHA) + gz * HEADING_LOCK_GYRO_FILTER_ALPHA
                yaw = (yaw + filtered_gz * dt) % 360.0
                err = angle_error(0.0, yaw)
                rotate = yaw_to_rotate_duty(err, filtered_gz, rotate, yaw_sign_override)
                run_duties = add_rotate(duties, rotate)
            elif bool(heading_lock) and heading_mode == "encoder":
                yaw = rotate_pulse(totals)
                filtered_gz = 0.0
                rotate = encoder_heading_to_rotate_duty(yaw, rotate)
                run_duties = add_rotate(duties, rotate)

            current_duties = ramp_duties(run_duties, current_duties, DUTY_RAMP_STEP)
            set_duties(motors, current_duties)

            if abs_pulses > last_progress_pulses:
                last_progress_ms = now
                last_progress_pulses = abs_pulses

            if time.ticks_diff(now, last_print) >= PRINT_MS:
                last_print = now
                log(
                    "{} stop_axis_est={:.3f} avg_abs_est={:.3f} axis_pulse={:.0f} yaw={:.2f} gz={:.2f} rot={} duties={} step={} total={}".format(
                        label,
                        estimated_m,
                        abs_pulses / ppm,
                        axis,
                        yaw,
                        filtered_gz,
                        rotate,
                        current_duties,
                        step_counts,
                        totals,
                    )
                )

            if abs(axis) >= next_stop_pulse:
                stop_motors(motors)
                current_duties = [0, 0, 0, 0]
                if abs(axis) >= target_pulse:
                    log("{} STOP_REASON=ENCODER_TARGET_REACHED".format(label))
                    break
                log(
                    "{} CHUNK_REACHED stop_axis_est={:.3f}/{:.3f}; next_stop={:.3f}".format(
                        label,
                        estimated_m,
                        target_m,
                        min(target_m, estimated_m + chunk_m),
                    )
                )
                if wait_between_chunks:
                    wait_c14_start("continue {} calibration".format(label))
                next_stop_m = min(target_m, estimated_m + chunk_m)
                next_stop_pulse = next_stop_m * ppm
                start_ms = time.ticks_ms()
                last_ms = start_ms
                last_print = start_ms
                last_progress_ms = start_ms
                last_progress_pulses = abs_pulses
                continue
            if abort_key is not None and abort_key.value() == 0:
                log("{} STOP_REASON=C14_ABORT".format(label))
                time.sleep_ms(250)
                while abort_key.value() == 0:
                    time.sleep_ms(20)
                break
            if abs_pulses < ENCODER_NO_PROGRESS_MIN_PULSES and time.ticks_diff(now, start_ms) >= ENCODER_NO_PROGRESS_STOP_MS:
                log("{} STOP_REASON=ENCODER_NO_PROGRESS check encoder wiring/counts".format(label))
                break
            if abs_pulses >= ENCODER_NO_PROGRESS_MIN_PULSES and time.ticks_diff(now, last_progress_ms) >= ENCODER_NO_PROGRESS_STOP_MS:
                log("{} STOP_REASON=ENCODER_NO_PROGRESS check encoder wiring/counts".format(label))
                break
            if max_ms > 0 and time.ticks_diff(now, start_ms) >= max_ms:
                log("{} STOP_REASON=TIMEOUT max_ms={}".format(label, max_ms))
                break
            time.sleep_ms(20)
    finally:
        stop_motors(motors)

    print_calibration_hint(label, ppm, target_m, estimated_m, axis_fn(totals), totals)
    time.sleep_ms(PAUSE_BETWEEN_SEGMENTS_MS)


def main():
    log("board uid:", unique_id())
    log("script version: manual_encoder_center_forward_0p70_right_1p10_v12_right_lock_active")
    if MainControl is None:
        log("MainControl import failed:", repr(MAINCONTROL_IMPORT_ERROR))
        log("Please upload/sync BSP/board_runtime.py and BSP/device_adapters.py to the board first.")
        return

    board = MainControl(config)
    motors = build_motors()
    try:
        log("config PULSE_PER_METER:", getattr(config, "PULSE_PER_METER", None))
        log("active calibration PULSE_PER_METER:", active_pulse_per_meter())
        log("forward target = {:.2f}m; right target = {:.2f}m".format(FORWARD_TARGET_M, RIGHT_TARGET_M))
        log("right chunk = {:.2f}m; wait between chunks = {}".format(RIGHT_CHUNK_M, WAIT_C14_BETWEEN_CHUNKS))
        log("run forward segment:", RUN_FORWARD_SEGMENT)
        log("run right segment:", RUN_RIGHT_SEGMENT)

        if RUN_FORWARD_SEGMENT:
            wait_c14_start("start FORWARD calibration")
            key = make_key()
            run_segment(
                board,
                motors,
                "FORWARD",
                FORWARD_DUTIES,
                FORWARD_TARGET_M,
                forward_pulse,
                MAX_FORWARD_MS,
                key,
                FORWARD_HEADING_LOCK_ENABLE,
                RIGHT_HEADING_LOCK_YAW_SIGN,
                0.0,
                False,
            )

        if RUN_RIGHT_SEGMENT:
            log("Auto start RIGHT calibration after forward segment.")
            key = make_key()
            run_segment(
                board,
                motors,
                "RIGHT",
                RIGHT_DUTIES,
                RIGHT_TARGET_M,
                right_pulse,
                MAX_RIGHT_MS,
                key,
                RIGHT_HEADING_LOCK_ENABLE,
                RIGHT_HEADING_LOCK_YAW_SIGN,
                RIGHT_CHUNK_M,
                WAIT_C14_BETWEEN_CHUNKS,
            )

        log("calibration run complete")
        log("If forward and right produce different new PPM values, prefer the forward value for PULSE_PER_METER")
        log("and use the right result only as a hint for mecanum slip / lateral scale.")
    finally:
        stop_motors(motors)
        log("manual encoder calibration stopped")


if __name__ == "__main__":
    main()
