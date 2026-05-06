"""Center navigation + tennis-ball push-out test.

Flow:
- Press C14 once.
- Move from (0, 0) to center probe point: x=0.70m, y=1.30m.
- Search/track the tennis ball using OpenART UART frames.
- Push along +X while holding heading 0 until estimated x >= 2.40m.
"""

import time
from machine import Pin, unique_id
from seekfree import IMU963RA, MOTOR_CONTROLLER

from BSP.board_runtime import MainControl
from Module import config
from Module.uart_vision_receiver import VisionReceiver


NAMES = ("fl", "fr", "bl", "br")

CENTER_FORWARD_M = 0.70
CENTER_RIGHT_M = 1.30
FIELD_EXIT_X_M = 2.40
PPM = 3325.625168175775

# Forward and strafe can have different effective PULSE_PER_METER on mecanum.
# - Forward uses config.PULSE_PER_METER (if present)
# - Right uses an optional independent calibration (or a scale factor)
PPM_FORWARD = float(getattr(config, "PULSE_PER_METER", PPM) or PPM)
PPM_RIGHT_SCALE = float(getattr(config, "PULSE_PER_METER_RIGHT_SCALE", 1.0) or 1.0)
PPM_RIGHT = float(getattr(config, "PULSE_PER_METER_RIGHT", PPM_FORWARD * PPM_RIGHT_SCALE) or (PPM_FORWARD * PPM_RIGHT_SCALE))

FORWARD_DUTIES = [6000, -6000, 6000, -6000]
RIGHT_DUTIES = [-3000, -3000, 3000, 3000]
BACK_DUTIES = [-d for d in FORWARD_DUTIES]
LEFT_DUTIES = [-d for d in RIGHT_DUTIES]
DUTY_RAMP_STEP = 240
PRINT_MS = 250

# Encoder move safety
# Disable time limit by default so long strafe can finish.
# Set >0 if you need a hard safety cutoff.
ENCODER_MOVE_TIMEOUT_MS = 0
ENCODER_STUCK_TIMEOUT_MS = 900

# Encoder count semantics differ across firmwares:
# - Some return absolute count since boot
# - Some return delta pulses since last read
# This script defaults to an auto mode that starts in "delta" (like the
# calibration script) and switches to "absolute" once counts become large.
ENCODER_COUNT_MODE = "auto"  # auto / delta / absolute
ENCODER_AUTO_ABSOLUTE_THRESHOLD_PULSE = 800  # avg_abs_pulse(raw_count) >= this => treat as absolute

RIGHT_LOCK_MAX_DUTY = 320
RIGHT_LOCK_STEP = 30
RIGHT_LOCK_FILTER_ALPHA = 0.15
RIGHT_LOCK_SIGN = 1.0
RIGHT_LOCK_KP = 6.0
RIGHT_LOCK_DEADBAND_PULSE = 2.0

SEARCH_ROTATE_DUTY = 1200
SEARCH_SWEEP_MS = 900
TRACK_MIN_FORWARD = 1400
TRACK_MAX_FORWARD = 3200
TRACK_SIZE_KP = 32
TRACK_STRAFE_KP = 8
TRACK_STRAFE_MAX = 700
CENTER_DEADBAND_PX = 10
PUSH_ENTER_SIZE = 80
PUSH_DUTY = 4200
PUSH_STRAFE_KP = 10
PUSH_STRAFE_MAX = 1200
TARGET_LOST_STOP_MS = 450

GYRO_RAW_TO_DPS = 0.07
YAW_TARGET_DEG = 0.0
YAW_KP = 95.0
YAW_KD = 12.0
YAW_MAX_DUTY = 1200
YAW_STEP_LIMIT = 180
YAW_FILTER_ALPHA = 0.35
YAW_DEADBAND_DEG = 1.0
YAW_SIGN = float(getattr(config, "HEADING_LOCK_YAW_SIGN", 1.0))


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
    print("press {} to start center tennis-ball pushout".format(pin_name))
    while key.value() != 0:
        time.sleep_ms(20)
    time.sleep_ms(60)
    while key.value() == 0:
        time.sleep_ms(20)
    print("{} start".format(pin_name))


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


def stop_motors(motors):
    for motor in motors:
        motor.duty(0)


def limit_step(target, current, limit):
    delta = target - current
    if delta > limit:
        return current + limit
    if delta < -limit:
        return current - limit
    return target


def ramp_duties(targets, currents, step):
    return [int(limit_step(int(targets[i]), int(currents[i]), int(step))) for i in range(4)]


def mix_duties(vx, vy, rot):
    return [
        int(clamp(vx - vy + rot, -10000, 10000)),
        int(clamp(-vx - vy + rot, -10000, 10000)),
        int(clamp(vx + vy + rot, -10000, 10000)),
        int(clamp(-vx + vy + rot, -10000, 10000)),
    ]


def snapshot_counts(board):
    data = board.read_encoder_snapshot()
    return {name: int(data[name]["count"]) for name in NAMES}


def snapshot_available(board):
    data = board.read_encoder_snapshot()
    try:
        return {name: bool(data[name].get("available", False)) for name in NAMES}
    except Exception:
        return {name: False for name in NAMES}


def delta_counts(curr, last):
    return {name: int(curr[name]) - int(last[name]) for name in NAMES}


class EncoderTotals:
    def __init__(self, board, mode=ENCODER_COUNT_MODE):
        self.board = board
        self.mode = str(mode or "auto").lower()
        self.active_mode = "delta" if self.mode == "auto" else self.mode
        self.initial_counts = None
        self.totals = {name: 0 for name in NAMES}
        self._switched = False

    def reset(self):
        self.initial_counts = snapshot_counts(self.board)
        self.totals = {name: 0 for name in NAMES}
        self.active_mode = "delta" if self.mode == "auto" else self.mode
        self._switched = False

    def update(self):
        if self.initial_counts is None:
            self.reset()
            return self.totals

        curr = snapshot_counts(self.board)

        if self.mode == "auto" and self.active_mode == "delta":
            if avg_abs_pulse(curr) >= ENCODER_AUTO_ABSOLUTE_THRESHOLD_PULSE:
                # Switch to absolute mode and re-baseline to avoid blowing up.
                self.active_mode = "absolute"
                self._switched = True

        if self.active_mode in ("absolute", "abs"):
            # Absolute mode: compute total movement from the initial baseline.
            self.totals = delta_counts(curr, self.initial_counts)
        else:
            # Treat raw count as delta pulse (reference calibration script style).
            for name in NAMES:
                self.totals[name] += int(curr[name])

        return self.totals


def avg_abs_pulse(delta):
    return sum(abs(int(delta[name])) for name in NAMES) * 0.25


def right_pulse(delta):
    return (-delta["fl"] + delta["fr"] + delta["bl"] - delta["br"]) * 0.25


def rotate_pulse(delta):
    return (delta["fl"] + delta["fr"] + delta["bl"] + delta["br"]) * 0.25


def encoder_right_lock_rotate(heading_pulse, last_rotate):
    if abs(heading_pulse) <= RIGHT_LOCK_DEADBAND_PULSE:
        target = 0.0
    else:
        target = float(heading_pulse) * RIGHT_LOCK_KP * RIGHT_LOCK_SIGN
    target = clamp(target, -RIGHT_LOCK_MAX_DUTY, RIGHT_LOCK_MAX_DUTY)
    filtered = last_rotate * (1.0 - RIGHT_LOCK_FILTER_ALPHA) + target * RIGHT_LOCK_FILTER_ALPHA
    duty = limit_step(filtered, last_rotate, RIGHT_LOCK_STEP)
    if abs(duty) < 1:
        return 0
    return int(clamp(duty, -RIGHT_LOCK_MAX_DUTY, RIGHT_LOCK_MAX_DUTY))


def run_encoder_move(board, motors, label, duties, target_m, axis_fn, lock_right_heading=False):
    counter = EncoderTotals(board, ENCODER_COUNT_MODE)
    counter.reset()
    current = [0, 0, 0, 0]
    rotate = 0
    ppm = PPM_FORWARD
    if label in ("CENTER_Y", "RETURN_Y"):
        ppm = PPM_RIGHT
    target_pulse = float(target_m) * ppm
    last_print = time.ticks_ms()
    start_ms = last_print
    last_progress_ms = last_print
    last_progress_pulses = 0.0
    avail = snapshot_available(board)
    if not any(bool(avail.get(name, False)) for name in NAMES):
        stop_motors(motors)
        print("{} ABORT: encoders unavailable: {}".format(label, avail))
        return None
    print("{} begin target_m={} target_pulse={:.0f} duties={} ppm={:.2f}".format(label, target_m, target_pulse, duties, ppm))
    try:
        while True:
            totals = counter.update()
            axis = axis_fn(totals)
            run_duties = duties
            if lock_right_heading:
                rotate = encoder_right_lock_rotate(rotate_pulse(totals), rotate)
                run_duties = [duties[0] + rotate, duties[1] + rotate, duties[2] + rotate, duties[3] + rotate]
            current = ramp_duties(run_duties, current, DUTY_RAMP_STEP)
            set_duties(motors, current)

            now = time.ticks_ms()
            if time.ticks_diff(now, last_print) >= PRINT_MS:
                last_print = now
                print(
                    "{} est={:.3f} axis={:.0f} rot={} duties={} total={}".format(
                        label,
                        abs(axis) / ppm,
                        axis,
                        rotate,
                        current,
                        totals,
                    )
                )

                if getattr(counter, "_switched", False):
                    # Print once after auto switch for field debugging.
                    print("{} encoder mode switched to ABSOLUTE".format(label))
                    counter._switched = False

            # progress / stuck detection
            progress_pulses = avg_abs_pulse(totals)
            if progress_pulses > last_progress_pulses + 1:
                last_progress_pulses = progress_pulses
                last_progress_ms = now
            if ENCODER_MOVE_TIMEOUT_MS > 0 and time.ticks_diff(now, start_ms) >= int(ENCODER_MOVE_TIMEOUT_MS):
                stop_motors(motors)
                print("{} TIMEOUT: no reach target est={:.3f} axis={:.0f} target_pulse={:.0f} total={}".format(label, abs(axis) / ppm, axis, target_pulse, totals))
                return None
            if ENCODER_STUCK_TIMEOUT_MS > 0 and time.ticks_diff(now, last_progress_ms) >= int(ENCODER_STUCK_TIMEOUT_MS):
                stop_motors(motors)
                print("{} STUCK: encoder no progress".format(label))
                return None

            if abs(axis) >= target_pulse:
                stop_motors(motors)
                print("{} STOP target reached est={:.3f} total={}".format(label, abs(axis) / ppm, totals))
                return totals
            time.sleep_ms(20)
    finally:
        stop_motors(motors)


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


def angle_error(target, current):
    err = float(target) - float(current)
    while err > 180.0:
        err -= 360.0
    while err < -180.0:
        err += 360.0
    return err


def yaw_lock_rotate(yaw, gz, last_rotate):
    err = angle_error(YAW_TARGET_DEG, yaw)
    if abs(err) <= YAW_DEADBAND_DEG:
        target = 0.0
    else:
        target = (err * YAW_KP + gz * YAW_KD) * YAW_SIGN
    target = clamp(target, -YAW_MAX_DUTY, YAW_MAX_DUTY)
    filtered = last_rotate * (1.0 - YAW_FILTER_ALPHA) + target * YAW_FILTER_ALPHA
    duty = limit_step(filtered, last_rotate, YAW_STEP_LIMIT)
    if abs(duty) < 1:
        return 0, err
    return int(clamp(duty, -YAW_MAX_DUTY, YAW_MAX_DUTY)), err


def target_is_tennis_ball(target):
    if not target or bool(target.get("held", False)):
        return False
    method = target.get("method")
    if method is not None and method in ("blue_blob", "blue_square", "square"):
        return False
    return True


def build_vision():
    return VisionReceiver(
        config.VISION_UART_ID,
        config.VISION_BAUDRATE,
        config.VISION_TX_PIN,
        config.VISION_RX_PIN,
        config.FRAME_WIDTH,
        config.FRAME_HEIGHT,
        debug_print=bool(getattr(config, "VISION_DEBUG_PRINT_RX", False)),
        label="ball_push",
    )


def main():
    print("board uid:", unique_id())
    print("script version: manual_center_tennis_ball_pushout_v1")
    print("flow: (0,0) -> x={:.2f}, y={:.2f}; push tennis ball until x>={:.2f}".format(CENTER_FORWARD_M, CENTER_RIGHT_M, FIELD_EXIT_X_M))
    print("ppm_forward={:.2f} ppm_right={:.2f} (scale={:.3f})".format(PPM_FORWARD, PPM_RIGHT, PPM_RIGHT_SCALE))
    board = MainControl(config)
    motors = build_motors()
    imu = IMU963RA()
    vision = build_vision()
    wait_c14_start()

    yaw = 0.0
    last_ms = time.ticks_ms()
    rotate = 0
    state = "center_forward"
    pose_x = 0.0
    pose_y = 0.0
    push_totals = {name: 0 for name in NAMES}
    push_counter = None
    current_duties = [0, 0, 0, 0]
    push_last_counts = None
    last_seen_ms = time.ticks_ms()
    last_print = time.ticks_ms()
    sweep_dir = 1
    sweep_start = time.ticks_ms()

    try:
        if run_encoder_move(board, motors, "CENTER_X", FORWARD_DUTIES, CENTER_FORWARD_M, avg_abs_pulse, False) is None:
            print("CENTER_X failed (timeout/stuck). Stop.")
            return
        pose_x = CENTER_FORWARD_M
        time.sleep_ms(250)
        if run_encoder_move(board, motors, "CENTER_Y", RIGHT_DUTIES, CENTER_RIGHT_M, right_pulse, True) is None:
            print("CENTER_Y failed (timeout/stuck).")
            print("Hint: mecanum strafe often needs a different ppm.")
            print("  Option A (preferred): run tests/hardware/manual_encoder_calibrate_long_xy.py and set config.PULSE_PER_METER_RIGHT")
            print("  Option B: keep PPM_RIGHT=PULSE_PER_METER, but increase ENCODER_MOVE_TIMEOUT_MS or raise RIGHT_DUTIES")
            print("           then use the measured distance to compute: new_ppm_right = old_ppm_right * target_m / actual_m")
            return
        pose_y = CENTER_RIGHT_M
        time.sleep_ms(250)
        print("center reached pose=({:.2f},{:.2f}); start vision search".format(pose_x, pose_y))
        state = "search"

        while True:
            now = time.ticks_ms()
            dt = max(1, time.ticks_diff(now, last_ms)) / 1000.0
            last_ms = now
            gz, raw = gyro_z(imu)
            yaw = (yaw + gz * dt) % 360.0
            rotate, yaw_err = yaw_lock_rotate(yaw, gz, rotate)

            frame = vision.get_latest_frame(timeout_ms=getattr(config, "FRAME_TIMEOUT_MS", 500))
            target = frame.get("object") if frame else None
            if not target_is_tennis_ball(target):
                target = None
            else:
                last_seen_ms = now

            if state == "search":
                if target:
                    state = "track"
                    stop_motors(motors)
                    current_duties = [0, 0, 0, 0]
                    print("state -> track target=", target)
                else:
                    if time.ticks_diff(now, sweep_start) >= SEARCH_SWEEP_MS:
                        sweep_start = now
                        sweep_dir = -sweep_dir
                    run = mix_duties(0, 0, SEARCH_ROTATE_DUTY * sweep_dir)
                    current_duties = ramp_duties(run, current_duties, DUTY_RAMP_STEP)
                    set_duties(motors, current_duties)

            elif state == "track":
                if not target:
                    stop_motors(motors)
                    current_duties = [0, 0, 0, 0]
                    if time.ticks_diff(now, last_seen_ms) > TARGET_LOST_STOP_MS:
                        state = "search"
                        print("state -> search")
                else:
                    offset = int(target.get("offset_x", 0))
                    size = int(target.get("size", 0))
                    if size >= PUSH_ENTER_SIZE and abs(offset) <= CENTER_DEADBAND_PX * 2:
                        state = "push"
                        push_totals = {name: 0 for name in NAMES}
                        push_counter = EncoderTotals(board, ENCODER_COUNT_MODE)
                        push_counter.reset()
                        push_last_counts = None
                        stop_motors(motors)
                        current_duties = [0, 0, 0, 0]
                        print("state -> push target=", target)
                    else:
                        vx = clamp((PUSH_ENTER_SIZE - size) * TRACK_SIZE_KP, TRACK_MIN_FORWARD, TRACK_MAX_FORWARD)
                        vy = 0 if abs(offset) <= CENTER_DEADBAND_PX else clamp(offset * TRACK_STRAFE_KP, -TRACK_STRAFE_MAX, TRACK_STRAFE_MAX)
                        run = mix_duties(vx, vy, rotate)
                        current_duties = ramp_duties(run, current_duties, DUTY_RAMP_STEP)
                        set_duties(motors, current_duties)

            elif state == "push":
                if push_counter is None:
                    push_counter = EncoderTotals(board, ENCODER_COUNT_MODE)
                    push_counter.reset()
                push_totals = push_counter.update()
                push_x = avg_abs_pulse(push_totals) / PPM_FORWARD
                pose_x = CENTER_FORWARD_M + push_x
                offset = int(target.get("offset_x", 0)) if target else 0
                vy = 0 if abs(offset) <= CENTER_DEADBAND_PX else clamp(offset * PUSH_STRAFE_KP, -PUSH_STRAFE_MAX, PUSH_STRAFE_MAX)
                run = mix_duties(PUSH_DUTY, vy, rotate)
                current_duties = ramp_duties(run, current_duties, DUTY_RAMP_STEP)
                set_duties(motors, current_duties)
                if pose_x >= FIELD_EXIT_X_M:
                    stop_motors(motors)
                    print("push complete pose_x={:.3f} push_x={:.3f} target={} frame_out={}".format(pose_x, push_x, target, bool(frame and frame.get("object_out_of_field", False))))
                    break

            if time.ticks_diff(now, last_print) >= PRINT_MS:
                last_print = now
                print(
                    "state={} pose=({:.2f},{:.2f}) yaw={:.1f} err={:.1f} rot={} target={} rx={} out={}".format(
                        state,
                        pose_x,
                        pose_y,
                        yaw,
                        yaw_err,
                        rotate,
                        target,
                        vision.rx_stat().get("ok"),
                        bool(frame and frame.get("object_out_of_field", False)),
                    )
                )
            time.sleep_ms(20)
    finally:
        stop_motors(motors)
        print("manual center tennis-ball pushout stopped")

    # After push-out: return to the same center probe point
    try:
        time.sleep_ms(250)
        print("return to center probe...")
        run_encoder_move(board, motors, "RETURN_Y", LEFT_DUTIES, CENTER_RIGHT_M, right_pulse, True)
        time.sleep_ms(250)
        run_encoder_move(board, motors, "RETURN_X", BACK_DUTIES, CENTER_FORWARD_M, avg_abs_pulse, False)
        print("return done")
    except Exception as e:
        stop_motors(motors)
        print("return failed:", repr(e))


if __name__ == "__main__":
    main()
