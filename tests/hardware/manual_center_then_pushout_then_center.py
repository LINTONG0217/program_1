"""Center -> push-out -> center (hardware test).

Goal (as requested):
- Go to field center first (requires pose estimation from encoders+IMU).
- Then use OpenART vision frames to find the ball and push it out of the field.
- After push-out success/timeout, go back to the field center.

Notes:
- Uses VisionReceiver UART frames: SP|json|CRC (same as vision.py output).
- Center navigation uses PoseEstimator (odometry). If pose is unavailable, it will stop and keep retrying.
"""

import math
import time

from machine import unique_id

from BSP.board_runtime import MainControl
from BSP.motor_actuator import Motor
from BSP.omni_chassis_driver import OmniChassis
from Module import config
from Module.pose_estimation import PoseEstimator
from Module.uart_vision_receiver import VisionReceiver


# ========= Push-out behavior (mostly copied from manual_openart_track_push_out.py) =========
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
PRINT_MS = 250


# ========= Center navigation =========
CENTER_NAV_TIMEOUT_MS = 12000
CENTER_NAV_STABLE_MS = 300
CENTER_NAV_TOL_M = 0.10
CENTER_YAW_DEADBAND_DEG = 1.5
CENTER_YAW_KP = 1.2
CENTER_YAW_MAX_SPEED = 24.0
CENTER_NAV_KP = 90.0
CENTER_NAV_MAX_SPEED = 38.0
CENTER_NAV_MIN_SPEED = 10.0
CENTER_NAV_AXIS_SEQUENCE_ENABLE = True


def clamp(value, low, high):
    return max(low, min(high, value))


def angle_error(target, current):
    err = float(target) - float(current)
    while err > 180.0:
        err -= 360.0
    while err < -180.0:
        err += 360.0
    return err


def world_to_body(vx_world, vy_world, yaw_deg):
    rad = math.radians(float(yaw_deg))
    cos_a = math.cos(rad)
    sin_a = math.sin(rad)
    return (
        vx_world * cos_a + vy_world * sin_a,
        -vx_world * sin_a + vy_world * cos_a,
    )


def heading_lock_vw(target_yaw, current_yaw, cfg=config):
    deadband = float(getattr(cfg, "CENTER_YAW_DEADBAND_DEG", CENTER_YAW_DEADBAND_DEG))
    err = angle_error(target_yaw, current_yaw)
    if abs(err) <= deadband:
        return 0.0, err
    kp = float(getattr(cfg, "CENTER_YAW_KP", CENTER_YAW_KP))
    limit = float(getattr(cfg, "CENTER_YAW_MAX_SPEED", CENTER_YAW_MAX_SPEED))
    sign = float(getattr(cfg, "HEADING_LOCK_YAW_SIGN", 1.0))
    return clamp(err * kp * sign, -limit, limit), err


def field_center(cfg=config):
    fx = float(getattr(cfg, "FIELD_SIZE_X_M", 3.2))
    fy = float(getattr(cfg, "FIELD_SIZE_Y_M", 2.8))
    return fx * 0.5, fy * 0.5


def center_target(cfg=config):
    x = getattr(cfg, "CENTER_TARGET_X", None)
    y = getattr(cfg, "CENTER_TARGET_Y", None)
    if x is None or y is None:
        return field_center(cfg)
    return float(x), float(y)


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
    return chassis


def target_out(frame, target):
    if not target:
        return False
    if frame and bool(frame.get("object_out_of_field", False)):
        return True
    w = int(frame.get("frame", {}).get("w", 320)) if frame else 320
    h = int(frame.get("frame", {}).get("h", 240)) if frame else 240
    x = int(target.get("x", w // 2))
    y = int(target.get("y", h // 2))
    margin = int(getattr(config, "OBJECT_OUT_MARGIN_PX", 14))
    return x <= margin or x >= w - margin or y <= margin or y >= h - margin


def go_center(chassis, estimator, board, cfg=config, label="center"):
    """Blocking go-center loop. Returns True if reached, False if timeout."""
    tx, ty = center_target(cfg)
    start_ms = time.ticks_ms()
    stable_start = None
    yaw_lock = None

    while True:
        now = time.ticks_ms()
        # Update pose
        imu_data = board.read_imu() if board else {"gyro": None, "mag": None}
        encoder_data = board.read_encoder_speeds() if board else {}
        pose = estimator.update(encoder_data, imu_data)

        if not isinstance(pose, dict):
            chassis.stop()
            time.sleep_ms(20)
            continue

        x = float(pose.get("x", 0.0))
        y = float(pose.get("y", 0.0))
        yaw = float(pose.get("yaw", 0.0))

        if yaw_lock is None:
            yaw_lock = yaw

        ex = tx - x
        ey = ty - y
        dist = math.sqrt(ex * ex + ey * ey)

        vw, yaw_err = heading_lock_vw(yaw_lock, yaw, cfg)
        pos_tol = float(getattr(cfg, "CENTER_NAV_TOLERANCE_M", CENTER_NAV_TOL_M))
        yaw_tol = float(getattr(cfg, "CENTER_YAW_TOLERANCE_DEG", 5.0))

        if dist <= pos_tol and abs(yaw_err) <= yaw_tol:
            if stable_start is None:
                stable_start = now
            stable_ms = int(getattr(cfg, "CENTER_NAV_STABLE_MS", CENTER_NAV_STABLE_MS))
            if time.ticks_diff(now, stable_start) >= stable_ms:
                chassis.stop()
                print(
                    "go center({}): reached".format(label),
                    "x={:.2f}".format(x),
                    "y={:.2f}".format(y),
                    "yaw={:.1f}".format(yaw),
                )
                return True
        else:
            stable_start = None

        timeout_ms = int(getattr(cfg, "CENTER_NAV_TIMEOUT_MS", CENTER_NAV_TIMEOUT_MS))
        if timeout_ms > 0 and time.ticks_diff(now, start_ms) >= timeout_ms:
            chassis.stop()
            print(
                "go center({}): timeout".format(label),
                "x={:.2f}".format(x),
                "y={:.2f}".format(y),
                "target=({:.2f},{:.2f})".format(tx, ty),
            )
            return False

        kp = float(getattr(cfg, "CENTER_NAV_KP", CENTER_NAV_KP))
        max_speed = float(getattr(cfg, "CENTER_NAV_MAX_SPEED", CENTER_NAV_MAX_SPEED))
        min_speed = float(getattr(cfg, "CENTER_NAV_MIN_SPEED", CENTER_NAV_MIN_SPEED))

        vx_world = clamp(ex * kp, -max_speed, max_speed)
        vy_world = clamp(ey * kp, -max_speed, max_speed)

        if bool(getattr(cfg, "CENTER_NAV_AXIS_SEQUENCE_ENABLE", CENTER_NAV_AXIS_SEQUENCE_ENABLE)) and dist > pos_tol:
            if abs(ex) > pos_tol:
                vy_world = 0.0
            else:
                vx_world = 0.0

        if dist > pos_tol:
            if abs(vx_world) > 0 and abs(vx_world) < min_speed:
                vx_world = min_speed if vx_world > 0 else -min_speed
            if abs(vy_world) > 0 and abs(vy_world) < min_speed:
                vy_world = min_speed if vy_world > 0 else -min_speed
        else:
            vx_world = 0.0
            vy_world = 0.0

        vx_body, vy_body = world_to_body(vx_world, vy_world, yaw)
        chassis.move(vx_body, vy_body, vw)
        time.sleep_ms(20)


def main():
    print("board uid:", unique_id())
    print("script version: manual_center_then_pushout_then_center_v1")
    print(
        "vision uart id={} baud={} tx={} rx={}".format(
            config.VISION_UART_ID,
            config.VISION_BAUDRATE,
            config.VISION_TX_PIN,
            config.VISION_RX_PIN,
        )
    )

    board = MainControl(config)
    board.init()

    chassis = build_chassis()
    chassis.attach_feedback(board.encoders, board.imu)

    estimator = PoseEstimator(config)

    vision = VisionReceiver(
        config.VISION_UART_ID,
        config.VISION_BAUDRATE,
        config.VISION_TX_PIN,
        config.VISION_RX_PIN,
        config.FRAME_WIDTH,
        config.FRAME_HEIGHT,
        debug_print=bool(getattr(config, "VISION_DEBUG_PRINT_RX", False)),
        label="center_pushout",
    )

    # C14 gate (use board key)
    if bool(getattr(config, "TEST_WAIT_C14_ENABLE", True)):
        pin_name = getattr(config, "NAV_KEY_PIN", "C14")
        print("press {} to start center->pushout->center test".format(pin_name))
        while not board.nav_key.is_pressed():
            time.sleep_ms(20)
        time.sleep_ms(60)
        while board.nav_key.is_pressed():
            time.sleep_ms(20)
        print("{} start".format(pin_name))

    # 1) Go to center first
    go_center(chassis, estimator, board, config, label="pre")

    # 2) Search/track/push-out
    state = "search"
    push_start_ms = None
    out_count = 0
    last_seen_ms = time.ticks_ms()
    last_print = last_seen_ms
    yaw_lock = None

    try:
        while True:
            now = time.ticks_ms()

            # Keep pose updated so yaw lock works.
            imu_data = board.read_imu()
            encoder_data = board.read_encoder_speeds()
            pose = estimator.update(encoder_data, imu_data)
            yaw = float(pose.get("yaw", 0.0)) if isinstance(pose, dict) else 0.0

            frame = vision.get_latest_frame(timeout_ms=int(getattr(config, "FRAME_TIMEOUT_MS", 500)))
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

                    if yaw_lock is None:
                        yaw_lock = yaw
                    vw, _ = heading_lock_vw(yaw_lock, yaw, config)

                    if size >= PUSH_ENTER_SIZE and abs(offset) <= CENTER_DEADBAND_PX * 3:
                        state = "push"
                        push_start_ms = now
                        out_count = 0
                        print("state -> push", target)
                    else:
                        vx = clamp((PUSH_ENTER_SIZE - size) * TRACK_KP_SIZE, TRACK_MIN_SPEED, TRACK_MAX_SPEED)
                        vy = 0.0 if abs(offset) <= CENTER_DEADBAND_PX else clamp(offset * TRACK_STRAFE_KP, -TRACK_STRAFE_MAX, TRACK_STRAFE_MAX)
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
                if yaw_lock is None:
                    yaw_lock = yaw
                vw, _ = heading_lock_vw(yaw_lock, yaw, config)

                vy = 0.0 if abs(offset) <= CENTER_DEADBAND_PX else clamp(offset * PUSH_STRAFE_KP, -PUSH_STRAFE_MAX, PUSH_STRAFE_MAX)
                chassis.move(PUSH_SPEED, vy, vw)

            if time.ticks_diff(now, last_print) >= PRINT_MS:
                last_print = now
                stat = vision.rx_stat()
                print(
                    "state={} yaw={:.1f} target={} out={} rx_ok={} crc_fail={} parse_fail={}".format(
                        state,
                        yaw,
                        target,
                        bool(frame and frame.get("object_out_of_field", False)),
                        stat.get("ok"),
                        stat.get("crc_fail"),
                        stat.get("parse_fail"),
                    )
                )

            time.sleep_ms(20)

    finally:
        chassis.stop()

    # 3) Return to center
    go_center(chassis, estimator, board, config, label="post")
    chassis.stop()
    print("center->pushout->center test done")


if __name__ == "__main__":
    main()
