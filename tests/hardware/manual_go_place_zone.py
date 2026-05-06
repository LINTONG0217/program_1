"""Rough raw-motor test for moving from start area to the object placing zone.

Assumption for this smoke test:
- The car starts near the lower-left start area.
- The car's initial heading is treated as field +X / yaw 0.
- It drives forward for X distance, then strafes left/right for Y distance.
"""

import time
from machine import Pin, unique_id
from seekfree import IMU963RA, MOTOR_CONTROLLER

from Module import config

try:
    from BSP.board_runtime import MainControl
except Exception as e:
    MainControl = None
    MAINCONTROL_IMPORT_ERROR = e


FORWARD_DUTIES = [3200, -3200, 3200, -3200]
STRAFE_DUTIES = [-3000, -3000, 3000, 3000]
TARGET_X_M = 1.00
TARGET_Y_M = 1.50
PRINT_MS = 250


def wait_c14_start():
    if not bool(getattr(config, "TEST_WAIT_C14_ENABLE", True)):
        return
    key = Pin(getattr(config, "NAV_KEY_PIN", "C14"), Pin.IN)
    print("press C14 to start go-place-zone test")
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


def set_duties(motors, duties):
    reverses = [
        getattr(config, "MOTOR_REVERSE", {}).get("fl", False),
        getattr(config, "MOTOR_REVERSE", {}).get("fr", True),
        getattr(config, "MOTOR_REVERSE", {}).get("bl", False),
        getattr(config, "MOTOR_REVERSE", {}).get("br", True),
    ]
    for i in range(4):
        duty = -duties[i] if reverses[i] else duties[i]
        motors[i].duty(int(duty))


def stop_motors(motors):
    for m in motors:
        m.duty(0)


def snapshot(board):
    return board.read_encoder_snapshot()


def drive_until_distance(board, motors, duties, target_m, label):
    names = ("fl", "fr", "bl", "br")
    abs_totals = {name: 0 for name in names}
    ppm = float(getattr(config, "PULSE_PER_METER", 1.0) or 1.0)
    last_print = time.ticks_ms()
    print(label, "target_m=", target_m, "duties=", duties)
    while True:
        data = snapshot(board)
        for name in names:
            abs_totals[name] += abs(int(data[name]["count"]))
        dist_m = (sum(abs_totals.values()) / 4.0) / ppm
        set_duties(motors, duties)
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print) >= PRINT_MS:
            last_print = now
            print(label, "dist={:.3f} abs={}".format(dist_m, abs_totals))
        if dist_m >= target_m:
            break
        time.sleep_ms(20)
    stop_motors(motors)
    time.sleep_ms(400)


def main():
    print("board uid:", unique_id())
    print("script version: manual_go_place_zone_v3")
    if MainControl is None:
        print("MainControl import failed:", repr(MAINCONTROL_IMPORT_ERROR))
        print("Please upload/sync BSP/board_runtime.py and BSP/device_adapters.py to the board first.")
        return
    board = MainControl(config)
    motors = build_motors()
    _imu = IMU963RA()
    wait_c14_start()
    try:
        drive_until_distance(board, motors, FORWARD_DUTIES, TARGET_X_M, "go X")
        drive_until_distance(board, motors, STRAFE_DUTIES, TARGET_Y_M, "go Y")
        print("place-zone smoke move done")
    finally:
        stop_motors(motors)


if __name__ == "__main__":
    main()
