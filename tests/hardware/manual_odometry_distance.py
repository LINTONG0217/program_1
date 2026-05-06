"""Raw forward distance test using encoder pulse accumulation.

Run:
    python -m mpremote run tests/hardware/manual_odometry_distance.py

Press C14, then the car drives forward while printing per-wheel accumulated
pulses and estimated distance. Stop with Ctrl+C.
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


FORWARD_DUTIES = [3200, -3200, 3200, -3200]
TARGET_DISTANCE_M = 0.50
MAX_RUN_MS = 6000
PRINT_MS = 200


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
    print("press {} to start odometry distance test".format(pin_name))
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
        motors[i].duty(int(duty))


def snapshot(board):
    return board.read_encoder_snapshot()


def main():
    print("board uid:", unique_id())
    print("script version: manual_odometry_distance_v2")
    if MainControl is None:
        print("MainControl import failed:", repr(MAINCONTROL_IMPORT_ERROR))
        print("Please upload/sync BSP/board_runtime.py and BSP/device_adapters.py to the board first.")
        return
    board = MainControl(config)
    motors = build_motors()
    wait_c14_start()

    names = ("fl", "fr", "bl", "br")
    totals = {name: 0 for name in names}
    abs_totals = {name: 0 for name in names}
    ppm = float(getattr(config, "PULSE_PER_METER", 1.0) or 1.0)
    start = time.ticks_ms()
    last_print = start

    try:
        while True:
            data = snapshot(board)
            for name in names:
                cnt = int(data[name]["count"])
                totals[name] += cnt
                abs_totals[name] += abs(cnt)

            avg_abs = sum(abs_totals.values()) / 4.0
            dist_m = avg_abs / ppm
            set_duties(motors, FORWARD_DUTIES)

            now = time.ticks_ms()
            if time.ticks_diff(now, last_print) >= PRINT_MS:
                last_print = now
                print(
                    "dist={:.3f}m avg_abs={:.0f} net={} abs={}".format(
                        dist_m,
                        avg_abs,
                        totals,
                        abs_totals,
                    )
                )

            if dist_m >= TARGET_DISTANCE_M:
                print("target distance reached")
                break
            if time.ticks_diff(now, start) >= MAX_RUN_MS:
                print("timeout")
                break
            time.sleep_ms(20)
    finally:
        stop_motors(motors)
        print("odometry distance test done")


if __name__ == "__main__":
    main()
