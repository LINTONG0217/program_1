"""Vision-guided approach test without pushing."""

import time
from machine import Pin, unique_id
from seekfree import MOTOR_CONTROLLER

from Module import config
from Module.uart_vision_receiver import VisionReceiver


FORWARD_BASE = 2600
STRAFE_KP = 10.0
STRAFE_MAX = 1800
APPROACH_STOP_SIZE = 260
CENTER_DEADBAND_PX = 10
PRINT_MS = 200


def clamp(value, low, high):
    return max(low, min(high, value))


def wait_c14_start():
    if not bool(getattr(config, "TEST_WAIT_C14_ENABLE", True)):
        return
    key = Pin(getattr(config, "NAV_KEY_PIN", "C14"), Pin.IN)
    print("press C14 to start approach test")
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


def drive(motors, forward, strafe):
    # Raw omni mix matching manual_chassis_raw.py polarity.
    fl = -forward - strafe
    fr = forward - strafe
    bl = -forward + strafe
    br = forward + strafe
    duties = [fl, fr, bl, br]
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
    return duties


def main():
    print("board uid:", unique_id())
    print("script version: manual_approach_object_v1")
    motors = build_motors()
    vision = VisionReceiver(config.VISION_UART_ID, config.VISION_BAUDRATE, config.VISION_TX_PIN, config.VISION_RX_PIN, config.FRAME_WIDTH, config.FRAME_HEIGHT)
    wait_c14_start()
    last_print = time.ticks_ms()
    try:
        while True:
            frame = vision.get_latest_frame(timeout_ms=getattr(config, "FRAME_TIMEOUT_MS", 500))
            target = frame.get("object") if frame else None
            if not target:
                stop_motors(motors)
                if time.ticks_diff(time.ticks_ms(), last_print) >= PRINT_MS:
                    last_print = time.ticks_ms()
                    print("no target")
                time.sleep_ms(20)
                continue

            offset = int(target.get("offset_x", 0))
            size = int(target.get("size", 0))
            if size >= APPROACH_STOP_SIZE and abs(offset) <= CENTER_DEADBAND_PX:
                stop_motors(motors)
                print("arrived target:", target)
                time.sleep_ms(300)
                continue

            forward = 0 if size >= APPROACH_STOP_SIZE else FORWARD_BASE
            strafe = 0 if abs(offset) <= CENTER_DEADBAND_PX else clamp(offset * STRAFE_KP, -STRAFE_MAX, STRAFE_MAX)
            duties = drive(motors, forward, strafe)
            now = time.ticks_ms()
            if time.ticks_diff(now, last_print) >= PRINT_MS:
                last_print = now
                print("target=", target, "forward=", forward, "strafe=", strafe, "duties=", duties)
            time.sleep_ms(20)
    finally:
        stop_motors(motors)
        print("approach test done")


if __name__ == "__main__":
    main()
