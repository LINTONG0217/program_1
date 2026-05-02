"""Rotate in place and print OpenArt vision frames."""

import time
from machine import Pin, unique_id
from seekfree import MOTOR_CONTROLLER

from Module import config
from Module.uart_vision_receiver import VisionReceiver


ROTATE_DUTY = 1800
PULSE_ROTATE_MS = 180
PULSE_PAUSE_MS = 140
PRINT_MS = 250


def wait_c14_start():
    if not bool(getattr(config, "TEST_WAIT_C14_ENABLE", True)):
        return
    key = Pin(getattr(config, "NAV_KEY_PIN", "C14"), Pin.IN)
    print("press C14 to start vision search")
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


def rotate(motors, duty):
    for m in motors:
        m.duty(int(duty))


def main():
    print("board uid:", unique_id())
    print("script version: manual_vision_search_v1")
    motors = build_motors()
    vision = VisionReceiver(
        config.VISION_UART_ID,
        config.VISION_BAUDRATE,
        config.VISION_TX_PIN,
        config.VISION_RX_PIN,
        frame_width=config.FRAME_WIDTH,
        frame_height=config.FRAME_HEIGHT,
        debug_print=False,
    )
    wait_c14_start()
    phase_until = time.ticks_ms()
    rotating = True
    last_print = time.ticks_ms()
    try:
        while True:
            now = time.ticks_ms()
            if time.ticks_diff(now, phase_until) >= 0:
                rotating = not rotating
                phase_until = time.ticks_add(now, PULSE_ROTATE_MS if rotating else PULSE_PAUSE_MS)
            rotate(motors, ROTATE_DUTY if rotating else 0)
            frame = vision.get_latest_frame(timeout_ms=getattr(config, "FRAME_TIMEOUT_MS", 500))
            if time.ticks_diff(now, last_print) >= PRINT_MS:
                last_print = now
                obj = frame.get("object") if frame else None
                zone = frame.get("zone") if frame else None
                print("frame=", bool(frame), "obj=", obj, "zone=", zone, "age=", vision.age_ms())
            if frame and frame.get("object"):
                stop_motors(motors)
                print("object found:", frame.get("object"))
                time.sleep_ms(500)
            time.sleep_ms(20)
    finally:
        stop_motors(motors)
        print("vision search done")


if __name__ == "__main__":
    main()
