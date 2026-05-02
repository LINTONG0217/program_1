import time
from machine import Pin, unique_id
from seekfree import MOTOR_CONTROLLER
from Module import config


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

    print("press {} to start test".format(pin_name))
    while key.value() != 0:
        time.sleep_ms(20)
    time.sleep_ms(60)
    while key.value() == 0:
        time.sleep_ms(20)
    print("{} start".format(pin_name))


def step(title, duties, duration_ms=1200):
    print("raw:", title, duties)
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
    time.sleep_ms(duration_ms)


motor_fl = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5, 13000)
motor_fr = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D6_DIR_D7, 13000)
motor_bl = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000)
motor_br = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000)
motors = [motor_fl, motor_fr, motor_bl, motor_br]

print("board uid:", unique_id())
print("raw motor test start")
wait_c14_start()

try:
    # 与你原工程一致的极性方向（fr, br 取反）
    step("forward", [7000, -7000, 7000, -7000])
    step("backward", [-7000, 7000, -7000, 7000])
    step("strafe right", [7000, 7000, -7000, -7000])
    step("strafe left", [-7000, -7000, 7000, 7000])
    step("rotate cw", [7000, 7000, 7000, 7000])
    step("rotate ccw", [-7000, -7000, -7000, -7000])
finally:
    for m in motors:
        m.duty(0)
    print("raw motor test done")
