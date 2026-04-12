import time
from machine import Pin

from BSP.omni_chassis_driver import OmniChassis
from BSP.motor_actuator import Motor
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


def build_chassis():
    motor_fl = Motor(*config.MOTOR_FL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fl", False))
    motor_fr = Motor(*config.MOTOR_FR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fr", False))
    motor_bl = Motor(*config.MOTOR_BL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("bl", False))
    motor_br = Motor(*config.MOTOR_BR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("br", False))
    return OmniChassis(motor_fl, motor_fr, motor_bl, motor_br, config=config)


def main():
    chassis = build_chassis()
    wait_c14_start()
    cfg = getattr(chassis, "cfg", None)
    if cfg:
        cfg.CHASSIS_ENABLE_CLOSED_LOOP = False
        cfg.CHASSIS_CMD_DEADBAND = 0
        cfg.CHASSIS_MAX_VX_STEP = 100
        cfg.CHASSIS_MAX_VY_STEP = 100
        cfg.CHASSIS_MAX_VW_STEP = 100

    chassis.wheel_trim = {"fl": 1.00, "fr": 0.70, "bl": 1.00, "br": 0.70}
    chassis.attach_feedback(encoders=None, imu=None)

    print("straight-force-trim v2")
    print("wheel_trim:", chassis.wheel_trim)
    start = time.ticks_ms()
    try:
        while time.ticks_diff(time.ticks_ms(), start) < 2500:
            chassis.forward(45)
            time.sleep_ms(50)
    finally:
        chassis.stop()
        print("done")


if __name__ == "__main__":
    main()
