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

    # 直线调试模式：关闭死区和步进限幅，确保指令稳定
    cfg = getattr(chassis, "cfg", None)
    if cfg:
        cfg.CHASSIS_CMD_DEADBAND = 0
        cfg.CHASSIS_MAX_VX_STEP = 100
        cfg.CHASSIS_MAX_VY_STEP = 100
        cfg.CHASSIS_MAX_VW_STEP = 100
        # 直线标定时先关闭闭环，方便观察 WHEEL_TRIM 是否生效
        cfg.CHASSIS_ENABLE_CLOSED_LOOP = False

    # 直线标定临时修正（避免受模块缓存影响，运行即生效）
    # 右偏时先减小右侧输出：fr/br < 1.0
    chassis.wheel_trim = {
        "fl": 1.00,
        "fr": 1.00,
        "bl": 1.00,
        "br": 1.00,
    }
    chassis.attach_feedback(encoders=None, imu=None)

    speed = 45
    duration_ms = 3000
    period_ms = 50

    print("straight test start", "speed=", speed, "duration_ms=", duration_ms)
    print("wheel_trim:", getattr(chassis, "wheel_trim", {}), "closed_loop:", getattr(chassis, "closed_loop_enabled", False))
    start = time.ticks_ms()
    try:
        while time.ticks_diff(time.ticks_ms(), start) < duration_ms:
            chassis.forward(speed)
            time.sleep_ms(period_ms)
    finally:
        chassis.stop()
        print("straight test done")


if __name__ == "__main__":
    main()
