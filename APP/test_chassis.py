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

    print("release {} then press to start test".format(pin_name))
    while key.value() == 0:
        time.sleep_ms(20)
    print("press {} to start test".format(pin_name))
    while key.value() != 0:
        time.sleep_ms(20)
    time.sleep_ms(60)
    while key.value() == 0:
        time.sleep_ms(20)
    print("{} start".format(pin_name))


def create_c14_toggle():
    if not bool(getattr(config, "TEST_RUNTIME_C14_TOGGLE_ENABLE", True)):
        return None
    pin_name = getattr(config, "NAV_KEY_PIN", "C14")
    key = Pin(pin_name, Pin.IN)
    pull_up = getattr(Pin, "PULL_UP_47K", None)
    if pull_up is not None:
        try:
            key = Pin(pin_name, Pin.IN, pull_up)
        except Exception:
            pass
    return {"pin": key, "last": False, "restart": False, "name": pin_name}


def handle_c14_toggle(toggle, chassis):
    if not toggle:
        return False
    pressed = toggle["pin"].value() == 0
    if pressed and not toggle["last"]:
        toggle["restart"] = True
        print(toggle["name"], "restart")
        chassis.stop()
        time.sleep_ms(60)
    toggle["last"] = pressed
    return toggle["restart"]


def consume_restart(toggle):
    if not toggle:
        return False
    if not toggle.get("restart"):
        return False
    toggle["restart"] = False
    return True


def sleep_with_toggle(ms, chassis, toggle=None):
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < ms:
        if handle_c14_toggle(toggle, chassis):
            return True
        time.sleep_ms(20)
    return False


def stop_gap(chassis, toggle=None):
    chassis.stop()
    gap_ms = int(getattr(config, "TEST_CHASSIS_STEP_GAP_MS", 1000))
    return sleep_with_toggle(gap_ms, chassis, toggle)


def run_motor_duty(chassis, motor, duty, duration_ms, toggle=None):
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < duration_ms:
        if handle_c14_toggle(toggle, chassis):
            return True
        motor.duty(duty)
        time.sleep_ms(50)
    motor.stop()
    return False


def build_chassis():
    motor_fl = Motor(*config.MOTOR_FL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fl", False))
    motor_fr = Motor(*config.MOTOR_FR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fr", False))
    motor_bl = Motor(*config.MOTOR_BL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("bl", False))
    motor_br = Motor(*config.MOTOR_BR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("br", False))
    return OmniChassis(motor_fl, motor_fr, motor_bl, motor_br, config=config)


def step(title, action, duration=1500, period_ms=50, chassis=None, toggle=None):
    print("test:", title)
    elapsed_active = 0
    while elapsed_active < duration:
        if handle_c14_toggle(toggle, chassis):
            return True
        action()
        if sleep_with_toggle(period_ms, chassis, toggle):
            return True
        elapsed_active += period_ms
    chassis.stop()
    if stop_gap(chassis, toggle):
        return True
    return False


def prepare_test_mode(chassis):
    cfg = getattr(chassis, "cfg", None)
    if not cfg:
        return
    # 测试模式：关闭死区与步进限幅，避免首次输出过小导致电机不转
    cfg.CHASSIS_CMD_DEADBAND = 0
    cfg.CHASSIS_MAX_VX_STEP = 100
    cfg.CHASSIS_MAX_VY_STEP = 100
    cfg.CHASSIS_MAX_VW_STEP = 100


def direct_motor_smoke_test(chassis, duty=None, duration_ms=None, toggle=None):
    if duty is None:
        duty = int(getattr(config, "TEST_CHASSIS_DIRECT_DUTY", 1200))
    if duration_ms is None:
        duration_ms = int(getattr(config, "TEST_CHASSIS_MOTOR_DURATION_MS", 2000))
    motors = [
        ("fl", chassis.fl),
        ("fr", chassis.fr),
        ("bl", chassis.bl),
        ("br", chassis.br),
    ]
    print("direct motor smoke test: begin")
    for name, motor in motors:
        print(
            "motor", name,
            "pins=", "{}->{}".format(getattr(motor, "pwm_pin", "?"), getattr(motor, "dir_pin", "?")),
            "backend=", "seekfree" if getattr(motor, "controller", None) is not None else "machine.PWM",
            "reverse=", getattr(motor, "reverse", False),
        )
        try:
            print("test: motor {} forward duty {}".format(name, duty))
            if run_motor_duty(chassis, motor, duty, duration_ms, toggle):
                return True
            if stop_gap(chassis, toggle):
                return True

            print("test: motor {} reverse duty {}".format(name, duty))
            if run_motor_duty(chassis, motor, -duty, duration_ms, toggle):
                return True
            if stop_gap(chassis, toggle):
                return True
        except Exception as e:
            print("motor", name, "test error:", e)
    print("direct motor smoke test: done")
    return False


def main():
    wait_c14_start()
    chassis = build_chassis()
    prepare_test_mode(chassis)
    toggle = create_c14_toggle()
    move_speed = float(getattr(config, "TEST_CHASSIS_MOVE_SPEED", 20))
    rotate_speed = float(getattr(config, "TEST_CHASSIS_ROTATE_SPEED", 15))
    move_duration = int(getattr(config, "TEST_CHASSIS_MOVE_DURATION_MS", 2000))
    try:
        while True:
            if consume_restart(toggle):
                print("restart sequence")
            if direct_motor_smoke_test(chassis, toggle=toggle):
                continue
            print("direct motor smoke test done; start chassis movement tests after gap")
            if stop_gap(chassis, toggle):
                continue
            if step("forward", lambda: chassis.forward(move_speed), duration=move_duration, chassis=chassis, toggle=toggle):
                continue
            if step("backward", lambda: chassis.forward(-move_speed), duration=move_duration, chassis=chassis, toggle=toggle):
                continue
            if step("strafe right", lambda: chassis.strafe(move_speed), duration=move_duration, chassis=chassis, toggle=toggle):
                continue
            if step("strafe left", lambda: chassis.strafe(-move_speed), duration=move_duration, chassis=chassis, toggle=toggle):
                continue
            if step("rotate cw", lambda: chassis.rotate(rotate_speed), duration=move_duration, chassis=chassis, toggle=toggle):
                continue
            if step("rotate ccw", lambda: chassis.rotate(-rotate_speed), duration=move_duration, chassis=chassis, toggle=toggle):
                continue
            if step("diagonal front-right", lambda: chassis.move(move_speed, move_speed, 0), duration=move_duration, chassis=chassis, toggle=toggle):
                continue
            if step("diagonal rear-left", lambda: chassis.move(-move_speed, -move_speed, 0), duration=move_duration, chassis=chassis, toggle=toggle):
                continue
            break
    finally:
        chassis.stop()
        print("test done")


if __name__ == '__main__':
    main()
