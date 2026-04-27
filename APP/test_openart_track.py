import gc
import time

from machine import Pin

from BSP.motor_actuator import Motor
from BSP.omni_chassis_driver import OmniChassis
from Module import config
from Module.lidar_grid_navigation import OpenArtMiniReceiver


def clamp(value, low, high):
    return max(low, min(high, value))


def wait_c14_start():
    if not bool(getattr(config, "TEST_WAIT_C14_ENABLE", True)):
        return
    pin_name = getattr(config, "NAV_KEY_PIN", "C14")
    pull_up = getattr(Pin, "PULL_UP_47K", None)
    if pull_up is not None:
        try:
            key = Pin(pin_name, Pin.IN, pull_up)
        except Exception:
            key = Pin(pin_name, Pin.IN)
    else:
        key = Pin(pin_name, Pin.IN)

    print("========================================")
    print("Press {} to start OpenArt Tracking Test".format(pin_name))
    print("========================================")
    while key.value() != 0:
        time.sleep_ms(20)
    time.sleep_ms(60)
    while key.value() == 0:
        time.sleep_ms(20)


def build_chassis():
    motor_fl = Motor(*config.MOTOR_FL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fl", False))
    motor_fr = Motor(*config.MOTOR_FR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fr", False))
    motor_bl = Motor(*config.MOTOR_BL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("bl", False))
    motor_br = Motor(*config.MOTOR_BR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("br", False))
    return OmniChassis(motor_fl, motor_fr, motor_bl, motor_br, config=config)


def search(chassis):
    chassis.move(0, 0, float(getattr(config, "OPENART_TRACK_SEARCH_VW", 12)))


def track_command(result):
    label = str(result.get("label", "")).strip()
    pixels = float(result.get("pixels", result.get("area", 0)) or 0)
    cx = result.get("cx", result.get("x", None))

    if label.lower() in ("none", "null", "no", "openart_near_ball_start"):
        return None, "[SEARCH] {}".format(label or "none")
    if pixels < float(getattr(config, "OPENART_TRACK_MIN_PIXELS", 120)):
        return None, "[SKIP] weak label={} pixels={:.0f}".format(label, pixels)
    if cx is None:
        return (float(getattr(config, "OPENART_TRACK_FORWARD_SPEED", 20)), 0), "[TRACK] label={} pixels={:.0f} no_cx".format(label, pixels)

    cx = float(cx)
    center_x = float(getattr(config, "OPENART_TRACK_CENTER_X", 160))
    err = cx - center_x
    deadband = float(getattr(config, "OPENART_TRACK_DEADBAND_PX", 18))

    vy = 0.0
    if abs(err) > deadband:
        vy = -err * float(getattr(config, "OPENART_TRACK_VY_PER_PX", 0.18))
        vy = clamp(vy, -float(getattr(config, "OPENART_TRACK_MAX_VY", 24)), float(getattr(config, "OPENART_TRACK_MAX_VY", 24)))

    if pixels >= float(getattr(config, "OPENART_TRACK_PUSH_PIXELS", 2200)):
        vx = float(getattr(config, "OPENART_TRACK_PUSH_SPEED", 34))
    else:
        vx = float(getattr(config, "OPENART_TRACK_FORWARD_SPEED", 20))

    return (vx, vy), "[TRACK] label={} cx={:.0f} err={:.0f} pixels={:.0f} | vx={:.0f} vy={:.0f}".format(label, cx, err, pixels, vx, vy)


def main():
    gc.collect()
    print("[INIT] Starting OpenArt mini...")
    print(
        "[INIT] OpenArt UART{} baud={} tx={} rx={}".format(
            getattr(config, "OPENART_MINI_UART_ID", None),
            getattr(config, "OPENART_MINI_BAUDRATE", None),
            getattr(config, "OPENART_MINI_TX_PIN", None),
            getattr(config, "OPENART_MINI_RX_PIN", None),
        )
    )
    openart = OpenArtMiniReceiver(config)

    print("[INIT] Starting Chassis...")
    chassis = build_chassis()
    chassis.cfg.CHASSIS_MAX_VX_STEP = 30
    chassis.cfg.CHASSIS_MAX_VY_STEP = 30
    chassis.cfg.CHASSIS_MAX_VW_STEP = 30
    gc.collect()

    wait_c14_start()
    print("[RUN] OpenArt tracking started.")

    last_print = 0
    try:
        while True:
            now = time.ticks_ms()
            result = openart.get_latest(timeout_ms=int(getattr(config, "OPENART_MINI_TIMEOUT_MS", 900)))
            if result:
                cmd, message = track_command(result)
                if time.ticks_diff(now, last_print) > 180:
                    print(message)
                    last_print = now
                if cmd is None:
                    search(chassis)
                else:
                    chassis.move(cmd[0], cmd[1], 0)
            else:
                if time.ticks_diff(now, last_print) > 300:
                    print("[SEARCH] no openart target")
                    last_print = now
                search(chassis)
            time.sleep_ms(int(getattr(config, "OPENART_TRACK_LOOP_MS", 20)))
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("Error:", e)
    finally:
        chassis.stop()
        print("Chassis stopped.")


if __name__ == "__main__":
    main()
