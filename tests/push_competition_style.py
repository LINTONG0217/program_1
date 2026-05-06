"""推球测试（competition.py 同结构：build_chassis/build_vision）。

用途：
- 复用工程的 Motor/OmniChassis/VisionReceiver 初始化方式
- 简单状态机：SEARCH -> APPROACH -> PUSH -> (optional) RETREAT
- 在终端限频打印视觉目标与状态，便于现场调试

运行：在主控 REPL 执行：
    exec(open('tests/push_competition_style.py').read())
"""

import time

from Module import config

try:
    from machine import Pin
except Exception:
    Pin = None


def clamp(value, low, high):
    return max(low, min(high, value))


def wait_c14_start():
    """按 C14 才开始（沿用 competition/manual_* 的逻辑）。"""
    if Pin is None:
        return
    if not bool(getattr(config, "TEST_WAIT_C14_ENABLE", True)):
        return
    try:
        pull_up = getattr(Pin, "PULL_UP_47K", None)
        pin_name = getattr(config, "NAV_KEY_PIN", "C14")
        if pull_up is not None:
            key_pin = Pin(pin_name, Pin.IN, pull_up)
        else:
            key_pin = Pin(pin_name, Pin.IN)
        print("press C14 to start push")
        while key_pin.value() != 0:
            time.sleep_ms(20)
        time.sleep_ms(60)
        while key_pin.value() == 0:
            time.sleep_ms(20)
        print("C14 start")
    except Exception:
        return


class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, limit=100.0):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.limit = abs(float(limit))
        self.integral = 0.0
        self.last_error = 0.0
        self.last_ms = None

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_ms = None

    def update(self, error):
        now = time.ticks_ms()
        dt = 0.0
        if self.last_ms is not None:
            dt = time.ticks_diff(now, self.last_ms) / 1000.0
        deriv = 0.0
        if dt > 0:
            self.integral += error * dt
            deriv = (error - self.last_error) / dt
        out = self.kp * error + self.ki * self.integral + self.kd * deriv
        self.last_error = error
        self.last_ms = now
        return clamp(out, -self.limit, self.limit)


def build_chassis():
    from BSP.motor_actuator import Motor
    from BSP.omni_chassis_driver import OmniChassis

    motor_fl = Motor(*config.MOTOR_FL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fl", False))
    motor_fr = Motor(*config.MOTOR_FR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fr", False))
    motor_bl = Motor(*config.MOTOR_BL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("bl", False))
    motor_br = Motor(*config.MOTOR_BR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("br", False))
    return OmniChassis(motor_fl, motor_fr, motor_bl, motor_br, config=config)


def build_vision(cfg):
    from Module.uart_vision_receiver import VisionReceiver

    backend = str(getattr(cfg, "VISION_BACKEND", "uart") or "").strip().lower()
    if backend in ("openart_local", "openart", "sensor"):
        try:
            from Module.openart_perception import OpenArtLocalVision

            print("vision backend: openart_local")
            return OpenArtLocalVision(cfg)
        except Exception as e:
            print("openart_local unavailable, fallback to uart:", e)

    print("vision backend: uart")
    debug_rx = bool(getattr(cfg, "VISION_DEBUG_PRINT_RX", False))
    return VisionReceiver(
        cfg.VISION_UART_ID,
        cfg.VISION_BAUDRATE,
        cfg.VISION_TX_PIN,
        cfg.VISION_RX_PIN,
        frame_width=cfg.FRAME_WIDTH,
        frame_height=cfg.FRAME_HEIGHT,
        debug_print=debug_rx,
        label="push",
    )


def main():
    print("push_competition_style start")
    chassis = build_chassis()
    vision = build_vision(config)

    # 上电后先等待按键，避免误动作
    wait_c14_start()

    center_pid = PID(
        getattr(config, "PID_CENTER_P", 0.45),
        getattr(config, "PID_CENTER_I", 0.0),
        getattr(config, "PID_CENTER_D", 0.08),
        getattr(config, "MAX_LATERAL_SPEED", 35),
    )
    dist_pid = PID(
        getattr(config, "PID_DIST_P", 0.90),
        getattr(config, "PID_DIST_I", 0.0),
        getattr(config, "PID_DIST_D", 0.05),
        getattr(config, "APPROACH_MAX_SPEED", 32),
    )

    state = "SEARCH"
    push_start_ms = None
    retreat_start_ms = None

    print_ms = 400
    last_print = time.ticks_ms()

    while True:
        frame = None
        try:
            frame = vision.get_latest_frame(timeout_ms=int(getattr(config, "FRAME_TIMEOUT_MS", 500)))
        except Exception:
            frame = None
        obj = frame.get("object") if frame else None

        now = time.ticks_ms()
        if time.ticks_diff(now, last_print) >= print_ms:
            last_print = now
            try:
                stat = vision.rx_stat() if hasattr(vision, "rx_stat") else None
                age = vision.age_ms() if hasattr(vision, "age_ms") else None
            except Exception:
                stat = None
                age = None
            offset_bias = int(getattr(config, "VISION_OFFSET_X_BIAS", 0))
            if obj:
                offx_raw = int(obj.get("offset_x", 0))
                offx_adj = int(offx_raw + offset_bias)
                print(
                    "STATE=", state,
                    "offx_raw=", offx_raw,
                    "offx=", offx_adj,
                    "size=", int(obj.get("size", 0)),
                    "held=", bool(obj.get("held", False)),
                    "age_ms=", age,
                    "stat=", stat,
                )
            else:
                print("STATE=", state, "VISION: none", "age_ms=", age, "stat=", stat)

        loop_delay = int(getattr(config, "LOOP_DELAY_MS", 50))

        if state == "SEARCH":
            if obj:
                state = "APPROACH"
                center_pid.reset()
                dist_pid.reset()
            else:
                chassis.rotate(getattr(config, "SEARCH_ROT_SPEED", 18))
                time.sleep_ms(loop_delay)
                continue

        if state == "APPROACH":
            if not obj:
                state = "SEARCH"
                chassis.stop()
                time.sleep_ms(loop_delay)
                continue

            offset_bias = int(getattr(config, "VISION_OFFSET_X_BIAS", 0))
            offset_x = int(obj.get("offset_x", 0)) + offset_bias
            size = int(obj.get("size", 0))
            held = bool(obj.get("held", False))
            method = obj.get("method")

            # 居中
            center_db = int(getattr(config, "CENTER_DEADBAND", 8))
            center_error = 0 if abs(offset_x) < center_db else offset_x
            vy = center_pid.update(center_error)
            vy = clamp(vy, -float(getattr(config, "MAX_LATERAL_SPEED", 35)), float(getattr(config, "MAX_LATERAL_SPEED", 35)))
            vy *= float(getattr(config, "VISION_OFFSET_X_SIGN", 1))

            # 接近
            target_size = int(getattr(config, "TARGET_OBJECT_SIZE", 280))
            dist_error = target_size - size
            vx = dist_pid.update(dist_error)
            vx = clamp(vx, 0, float(getattr(config, "APPROACH_MAX_SPEED", 32)))

            # 安全：held/circle/太小/太偏 -> 禁止向前
            if held or method == "circle":
                vx = 0
            min_size = int(getattr(config, "APPROACH_FORWARD_MIN_SIZE", 28))
            if size < min_size:
                vx = 0
            strafe_only_px = int(getattr(config, "APPROACH_STRAFE_ONLY_PX", 140))
            if abs(offset_x) >= strafe_only_px:
                vx = 0

            chassis.move(vx, vy, 0)

            # 进入推球
            enter_size = int(getattr(config, "PUSH_ENTER_MIN_SIZE", 80))
            if size >= enter_size and not held:
                state = "PUSH"
                push_start_ms = time.ticks_ms()
                chassis.move(getattr(config, "PUSH_SPEED", 55), 0, 0)

            time.sleep_ms(loop_delay)
            continue

        if state == "PUSH":
            require_target = bool(getattr(config, "PUSH_REQUIRE_TARGET", True))
            if require_target and not obj:
                chassis.stop()
                state = "SEARCH"
                time.sleep_ms(loop_delay)
                continue

            if push_start_ms is None:
                push_start_ms = time.ticks_ms()
            push_ms = int(getattr(config, "PUSH_TIME_MS", 2200))
            if time.ticks_diff(time.ticks_ms(), push_start_ms) >= push_ms:
                if bool(getattr(config, "AFTER_PUSH_RETREAT_ENABLE", True)):
                    state = "RETREAT"
                    retreat_start_ms = time.ticks_ms()
                    chassis.move(getattr(config, "AFTER_PUSH_RETREAT_SPEED", -22), 0, 0)
                else:
                    chassis.stop()
                    state = "SEARCH"
                time.sleep_ms(loop_delay)
                continue

            chassis.move(getattr(config, "PUSH_SPEED", 55), 0, 0)
            time.sleep_ms(loop_delay)
            continue

        if state == "RETREAT":
            if retreat_start_ms is None:
                retreat_start_ms = time.ticks_ms()
            retreat_ms = int(getattr(config, "AFTER_PUSH_RETREAT_MS", 550))
            if time.ticks_diff(time.ticks_ms(), retreat_start_ms) >= retreat_ms:
                chassis.stop()
                state = "SEARCH"
                time.sleep_ms(loop_delay)
                continue
            chassis.move(getattr(config, "AFTER_PUSH_RETREAT_SPEED", -22), 0, 0)
            time.sleep_ms(loop_delay)


if __name__ == "__main__":
    main()
