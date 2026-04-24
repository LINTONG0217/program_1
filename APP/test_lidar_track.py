import time
from machine import Pin
from Module import config
from Module.ydlidar_receiver import YDLidarReceiver
from BSP.omni_chassis_driver import OmniChassis
from BSP.motor_actuator import Motor

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
    print("Press {} to start Lidar Tracking Test".format(pin_name))
    print("========================================")
    while key.value() != 0:
        time.sleep_ms(20)
    time.sleep_ms(60)
    while key.value() == 0:
        time.sleep_ms(20)

def build_chassis():
    motor_fl = Motor(*config.MOTOR_FL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fl", False))
    motor_fr = Motor(*config.MOTOR_FR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fr", True))
    motor_bl = Motor(*config.MOTOR_BL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("bl", False))
    motor_br = Motor(*config.MOTOR_BR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("br", True))
    return OmniChassis(motor_fl, motor_fr, motor_bl, motor_br, config=config)

def main():
    print("[INIT] Starting Lidar...")
    rx = YDLidarReceiver(
        uart_id=getattr(config, "LIDAR_UART_ID", 5),
        baudrate=getattr(config, "LIDAR_BAUDRATE", 230400),
        tx_pin=getattr(config, "LIDAR_TX_PIN", 4),
        rx_pin=getattr(config, "LIDAR_RX_PIN", 5),
        config=config,
    )
    
    print("[INIT] Starting Chassis...")
    chassis = build_chassis()
    
    # 强制修改PID限制，防止车跑太快
    chassis.cfg.CHASSIS_MAX_VX_STEP = 50
    chassis.cfg.CHASSIS_MAX_VY_STEP = 50
    chassis.cfg.CHASSIS_MAX_VW_STEP = 50

    wait_c14_start()
    print("[RUN] Tracking started! Put the ball in front of the Lidar.")

    last_print = 0
    try:
        while True:
            now = time.ticks_ms()
            frame = rx.read_frame()
            obj = None
            if isinstance(frame, dict):
                obj = frame.get("object")

            # 卡尔曼滤波增强：丢帧时用上一次目标的预测
            if obj is None and hasattr(rx, "last_obj_kf"):
                rx.last_obj_kf.predict()
                kx, ky, kvx, kvy = rx.last_obj_kf.get_state()
                obj = {"distance_mm": kx, "angle_deg": ky, "size": 30}  # 这里假设distance/angle可用

            if obj:
                # 初始化/更新卡尔曼
                if not hasattr(rx, "last_obj_kf"):
                    from Module.lidar_grid_navigation import KalmanFilter2D
                    rx.last_obj_kf = KalmanFilter2D(obj.get("distance_mm", 0), obj.get("angle_deg", 0))
                rx.last_obj_kf.update(obj.get("distance_mm", 0), obj.get("angle_deg", 0))
                # 提取雷达解算后的球的参数
                distance = obj.get("distance_mm", 0)
                angle = obj.get("angle_deg", 0)
                size = obj.get("size", 0)
                
                vx = 0
                vy = 0
                
                # 1. 如果看到了球（由 config 中 TARGET_OBJECT_SIZE 决定的 size）
                # 雷达默认：尺寸 > 100 算是看到球了，如果大于 200 就是快撞上了
                if size > 25:
                    # 如果距离在 150mm 到 1500mm 之间，进行追踪
                    if 150 < distance < 1500:
                        # 往前走，固定一个比较柔和的速度 (vx = 25 差不多)
                        vx = 30
                        
                        # 左右偏离的话，加上横移来对中
                        if angle > 8:   # 球在左边
                            vy = -20
                        elif angle < -8: # 球在右边
                            vy = 20
                            
                        # 如果打印频率控制
                        if time.ticks_diff(now, last_print) > 200:
                            print("[TRACK] Ball D:{:.0f}mm, Ang:{:.1f}°, S:{:.0f} | Move: vx={}, vy={}".format(distance, angle, size, vx, vy))
                            last_print = now
                            
                        chassis.move(vx, vy, 0)
                    elif distance <= 150:
                        # 靠太近了，停车！
                        if time.ticks_diff(now, last_print) > 200:
                            print("[CATCH] Ball is too close! Stopping.")
                            last_print = now
                        chassis.stop()
                    else:
                        # 太远了，不动
                        chassis.stop()
                else:
                    chassis.stop()
            else:
                # 没看到球，停车
                chassis.stop()
                
            time.sleep_ms(20)
            
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("Error:", e)
    finally:
        chassis.stop()
        print("Chassis stopped.")

if __name__ == "__main__":
    main()
