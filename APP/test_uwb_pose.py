import time
from machine import Pin

from Module import config
from Module.uwb_pose_receiver import UWBPoseReceiver


def wait_c14_start():
    if not bool(getattr(config, "TEST_WAIT_C14_ENABLE", True)):
        return
    pin_name = getattr(config, "NAV_KEY_PIN", "C14")
    
    # 兼容处理无上拉的引脚
    pull_up = getattr(Pin, "PULL_UP_47K", None)
    if pull_up is not None:
        try:
            key = Pin(pin_name, Pin.IN, pull_up)
        except Exception:
            key = Pin(pin_name, Pin.IN)
    else:
        key = Pin(pin_name, Pin.IN)

    print("press {} to start UWB DW3000 test".format(pin_name))
    while key.value() != 0:
        time.sleep_ms(20)
    time.sleep_ms(60)
    while key.value() == 0:
        time.sleep_ms(20)


def main():
    wait_c14_start()

    uart_id = getattr(config, "UWB_UART_ID", 1)
    baudrate = getattr(config, "UWB_BAUDRATE", 115200)
    fmt = getattr(config, "UWB_POSE_FORMAT", "csv")
    
    # 初始化 UWB 接收模块
    try:
        uwb = UWBPoseReceiver(
            uart_id=uart_id,
            baudrate=baudrate,
            tx_pin=getattr(config, "UWB_TX_PIN", None),
            rx_pin=getattr(config, "UWB_RX_PIN", None),
            fmt=fmt
        )
        print("[TEST] DW3000 UWB init success.")
        print("[TEST] UART: {}, Baudrate: {}, Format: {}".format(uart_id, baudrate, fmt))
        print("--------------------------------------------------")
        print("Waiting for UWB data... (Ensure your DW3000 is sending data)")
    except Exception as e:
        print("[ERR] DW3000 UWB init failed:", e)
        return

    last_print = 0
    read_count = 0

    while True:
        # 非阻塞读取串口的 UWB 数据
        if getattr(uwb.uart, "any", lambda: 0)():
            pose = uwb.read_pose()
            if pose:
                read_count += 1
                now = time.ticks_ms()
                # 控制打印频率，防止刷屏（大概 100ms 打印一次）
                if time.ticks_diff(now, last_print) >= 100:
                    last_print = now
                    print("[UWB Pose] X: {:.3f} m, Y: {:.3f} m, Yaw: {:.1f}° (Total count: {})".format(
                        pose.get("x", 0.0),
                        pose.get("y", 0.0),
                        pose.get("yaw", 0.0),
                        read_count
                    ))
        
        time.sleep_ms(10)


if __name__ == "__main__":
    main()
