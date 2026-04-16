import time
from machine import UART, Pin
from Module import config

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

    print("press {} to start UWB Distance Polling test".format(pin_name))
    while key.value() != 0:
        time.sleep_ms(20)
    time.sleep_ms(60)
    while key.value() == 0:
        time.sleep_ms(20)

def main():
    wait_c14_start()

    uart_id = getattr(config, "UWB_UART_ID", 1)
    baudrate = getattr(config, "UWB_BAUDRATE", 115200)
    tx_pin = getattr(config, "UWB_TX_PIN", None)
    rx_pin = getattr(config, "UWB_RX_PIN", None)
    
    try:
        # 兼容不同 MicroPython 版本的串口初始化
        try:
            if tx_pin is not None and rx_pin is not None:
                uart = UART(uart_id, baudrate, tx=Pin(tx_pin), rx=Pin(rx_pin))
            else:
                uart = UART(uart_id, baudrate)
        except Exception:
            uart = UART(uart_id, baudrate)
        
        print("[TEST] DW3000 UWB UART init success.")
        print("[TEST] UART: {}, Baudrate: {}".format(uart_id, baudrate))
        print("--------------------------------------------------")
    except Exception as e:
        print("[ERR] DW3000 UWB UART init failed:", e)
        return

    last_poll = 0
    poll_interval = 150  # 每 150ms 发送一次查询指令

    while True:
        now = time.ticks_ms()
        
        # 1. 主动发送查询距离的 AT 指令
        if time.ticks_diff(now, last_poll) >= poll_interval:
            last_poll = now
            # 发送刚才我们测出来的绝密指令，必须带回车换行 \r\n
            uart.write(b"AT+DISTANCE\r\n")

        # 2. 接收 UWB 模块的回复
        if getattr(uart, "any", lambda: 0)():
            try:
                line = uart.readline()
                if line:
                    text = line.decode('utf-8', 'ignore').strip()
                    # 只要包含 distance: 就提取后面的数字
                    if "distance:" in text.lower():
                        try:
                            # 格式如 "distance: 1.250000"
                            dist_str = text.split(":")[1].strip()
                            dist_m = float(dist_str)
                            print("[UWB] 双车距离: {:.3f} 米".format(dist_m))
                        except ValueError:
                            print("[RAW 解析失败]:", text)
                    else:
                        # 打印其它原始回复内容，方便我们排查到底回了什么！
                        if text:
                            print("[RAW 回复]:", text)
            except Exception as e:
                pass
        
        time.sleep_ms(10)

if __name__ == "__main__":
    main()
