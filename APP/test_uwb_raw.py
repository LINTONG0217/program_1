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

    print("press {} to start UWB Raw test".format(pin_name))
    while key.value() != 0:
        time.sleep_ms(20)
    time.sleep_ms(60)
    while key.value() == 0:
        time.sleep_ms(20)

def main():
    wait_c14_start()

    uart_id = getattr(config, "UWB_UART_ID", 1)  # 默认串口 1
    baudrate = getattr(config, "UWB_BAUDRATE", 115200)
    tx_pin = getattr(config, "UWB_TX_PIN", None)
    rx_pin = getattr(config, "UWB_RX_PIN", None)
    
    try:
        # MicroPython UART initialization fallback
        try:
            # 1. Try standard with dict kwargs (often fails on some MicroPython ports)
            if tx_pin is not None and rx_pin is not None:
                uart = UART(uart_id, baudrate, tx=Pin(tx_pin), rx=Pin(rx_pin))
            else:
                uart = UART(uart_id, baudrate)
        except Exception:
            # 2. Try positional arguments only or without Pin wrapper
            if tx_pin is not None and rx_pin is not None:
                uart = UART(uart_id, baudrate)
            else:
                uart = UART(uart_id, baudrate)
        
        print("[TEST] DW3000 UWB Raw UART init success.")
        print("[TEST] UART: {}, Baudrate: {}".format(uart_id, baudrate))
        print("--------------------------------------------------")
        print("Waiting for any raw data from DW3000...")
    except Exception as e:
        print("[ERR] DW3000 UWB Raw UART init failed:", e)
        return

    while True:
        if getattr(uart, "any", lambda: 0)():
            try:
                line = uart.readline()
                if line:
                    # 尝试用 utf-8 解码，如果不行就直接打印原始字节
                    try:
                        text = line.decode('utf-8').strip()
                        if text:
                            print("[UWB Raw Data]:", text)
                    except UnicodeError:
                        print("[UWB Raw Bytes]:", line)
            except Exception as e:
                print("Read error:", e)
        
        time.sleep_ms(10)

if __name__ == "__main__":
    main()
