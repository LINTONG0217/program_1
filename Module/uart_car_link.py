import time
import json
try:
    from machine import UART
except ImportError:
    pass

class UartCarLink:
    """
    閫氳繃閫忎紶妯″潡 (濡傞€愰鏃犵嚎涓插彛妯″潡 CH573/CH582) 瀹炵幇鍙岃溅閫氫俊
    鍩轰簬 JSON 鏍煎紡鎺у埗
    """
    def __init__(self, uart_id, baudrate=115200, tx_pin=None, rx_pin=None):
        try:
            self.uart = UART(int(uart_id), int(baudrate))
            print("Seekfree Wireless UART {} initialized at {} baud.".format(uart_id, baudrate))
        except Exception as e:
            print("Seekfree Wireless UART init error:", e)
            self.uart = None
        self.last_send_ms = 0
        self.last_recv_ms = 0
        self.remote_data = {}
        
    def send(self, data_dict):
        if not self.uart:
            return False
        try:
            payload = json.dumps(data_dict) + "\n"
            self.uart.write(payload.encode("utf-8"))
            self.last_send_ms = time.ticks_ms()
            return True
        except Exception as e:
            return False
            
    def receive(self):
        if not self.uart:
            return None
        try:
            if not self.uart.any():
                return None
            line = self.uart.readline()
            if line:
                try:
                    data_dict = json.loads(line.decode("utf-8").strip())
                    self.remote_data.update(data_dict)
                    self.last_recv_ms = time.ticks_ms()
                    return data_dict
                except Exception:
                    pass # ignore broken json payload from transparent connection 
        except Exception:
            pass
        return None
        
    def get_remote_data(self):
        return self.remote_data
