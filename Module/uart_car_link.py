import json
import time

try:
    from machine import Pin, UART
except ImportError:
    Pin = None
    UART = None


class UartCarLink:
    """Lightweight JSON line link for master/slave coordination."""

    def __init__(self, uart_id, baudrate=115200, tx_pin=None, rx_pin=None):
        try:
            if UART is None:
                raise RuntimeError("machine.UART unavailable")
            self.uart = self._build_uart(uart_id, baudrate, tx_pin, rx_pin)
            print("Seekfree Wireless UART {} initialized at {} baud.".format(uart_id, baudrate))
        except Exception as e:
            print("Seekfree Wireless UART init error:", e)
            self.uart = None
        self.last_send_ms = 0
        self.last_recv_ms = 0
        self.remote_data = {}

    def _safe_pin(self, value):
        if value is None or Pin is None:
            return None
        try:
            return Pin(value)
        except Exception:
            try:
                return Pin(int(value))
            except Exception:
                return None

    def _build_uart(self, uart_id, baudrate, tx_pin=None, rx_pin=None):
        tx_obj = self._safe_pin(tx_pin)
        rx_obj = self._safe_pin(rx_pin)

        if tx_obj is not None and rx_obj is not None:
            try:
                return UART(int(uart_id), int(baudrate), tx=tx_obj, rx=rx_obj)
            except Exception:
                pass

        return UART(int(uart_id), int(baudrate))

    def send(self, data_dict):
        if not self.uart:
            return False
        try:
            payload = json.dumps(data_dict) + "\n"
            self.uart.write(payload.encode("utf-8"))
            self.last_send_ms = time.ticks_ms()
            return True
        except Exception:
            return False

    def send_status(self, state=None, seq=None, status=None, online=True, role=None):
        payload = {
            "type": "status",
            "online": bool(online),
            "ts": time.ticks_ms(),
        }
        if state is not None:
            payload["state"] = state
        if seq is not None:
            payload["seq"] = int(seq)
        if status is not None:
            payload["status"] = status
        if role is not None:
            payload["role"] = role
        return self.send(payload)

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
                    pass
        except Exception:
            pass
        return None

    def get_remote_data(self):
        return self.remote_data

    def peer_age_ms(self):
        if not self.last_recv_ms:
            return None
        return time.ticks_diff(time.ticks_ms(), self.last_recv_ms)

    def is_peer_online(self, timeout_ms=300):
        age = self.peer_age_ms()
        if age is None:
            return False
        return age <= int(timeout_ms)
