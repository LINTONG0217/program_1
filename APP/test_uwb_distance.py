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

    print("press {} to start UWB distance test".format(pin_name))
    while key.value() != 0:
        time.sleep_ms(20)
    time.sleep_ms(60)
    while key.value() == 0:
        time.sleep_ms(20)


def _pin_value(pin):
    if pin is None:
        return None
    try:
        return int(pin)
    except Exception:
        return pin


def _open_uart(uart_id, baudrate, tx_pin=None, rx_pin=None):
    if tx_pin is not None and rx_pin is not None:
        tx_obj = Pin(_pin_value(tx_pin))
        rx_obj = Pin(_pin_value(rx_pin))
        try:
            return UART(uart_id, baudrate=baudrate, tx=tx_obj, rx=rx_obj)
        except Exception:
            try:
                return UART(uart_id, baudrate, tx=tx_obj, rx=rx_obj)
            except Exception:
                pass
    try:
        return UART(uart_id, baudrate=baudrate)
    except Exception:
        return UART(uart_id, baudrate)


def _decode(payload):
    if not payload:
        return ""
    if isinstance(payload, bytes):
        try:
            return payload.decode("utf-8").strip()
        except Exception:
            return payload.decode("latin1").strip()
    return str(payload).strip()


def _read_uart(uart):
    try:
        any_count = uart.any()
    except Exception:
        any_count = 0
    if not any_count:
        return None

    try:
        line = uart.readline()
        if line:
            return line
    except Exception:
        pass

    try:
        return uart.read()
    except Exception:
        return None


def main():
    wait_c14_start()

    uart_id = getattr(config, "UWB_UART_ID", 1)
    baudrate = getattr(config, "UWB_BAUDRATE", 115200)
    tx_pin = getattr(config, "UWB_TX_PIN", None)
    rx_pin = getattr(config, "UWB_RX_PIN", None)
    poll_interval = int(getattr(config, "UWB_RANGE_POLL_MS", 150))
    poll_cmd = str(getattr(config, "UWB_RANGE_CMD_TEMPLATE", "AT+DISTANCE\r\n") or "")
    if not poll_cmd:
        poll_cmd = "AT+DISTANCE\r\n"

    try:
        uart = _open_uart(uart_id, baudrate, tx_pin, rx_pin)
        print("[UWB] UART open ok: id={} baud={} tx={} rx={}".format(uart_id, baudrate, tx_pin, rx_pin))
        print("[UWB] poll command: {!r}".format(poll_cmd))
        print("[UWB] if only TX count increases and RX stays 0, check DW3000 wiring/config.")
    except Exception as exc:
        print("[ERR] UWB UART open failed:", exc)
        return

    last_poll = 0
    last_report = time.ticks_ms()
    tx_count = 0
    rx_count = 0
    last_rx_text = ""

    while True:
        now = time.ticks_ms()

        if time.ticks_diff(now, last_poll) >= poll_interval:
            last_poll = now
            cmd = poll_cmd
            if "{id}" in cmd:
                anchor_id = tx_count % 2
                cmd = cmd.format(id=anchor_id)
            try:
                uart.write(cmd.encode("utf-8"))
                tx_count += 1
            except Exception as exc:
                print("[ERR] write failed:", exc)

        payload = _read_uart(uart)
        if payload:
            rx_count += 1
            text = _decode(payload)
            last_rx_text = text
            if text:
                print("[RX {}] {}".format(rx_count, text))
            else:
                print("[RX {} bytes] {}".format(rx_count, payload))

        if time.ticks_diff(now, last_report) >= 1000:
            last_report = now
            if rx_count:
                print("[STAT] tx={} rx={} last={!r}".format(tx_count, rx_count, last_rx_text[:60]))
            else:
                print("[STAT] tx={} rx=0 no DW3000 reply yet".format(tx_count))

        time.sleep_ms(10)


if __name__ == "__main__":
    main()
