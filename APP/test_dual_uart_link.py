import time
from machine import Pin

from Module import config
from Module.uart_car_link import UartCarLink


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

    print("press {} to start uart dual-car test".format(pin_name))
    while key.value() != 0:
        time.sleep_ms(20)
    time.sleep_ms(60)
    while key.value() == 0:
        time.sleep_ms(20)


def main():
    # 角色可在 config.py 中增加 CAR_LINK_TEST_ROLE = "master" / "slave"
    role = str(getattr(config, "CAR_LINK_TEST_ROLE", "master")).lower()
    if role not in ("master", "slave"):
        role = "master"

    send_ms = int(getattr(config, "CAR_LINK_SEND_MS", 100))
    timeout_ms = int(getattr(config, "CAR_LINK_TIMEOUT_MS", 300))

    wait_c14_start()

    link = UartCarLink(
        getattr(config, "CAR_LINK_UART_ID", 2),
        getattr(config, "CAR_LINK_BAUDRATE", 115200),
        getattr(config, "CAR_LINK_TX_PIN", None),
        getattr(config, "CAR_LINK_RX_PIN", None),
    )
    if not getattr(link, "uart", None):
        print("[ERR] uart init failed")
        return

    print("[TEST] dual-car uart test start, role={}".format(role))
    print("[TEST] uart_id={}, baud={}".format(getattr(config, "CAR_LINK_UART_ID", 2), getattr(config, "CAR_LINK_BAUDRATE", 115200)))

    seq = 0
    last_send = 0
    last_peer = time.ticks_ms()
    pending_ping_ms = {}

    while True:
        now = time.ticks_ms()

        # 接收
        msg = link.receive()
        if msg:
            last_peer = now
            mtype = msg.get("type", "")

            if mtype == "ping":
                echo = msg.get("seq", -1)
                reply = {
                    "type": "pong",
                    "from": role,
                    "echo": echo,
                    "ts": now,
                }
                link.send(reply)
                print("<- ping seq={} | -> pong".format(echo))

            elif mtype == "pong":
                echo = msg.get("echo", -1)
                sent_ms = pending_ping_ms.pop(echo, None)
                if sent_ms is not None:
                    rtt = time.ticks_diff(now, sent_ms)
                    print("<- pong echo={} | rtt={}ms".format(echo, rtt))
                else:
                    print("<- pong echo={}".format(echo))

            elif mtype == "hello":
                print("<- hello from {} seq={}".format(msg.get("from", "?"), msg.get("seq", -1)))

            else:
                print("<-", msg)

        # 发送
        if time.ticks_diff(now, last_send) >= send_ms:
            last_send = now
            seq += 1
            if role == "master":
                payload = {
                    "type": "ping",
                    "from": role,
                    "seq": seq,
                    "ts": now,
                }
                pending_ping_ms[seq] = now
            else:
                payload = {
                    "type": "hello",
                    "from": role,
                    "seq": seq,
                    "ts": now,
                }
            link.send(payload)

        # 超时提示
        if time.ticks_diff(now, last_peer) > timeout_ms:
            print("[WARN] peer timeout > {}ms".format(timeout_ms))
            last_peer = now

        time.sleep_ms(10)


if __name__ == "__main__":
    main()
