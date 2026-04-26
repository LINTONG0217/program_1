import time
import sys

from Module import config
from Module.ydlidar_receiver import YDLidarReceiver


def build_receiver():
    return YDLidarReceiver(
        uart_id=getattr(config, "LIDAR_UART_ID", 5),
        baudrate=getattr(config, "LIDAR_BAUDRATE", 230400),
        tx_pin=getattr(config, "LIDAR_TX_PIN", 4),
        rx_pin=getattr(config, "LIDAR_RX_PIN", 5),
        config=config,
    )


def _ticks_ms():
    fn = getattr(time, "ticks_ms", None)
    if callable(fn):
        return fn()
    return int(time.time() * 1000)


def _ticks_diff(a, b):
    fn = getattr(time, "ticks_diff", None)
    if callable(fn):
        return fn(a, b)
    return a - b


def main():
    rx = build_receiver()

    timeout_ms = int(getattr(config, "LIDAR_DEBUG_TIMEOUT_MS", 600))
    period_ms = int(getattr(config, "LIDAR_DEBUG_PERIOD_MS", 100))

    print("===== LIDAR TEST (YDLIDAR T-MINI PLUS) =====")
    try:
        print("platform=", getattr(sys, "platform", None))
    except Exception:
        pass
    try:
        info = rx.get_uart_info() if hasattr(rx, "get_uart_info") else None
        if info:
            print("uart_open=", info)
    except Exception:
        pass
    print(
        "uart_id=", getattr(config, "LIDAR_UART_ID", None),
        "baud=", getattr(config, "LIDAR_BAUDRATE", None),
        "tx=", getattr(config, "LIDAR_TX_PIN", None),
        "rx=", getattr(config, "LIDAR_RX_PIN", None),
    )
    print(
        "front_deg=[{}, {}] dist_mm=[{}, {}] hold_ms={} size_scale={} offx/deg={}".format(
            getattr(config, "LIDAR_FRONT_MIN_DEG", None),
            getattr(config, "LIDAR_FRONT_MAX_DEG", None),
            getattr(config, "LIDAR_MIN_DISTANCE_MM", None),
            getattr(config, "LIDAR_MAX_DISTANCE_MM", None),
            getattr(config, "LIDAR_HOLD_MS", None),
            getattr(config, "LIDAR_SIZE_SCALE", None),
            getattr(config, "LIDAR_OFFSET_X_PER_DEG", None),
        )
    )
    print("timeout_ms=", timeout_ms, "period_ms=", period_ms)
    print("CTRL+C to stop")

    while True:
        frame = rx.read_frame()
        now = _ticks_ms()
        src = "NEW" if frame else "NONE"

        if not frame:
            age_ms = _ticks_diff(now, rx.last_update_ms)
            if rx.last_frame is not None and age_ms <= timeout_ms:
                frame = rx.last_frame
                src = "HELD" if age_ms >= int(getattr(config, "LIDAR_HOLD_MS", 120)) else "CACHE"
                obj = frame.get("object") if isinstance(frame, dict) else None
                if isinstance(obj, dict):
                    obj["held"] = bool(src == "HELD")

        obj = None
        if isinstance(frame, dict):
            obj = frame.get("object")

        age_ms = _ticks_diff(now, rx.last_update_ms)
        if isinstance(obj, dict):
            print(
                "{:>8}ms {:>5} dist={:>4}mm ang={:>6.1f}deg size={:>3} offx={:>4} held={} age={}ms".format(
                    now,
                    src,
                    int(obj.get("distance_mm", -1)),
                    float(obj.get("angle_deg", 0.0)),
                    int(obj.get("size", 0)),
                    int(obj.get("offset_x", 0)),
                    bool(obj.get("held", False)),
                    int(age_ms),
                )
            )
        else:
            print("{:>8}ms {:>5} no_target age={}ms".format(now, src, int(age_ms)))

        try:
            time.sleep_ms(period_ms)
        except Exception:
            time.sleep(period_ms / 1000.0)


if __name__ == "__main__":
    main()
