# OpenART (OpenMV-compatible) - Near camera: stable tennis-ball detection
# Output: UART JSON line with fields: frame, object, zone

import sensor, image, time
try:
    import ujson as json
except Exception:
    import json
try:
    from machine import UART
except Exception:
    from pyb import UART

# UART config (camera side). Keep 115200 to match host.
# Different OpenART/OpenMV firmwares expose the external header UART on different IDs.
# You have already observed `"uart_id": 2` on the receiver side, so default to UART2.
# If your wiring uses another UART, change this (e.g. (1,) or (3,)).
UART_IDS = (2,)
uarts = []  # list of (uid, uart)
for _uid in UART_IDS:
    try:
        uarts.append((_uid, UART(_uid, 115200)))
    except Exception:
        pass

print(
    "openart near started; uarts_opened=", len(uarts),
    "uart_ids_ok=", [p[0] for p in uarts],
    "uart_ids_tried=", UART_IDS,
)

_stat_ok = 0
_stat_fail = 0
_stat_last_ms = time.ticks_ms()

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)      # 320x240
sensor.skip_frames(time=1500)
sensor.set_auto_whitebal(False)
sensor.set_auto_gain(False)

# Tennis ball LAB threshold (yellow-green). Tune on-site if needed.
TH_BALL = (35, 92, -35, 18, 18, 90)

# Near camera: stricter thresholds to reduce false positives.
PIXELS_TH = 120
AREA_TH = 120

while True:
    img = sensor.snapshot()

    blobs = img.find_blobs([TH_BALL], pixels_threshold=PIXELS_TH, area_threshold=AREA_TH, merge=True)

    obj = None
    if blobs:
        b = max(blobs, key=lambda x: x.pixels())
        img.draw_rectangle(b.rect(), color=(255, 0, 0))
        img.draw_cross(b.cx(), b.cy(), color=(255, 0, 0))
        obj = {
            "x": int(b.cx()),
            "y": int(b.cy()),
            "w": int(b.w()),
            "h": int(b.h()),
            "size": int(max(b.w(), b.h())),
            "held": False,
            "method": "near_blob",
        }

    base_payload = {
        "frame": {"w": 320, "h": 240},
        "object": obj,
        "zone": None,
    }

    for (_uid, _u) in uarts:
        payload = base_payload.copy()
        payload["uart_id"] = int(_uid)
        line = json.dumps(payload) + "\n"
        try:
            line = line.encode("utf-8")
        except Exception:
            pass
        try:
            _u.write(line)
            _stat_ok += 1
        except Exception:
            _stat_fail += 1

    _now = time.ticks_ms()
    if time.ticks_diff(_now, _stat_last_ms) >= 1000:
        print("uart_write ok=", _stat_ok, "fail=", _stat_fail)
        _stat_ok = 0
        _stat_fail = 0
        _stat_last_ms = _now
    time.sleep_ms(40)   # ~25Hz
