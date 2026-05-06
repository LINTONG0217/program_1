# OpenART/OpenMV-compatible tennis-ball detector for push-out tests.
# Sends: SP|<json>|<crc16>\n over UART2 by default.

import sensor
import time

try:
    import ujson as json
except Exception:
    import json

try:
    from machine import UART
except Exception:
    from pyb import UART


SCRIPT_VERSION = "openart_ball_pushout_v1"
UART_IDS = (2,)
BAUDRATE = 115200
FRAME_W = 320
FRAME_H = 240

# Tennis ball / yellow-green thresholds. Tune on site if needed.
BALL_THRESHOLDS = (
    (35, 92, -35, 18, 18, 90),
    (25, 100, -65, -5, -20, 95),
    (45, 100, -50, 5, 10, 95),
)

# Yellow field boundary tape. Used only to report object_out_of_field.
FIELD_THRESHOLD = (40, 95, -15, 25, 35, 95)
FIELD_MIN_PIXELS = 120
FIELD_MIN_AREA = 120
FIELD_MAX_THICKNESS = 42
FIELD_HOLD_FRAMES = 5
FIELD_INNER_MARGIN = 8

MIN_BALL_PIXELS = 45
MIN_BALL_AREA = 45
MIN_BALL_SIZE = 6
MAX_BALL_SIZE = 260
MIN_FILL_RATIO = 0.12
MIN_ASPECT = 0.55
MAX_ASPECT = 1.85
LOST_HOLD_FRAMES = 2
SMOOTH_ALPHA = 0.45
EDGE_OUT_MARGIN = 14


def crc16_ibm(data_bytes, init=0xFFFF):
    crc = int(init) & 0xFFFF
    for b in data_bytes:
        crc ^= int(b) & 0xFF
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


def open_uarts():
    outs = []
    for uid in UART_IDS:
        try:
            try:
                u = UART(uid, baudrate=BAUDRATE)
            except Exception:
                u = UART(uid, BAUDRATE)
            outs.append((uid, u))
            print("uart open ok:", uid)
        except Exception as e:
            print("uart open failed:", uid, e)
    return outs


def blob_score(blob):
    w = max(1, int(blob.w()))
    h = max(1, int(blob.h()))
    size = max(w, h)
    if size < MIN_BALL_SIZE or size > MAX_BALL_SIZE:
        return -1

    aspect = w / float(h)
    if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
        return -1

    fill = blob.pixels() / float(w * h)
    if fill < MIN_FILL_RATIO:
        return -1

    # Reject long tape-like blobs.
    if (w >= 90 and h <= 8) or (h >= 90 and w <= 8):
        return -1
    if (w >= 140 and h <= 14) or (h >= 140 and w <= 14):
        return -1

    square_penalty = abs(aspect - 1.0)
    return blob.pixels() * (1.0 - 0.35 * square_penalty) * (0.75 + 0.25 * fill)


def is_greenish(img, x, y):
    try:
        sr = sg = sb = count = 0
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                p = img.get_pixel(x + dx, y + dy)
                if isinstance(p, tuple):
                    r, g, b = p
                else:
                    r = g = b = p
                sr += r
                sg += g
                sb += b
                count += 1
        if count <= 0:
            return True
        r = sr // count
        g = sg // count
        b = sb // count
        return g > r + 10 and g > b + 10 and g > 35
    except Exception:
        return True


def detect_ball(img):
    blobs = []
    for threshold in BALL_THRESHOLDS:
        try:
            found = img.find_blobs(
                [threshold],
                pixels_threshold=MIN_BALL_PIXELS,
                area_threshold=MIN_BALL_AREA,
                merge=False,
            )
            if found:
                blobs.extend(found)
        except Exception:
            pass

    best = None
    best_score = -1
    for blob in blobs:
        score = blob_score(blob)
        if score > best_score and is_greenish(img, int(blob.cx()), int(blob.cy())):
            best_score = score
            best = blob

    if best is None:
        return None

    try:
        img.draw_rectangle(best.rect(), color=(255, 0, 0))
        img.draw_cross(best.cx(), best.cy(), color=(255, 0, 0))
    except Exception:
        pass

    return {
        "x": int(best.cx()),
        "y": int(best.cy()),
        "w": int(best.w()),
        "h": int(best.h()),
        "size": int(max(best.w(), best.h())),
        "held": False,
        "method": "ball_blob",
    }


def detect_field_bounds(img):
    try:
        blobs = img.find_blobs(
            [FIELD_THRESHOLD],
            pixels_threshold=FIELD_MIN_PIXELS,
            area_threshold=FIELD_MIN_AREA,
            merge=True,
        )
    except Exception:
        blobs = None

    if not blobs:
        return None

    valid = []
    for blob in blobs:
        w = int(blob.w())
        h = int(blob.h())
        long_enough = w >= int(FRAME_W * 0.20) or h >= int(FRAME_H * 0.20)
        thin_enough = min(w, h) <= FIELD_MAX_THICKNESS
        if long_enough and thin_enough:
            valid.append(blob)

    if not valid:
        try:
            valid = sorted(blobs, key=lambda b: b.pixels(), reverse=True)[:3]
        except Exception:
            valid = blobs[:3]

    min_x = FRAME_W
    min_y = FRAME_H
    max_x = 0
    max_y = 0
    for blob in valid:
        x0 = int(blob.x())
        y0 = int(blob.y())
        x1 = x0 + int(blob.w())
        y1 = y0 + int(blob.h())
        min_x = min(min_x, x0)
        min_y = min(min_y, y0)
        max_x = max(max_x, x1)
        max_y = max(max_y, y1)

    return (
        max(0, min(FRAME_W - 1, min_x)),
        max(0, min(FRAME_H - 1, min_y)),
        max(0, min(FRAME_W, max_x)),
        max(0, min(FRAME_H, max_y)),
    )


def object_is_out(obj, bounds):
    if not obj:
        return False
    x = int(obj.get("x", FRAME_W // 2))
    y = int(obj.get("y", FRAME_H // 2))
    if bounds is not None:
        x0, y0, x1, y1 = bounds
        margin = FIELD_INNER_MARGIN
        return x <= x0 + margin or x >= x1 - margin or y <= y0 + margin or y >= y1 - margin
    return x <= EDGE_OUT_MARGIN or x >= FRAME_W - EDGE_OUT_MARGIN or y <= EDGE_OUT_MARGIN or y >= FRAME_H - EDGE_OUT_MARGIN


def send_payload(uarts, payload):
    line = json.dumps(payload)
    try:
        body = line.encode("utf-8")
    except Exception:
        body = bytes(line)
    crc = crc16_ibm(body)
    packet = b"SP|" + body + b"|" + ("%04X" % crc).encode("ascii") + b"\n"
    for _, uart in uarts:
        try:
            uart.write(packet)
        except Exception:
            pass


print(SCRIPT_VERSION)
uarts = open_uarts()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=1500)
sensor.set_auto_whitebal(False)
sensor.set_auto_gain(True)

last_obj = None
lost_count = 0
last_bounds = None
field_lost = 0
sent = 0
last_print = time.ticks_ms()

while True:
    img = sensor.snapshot()
    obj = detect_ball(img)

    if obj is not None:
        if last_obj is not None:
            obj["x"] = int(last_obj["x"] * (1.0 - SMOOTH_ALPHA) + obj["x"] * SMOOTH_ALPHA)
            obj["y"] = int(last_obj["y"] * (1.0 - SMOOTH_ALPHA) + obj["y"] * SMOOTH_ALPHA)
            obj["w"] = int(last_obj["w"] * (1.0 - SMOOTH_ALPHA) + obj["w"] * SMOOTH_ALPHA)
            obj["h"] = int(last_obj["h"] * (1.0 - SMOOTH_ALPHA) + obj["h"] * SMOOTH_ALPHA)
            obj["size"] = int(max(obj["w"], obj["h"]))
        last_obj = obj
        lost_count = 0
    elif last_obj is not None and lost_count < LOST_HOLD_FRAMES:
        obj = last_obj.copy()
        obj["held"] = True
        lost_count += 1
    else:
        last_obj = None

    bounds = detect_field_bounds(img)
    field_held = False
    if bounds is None:
        if last_bounds is not None and field_lost < FIELD_HOLD_FRAMES:
            bounds = last_bounds
            field_lost += 1
            field_held = True
        else:
            last_bounds = None
            field_lost = 0
    else:
        last_bounds = bounds
        field_lost = 0

    payload = {
        "frame": {"w": FRAME_W, "h": FRAME_H},
        "object": obj,
        "zone": None,
        "objects_count": 1 if obj else 0,
        "object_out_of_field": bool(object_is_out(obj, bounds)),
        "field_bounds": list(bounds) if bounds is not None else None,
        "field_held": bool(field_held),
    }
    send_payload(uarts, payload)
    sent += 1

    try:
        img.draw_cross(FRAME_W // 2, FRAME_H // 2, color=(0, 255, 0))
        img.draw_string(2, 2, "sent:{} obj:{} out:{}".format(sent, 1 if obj else 0, 1 if payload["object_out_of_field"] else 0), color=(255, 255, 255))
    except Exception:
        pass

    now = time.ticks_ms()
    if time.ticks_diff(now, last_print) >= 1000:
        print("alive sent=", sent, "obj=", obj is not None, "out=", payload["object_out_of_field"], "uarts=", [uid for uid, _ in uarts])
        last_print = now

    time.sleep_ms(40)
