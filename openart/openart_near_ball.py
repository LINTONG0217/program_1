import sensor, time, gc

try:
    from machine import UART
except Exception:
    from pyb import UART

print("VISION_SCRIPT_LOW_MEM_V1")

DEBUG_PRINT = False
BAUDRATE = 115200
UART_CANDIDATES = (2,)
WIRE_USE_SP_CRC = True

FRAME_W = 320
FRAME_H = 240
FRAME_AREA = FRAME_W * FRAME_H

TH_BALLS = (
    (0, 100, -90, -5, -60, 80),
    (30, 100, -60, -10, -40, 60),
    (20, 100, -75, -5, -50, 90),
)

MIN_SIZE = 6
MAX_SIZE = 260
MIN_FILL_RATIO = 0.12
MIN_ASPECT = 0.55
MAX_ASPECT = 1.85
SMOOTH_ALPHA = 0.45
LOST_HOLD_FRAMES = 1


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
    for uid in UART_CANDIDATES:
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


def blob_score(b):
    w = max(1, int(b.w()))
    h = max(1, int(b.h()))
    size = max(w, h)
    if size < MIN_SIZE or size > MAX_SIZE:
        return -1
    aspect = w / float(h)
    if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
        return -1
    fill_ratio = b.pixels() / float(w * h)
    if fill_ratio < MIN_FILL_RATIO:
        return -1
    if (w >= 90 and h <= 8) or (h >= 90 and w <= 8):
        return -1
    if w >= 140 and h <= 14:
        return -1
    if h >= 140 and w <= 14:
        return -1
    if b.pixels() >= FRAME_AREA * 0.85:
        return -1
    return b.pixels() * (1.0 - 0.35 * abs(aspect - 1.0))


def center_rgb(img, x, y):
    sr = sg = sb = 0
    sc = 0
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            try:
                p = img.get_pixel(x + dx, y + dy)
                if isinstance(p, tuple):
                    r, g, b = p
                else:
                    r = g = b = p
                sr += r
                sg += g
                sb += b
                sc += 1
            except Exception:
                pass
    if sc:
        return sr // sc, sg // sc, sb // sc
    return 0, 0, 0


def is_greenish(rgb):
    r, g, b = rgb
    return g > r + 12 and g > b + 12 and g > 40


def make_payload(obj):
    if obj:
        label = "ball"
        conf = 0.0 if obj.get("held", False) else 1.0
        cx = int(obj["x"])
        cy = int(obj["y"])
        pixels = int(obj.get("pixels", obj["w"] * obj["h"]))
        held = "true" if obj.get("held", False) else "false"
    else:
        label = "none"
        conf = 0.0
        cx = -1
        cy = -1
        pixels = 0
        held = "false"
    return '{{"label":"{}","confidence":{:.1f},"cx":{},"cy":{},"pixels":{},"held":{}}}'.format(
        label, conf, cx, cy, pixels, held
    )


def send_payload(uarts, payload):
    try:
        data = payload.encode("utf-8")
    except Exception:
        data = bytes(payload)
    if WIRE_USE_SP_CRC:
        tx = b"SP|" + data + b"|%04X\n" % crc16_ibm(data)
    else:
        tx = data + b"\n"
    ok = 0
    fail = 0
    for _, u in uarts:
        try:
            u.write(tx)
            ok += 1
        except Exception:
            fail += 1
    return ok, fail


uarts = open_uarts()
print("OPEN_UARTS:", [uid for uid, _ in uarts])

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=1200)
sensor.set_auto_whitebal(False)
sensor.set_auto_gain(True)

last_obj = None
lost_count = 0
sent_count = 0
uart_ok = 0
uart_fail = 0
last_heartbeat = time.ticks_ms()

while True:
    img = sensor.snapshot()
    best = None
    best_score = -1

    for th in TH_BALLS:
        try:
            blobs = img.find_blobs([th], pixels_threshold=12, area_threshold=50, merge=False)
        except Exception:
            blobs = None
        if not blobs:
            continue
        for b in blobs:
            score = blob_score(b)
            if score <= best_score:
                continue
            if not is_greenish(center_rgb(img, b.cx(), b.cy())):
                continue
            best = b
            best_score = score

    obj = None
    if best is not None:
        obj = {
            "x": int(best.cx()),
            "y": int(best.cy()),
            "w": int(best.w()),
            "h": int(best.h()),
            "pixels": int(best.pixels()),
            "held": False,
        }
        if last_obj is not None:
            obj["x"] = int(last_obj["x"] * (1.0 - SMOOTH_ALPHA) + obj["x"] * SMOOTH_ALPHA)
            obj["y"] = int(last_obj["y"] * (1.0 - SMOOTH_ALPHA) + obj["y"] * SMOOTH_ALPHA)
            obj["w"] = int(last_obj["w"] * (1.0 - SMOOTH_ALPHA) + obj["w"] * SMOOTH_ALPHA)
            obj["h"] = int(last_obj["h"] * (1.0 - SMOOTH_ALPHA) + obj["h"] * SMOOTH_ALPHA)
            obj["pixels"] = int(obj["w"] * obj["h"])
        last_obj = obj
        lost_count = 0
    elif last_obj is not None and lost_count < LOST_HOLD_FRAMES:
        obj = last_obj.copy()
        obj["held"] = True
        lost_count += 1
    else:
        last_obj = None

    if obj:
        try:
            img.draw_rectangle(
                int(obj["x"] - obj["w"] // 2),
                int(obj["y"] - obj["h"] // 2),
                int(obj["w"]),
                int(obj["h"]),
                (255, 0, 0),
            )
            img.draw_cross(int(obj["x"]), int(obj["y"]), (255, 0, 0))
        except Exception:
            pass

    ok, fail = send_payload(uarts, make_payload(obj))
    uart_ok += ok
    uart_fail += fail
    sent_count += 1

    now = time.ticks_ms()
    if time.ticks_diff(now, last_heartbeat) >= 1000:
        print("VISION_ALIVE sent=", sent_count, "obj=", obj is not None, "ok=", uart_ok, "fail=", uart_fail)
        last_heartbeat = now
        gc.collect()

    time.sleep_ms(35)
