import sensor, time, gc

try:
    from machine import UART
except Exception:
    from pyb import UART

print("VISION_TINY_V1")

uart = UART(2, 115200)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=800)
sensor.set_auto_whitebal(False)
sensor.set_auto_gain(True)

TH = (0, 100, -90, -5, -60, 80)
last = time.ticks_ms()


def crc16(data):
    c = 0xFFFF
    for b in data:
        c ^= int(b) & 0xFF
        for _ in range(8):
            if c & 1:
                c = (c >> 1) ^ 0xA001
            else:
                c >>= 1
            c &= 0xFFFF
    return c


while True:
    img = sensor.snapshot()
    obj = None
    try:
        blobs = img.find_blobs([TH], pixels_threshold=10, area_threshold=30, merge=True)
    except Exception:
        blobs = None

    if blobs:
        b = max(blobs, key=lambda x: x.pixels())
        x = int(b.cx()) * 2
        y = int(b.cy()) * 2
        p = int(b.pixels()) * 4
        obj = (x, y, p)
        try:
            img.draw_rectangle(b.rect(), (255, 0, 0))
            img.draw_cross(b.cx(), b.cy(), (255, 0, 0))
        except Exception:
            pass

    if obj:
        body = '{{"label":"ball","confidence":1.0,"cx":{},"cy":{},"pixels":{},"held":false}}'.format(obj[0], obj[1], obj[2])
    else:
        body = '{"label":"none","confidence":0.0,"cx":-1,"cy":-1,"pixels":0,"held":false}'
    data = body.encode()
    uart.write(b"SP|" + data + b"|%04X\n" % crc16(data))

    now = time.ticks_ms()
    if time.ticks_diff(now, last) > 1000:
        print("TINY_ALIVE", obj)
        last = now
        gc.collect()

    time.sleep_ms(50)
