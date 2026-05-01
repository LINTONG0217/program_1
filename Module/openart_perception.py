import sensor, image, time
try:
    import ujson as json
except:
    import json
from machine import UART

# 主控端当前配置：UART1, 115200
uart = UART(1, 115200)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)      # 320x240
sensor.skip_frames(time=1500)
sensor.set_auto_whitebal(False)
sensor.set_auto_gain(False)

# 网球初始阈值（黄绿）
TH_BALL = (35, 92, -35, 18, 18, 90)

while True:
    img = sensor.snapshot()

    blobs = img.find_blobs([TH_BALL], pixels_threshold=120, area_threshold=120, merge=True)

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
            "size": int(max(b.w(), b.h()))
        }

    payload = {
        "frame": {"w": 320, "h": 240},
        "object": obj,
        "zone": None
    }

    uart.write(json.dumps(payload) + "\n")
    time.sleep_ms(40)   # 25Hz
