import sensor, image, time
try:
    import ujson as json
except:
    import json

# OpenART 固件兼容：有的用 machine.UART，有的用 pyb.UART
try:
    from machine import UART
except Exception:
    from pyb import UART

print("VISION_SCRIPT_V3")


# 调试开关：OpenART 上频繁 print 会显著降帧，反而更容易“球在但识别不出”。
DEBUG_PRINT = False
DEBUG_PRINT_EVERY_MS = 800
_dbg_last_ms = time.ticks_ms()


# ================= 串口发送配置 =================
BAUDRATE = 115200
# 仅近场模式：固定只向 OpenART 的 UART2 发送（你前面已实测 uart_id=2 有效）
UART_CANDIDATES = (2,)


# ================= 线路协议（帧头 + CRC16） =================
# 参考你给的步兵自瞄链路：帧头 + CRC 能显著提升抗错位/抗干扰能力。
# 这里保持“每帧一行”，但把每行变成：
#   SP|<json>|<crc16_hex>\n
WIRE_USE_SP_CRC = True


def crc16_ibm(data_bytes, init=0xFFFF):
    """CRC-16/IBM(ARC), init=0xFFFF, poly=0xA001.

    与你发的 C 端 table 实现一致（常见 UART 协议用法）。
    """
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


uarts = open_uarts()
print("OPEN_UARTS:", [uid for uid, _ in uarts])


# ================= 视觉初始化 =================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.skip_frames(time=1500)
sensor.set_auto_whitebal(False)
# 现场偏暗时可开启自动增益以提升可见度（注意：可能改变颜色，若颜色漂移严重可设为 False）
AUTO_GAIN = True
sensor.set_auto_gain(AUTO_GAIN)

# 多候选颜色阈值：针对现场偏绿/偏暗场景做收敛
# 说明：阈值为 (L_min,L_max, A_min,A_max, B_min,B_max)（OpenMV LAB 空间）
TH_BALLS = [
    # 暗光偏绿（主候选）
    (0, 100, -90, -5, -60, 80),
    # 中等亮度偏绿（窄幅，减少背景误检）
    (30, 100, -60, -10, -40, 60),
    # 亮场偏绿黄（仍偏绿，避免纯黄边界线误检）
    (20, 100, -75, -5, -50, 90),
]

# 黄线/边界（黄色胶带）阈值：用于“出界判断”，不参与网球识别
# 这个阈值需要现场微调；若检测不到黄线，先只用 EDGE_OUT_MARGIN 退化判定。
TH_FIELD = (40, 95, -15, 25, 35, 95)
FIELD_MIN_PIXELS = 120
FIELD_MIN_AREA = 120
FIELD_MAX_THICKNESS = 40  # 放宽：旋转/模糊时黄线会变“厚”
FIELD_MIN_LONG_RATIO = 0.20  # 放宽：可能只看到一段黄线
FIELD_INNER_MARGIN = 6  # 认为“线内侧”留一点余量

# 黄线检测短暂保持，避免偶发一两帧检测不到导致出界判断抖动
FIELD_HOLD_FRAMES = 5

# 帧区域常量，用于过滤覆盖过大的伪目标
FRAME_W = 320
FRAME_H = 240
FRAME_AREA = FRAME_W * FRAME_H

# 精度增强参数（更宽松以适配暗光环境，可按现场微调）
MIN_SIZE = 6
MAX_SIZE = 360
# 过滤黄线/胶带等细长目标：提高填充率门槛并收紧长宽比
MIN_FILL_RATIO = 0.12   # pixels / (w*h)
MIN_ASPECT = 0.55       # w/h
MAX_ASPECT = 1.85
SMOOTH_ALPHA = 0.45     # 平滑系数，降低以提高响应
# 丢失保持：用于抗抖（偶发掉帧时不中断跟踪）。
# 建议保持很小：1~2 帧（25Hz 下约 40~80ms），避免球消失后仍长时间“沿旧目标冲”。
LOST_HOLD_FRAMES = 2


def _blob_score(b):
    w = max(1, int(b.w()))
    h = max(1, int(b.h()))
    size = max(w, h)
    if size < MIN_SIZE or size > MAX_SIZE:
        return -1

    aspect = w / float(h)
    if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
        return -1

    fill_ratio = b.pixels() / float(w * h)
    # 强过滤：细长条（边界黄线/胶带）直接剔除
    if (w >= 90 and h <= 8) or (h >= 90 and w <= 8):
        return -1
    if w >= 140 and h <= 14:
        return -1
    if h >= 140 and w <= 14:
        return -1
    # 过滤覆盖过大区域的伪目标（例如整个画面被极宽松阈值命中的情况）
    if b.pixels() >= FRAME_AREA * 0.9:
        return -1
    if fill_ratio < MIN_FILL_RATIO:
        return -1

    # 分数越高越像网球：面积大优先；更强调接近圆形
    square_penalty = abs(aspect - 1.0)
    return b.pixels() * (1.0 - 0.35 * square_penalty) * (0.75 + 0.25 * fill_ratio)


def pick_ball_blob(blobs):
    best = None
    best_score = -1
    for b in blobs:
        s = _blob_score(b)
        if s > best_score:
            best_score = s
            best = b
    return best


# ================= 主循环 =================
last_heartbeat = time.ticks_ms()
sent_count = 0
uart_ok = 0
uart_fail = 0
last_obj = None
lost_count = 0
OUT_MARGIN = 14
last_field_bounds = None
field_lost_count = 0


def detect_field_bounds(img_src):
    """检测黄线边界的外接范围，用于出界判断。

    返回 (min_x, min_y, max_x, max_y) 或 None。
    """
    try:
        blobs = img_src.find_blobs([TH_FIELD], pixels_threshold=FIELD_MIN_PIXELS, area_threshold=FIELD_MIN_AREA, merge=True)
    except Exception:
        blobs = None

    if not blobs:
        return None

    valid = []
    for b in blobs:
        w = int(b.w())
        h = int(b.h())
        long_enough = (w >= int(FRAME_W * FIELD_MIN_LONG_RATIO)) or (h >= int(FRAME_H * FIELD_MIN_LONG_RATIO))
        thin_enough = min(w, h) <= int(FIELD_MAX_THICKNESS)
        if long_enough and thin_enough:
            valid.append(b)

    # 如果严格条件下没有命中，退化：取像素数最大的前几段（防止完全检测不到黄线）
    if not valid:
        try:
            valid = sorted(blobs, key=lambda bb: bb.pixels(), reverse=True)[:3]
        except Exception:
            valid = blobs[:3]

    min_x = 9999
    min_y = 9999
    max_x = -1
    max_y = -1
    for b in valid:
        x0 = int(b.x())
        y0 = int(b.y())
        x1 = x0 + int(b.w())
        y1 = y0 + int(b.h())
        if x0 < min_x:
            min_x = x0
        if y0 < min_y:
            min_y = y0
        if x1 > max_x:
            max_x = x1
        if y1 > max_y:
            max_y = y1

    # 防御性裁剪
    min_x = max(0, min(FRAME_W - 1, min_x))
    max_x = max(0, min(FRAME_W, max_x))
    min_y = max(0, min(FRAME_H - 1, min_y))
    max_y = max(0, min(FRAME_H, max_y))
    return (min_x, min_y, max_x, max_y)

while True:
    img = sensor.snapshot()
    # 保留一份原始拷贝用于检测与采样，避免后续绘图覆盖影响像素读数
    try:
        orig = img.copy()
    except Exception:
        orig = img

    # 采样中心小区域平均颜色（立即在原始图像上采样），用于判断现场颜色
    try:
        cx, cy = 160, 120
        sr = sg = sb = 0
        sc = 0
        for dx in (-2, -1, 0, 1, 2):
            for dy in (-2, -1, 0, 1, 2):
                try:
                    p = orig.get_pixel(cx + dx, cy + dy)
                    if isinstance(p, tuple):
                        r, g, b = p
                    else:
                        r = g = b = p
                    sr += r; sg += g; sb += b; sc += 1
                except Exception:
                    pass
        if sc:
            avg_center = (sr // sc, sg // sc, sb // sc)
        else:
            avg_center = (0, 0, 0)
    except Exception:
        avg_center = (0, 0, 0)

    # 同时使用多组阈值做候选，合并所有找到的 blob
    blobs = []
    for th in TH_BALLS:
        try:
            # 调整为中等门限：兼顾暗光召回与背景噪声抑制，关闭 merge
            # 自转时运动模糊会让 blob 变碎/变淡：适当放宽像素/面积门限提高召回。
            bbs = orig.find_blobs([th], pixels_threshold=12, area_threshold=50, merge=False)
            if bbs:
                for b in bbs:
                    blobs.append(b)
        except Exception:
            pass

    if blobs:
        # 过滤掉接近整帧的伪目标（宽或高占比 >= 90%）以及过大的像素面积
        blobs = [bb for bb in blobs if not (bb.w() >= FRAME_W * 0.9 or bb.h() >= FRAME_H * 0.9) and bb.pixels() < FRAME_AREA * 0.85]
        # 再过滤：明显的细长黄线/胶带（横向或纵向细长条）
        blobs = [
            bb
            for bb in blobs
            if not ((bb.w() >= 90 and bb.h() <= 8) or (bb.h() >= 90 and bb.w() <= 8) or (bb.w() >= 140 and bb.h() <= 14) or (bb.h() >= 140 and bb.w() <= 14))
        ]
        if DEBUG_PRINT:
            now_dbg = time.ticks_ms()
            if time.ticks_diff(now_dbg, _dbg_last_ms) >= DEBUG_PRINT_EVERY_MS:
                print("blobs found total:", len(blobs))
                _dbg_last_ms = now_dbg
        # 不绘制候选蓝框（只在最终选中时绘制红框）

    

    # 黄线范围：允许短暂沿用上一帧，减少抖动
    field_bounds = detect_field_bounds(orig)
    field_held = False
    if field_bounds is None:
        if last_field_bounds is not None and field_lost_count < FIELD_HOLD_FRAMES:
            field_bounds = last_field_bounds
            field_lost_count += 1
            field_held = True
        else:
            last_field_bounds = None
            field_lost_count = 0
    else:
        last_field_bounds = field_bounds
        field_lost_count = 0

    obj = None
    if blobs:
        # 按分数从高到低排序，优先选择中心颜色偏绿的候选
        candidates = sorted(blobs, key=lambda bb: _blob_score(bb), reverse=True)
        selected = None
        for b_cand in candidates:
            try:
                # 在原始图上做 3x3 中心平均采样以减少噪声
                sr = sg = sb = sc = 0
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        try:
                            p = orig.get_pixel(b_cand.cx() + dx, b_cand.cy() + dy)
                            if isinstance(p, tuple):
                                r, g, b = p
                            else:
                                r = g = b = p
                            sr += r; sg += g; sb += b; sc += 1
                        except Exception:
                            pass
                if sc:
                    avg_r = sr // sc; avg_g = sg // sc; avg_b = sb // sc
                else:
                    avg_r = avg_g = avg_b = 0
                # 偏绿判定：G 明显高于 R/B 且亮度不为 0
                if (avg_g > avg_r + 12 and avg_g > avg_b + 12 and avg_g > 40):
                    selected = b_cand
                    # 打印中心颜色用于调参观察
                    if DEBUG_PRINT:
                        print("blob center rgb:", (avg_r, avg_g, avg_b))
                    break
            except Exception:
                pass

        # 如果没有通过颜色检验的候选，退回最高分候选
        if selected is None:
            selected = candidates[0]
            try:
                rgb = orig.get_pixel(selected.cx(), selected.cy())
                if DEBUG_PRINT:
                    print("blob center rgb:", rgb)
            except Exception:
                pass

        b = selected
        try:
            # 不同固件对关键字参数支持不一致，尽量用位置参数
            img.draw_rectangle(b.rect(), (255, 0, 0))
            img.draw_cross(b.cx(), b.cy(), (255, 0, 0))
        except Exception:
            pass
        obj = {
            "x": int(b.cx()),
            "y": int(b.cy()),
            "w": int(b.w()),
            "h": int(b.h()),
            "size": int(max(b.w(), b.h())),
            "method": "blob",
            "held": False,
        }
    # 如果基于颜色的 blob 没有找到，尝试基于形状的圆检测回退
    if obj is None:
        try:
            # 参数可按现场微调：threshold 越低越宽松，r_min/r_max 控制圆半径范围
            # 略微放宽圆检测阈值以增加回退检测的鲁棒性
            circles = orig.find_circles(threshold=1800, x_margin=10, y_margin=10, r_margin=10, r_min=6, r_max=200, r_step=2)
            if circles:
                # 选最大的圆作为候选，回退时用红色矩形表示（与 blob 一致）
                c = max(circles, key=lambda x: x.r())
                if DEBUG_PRINT:
                    print("circle candidate:", c.x(), c.y(), c.r())

                # 颜色二次确认：避免把场地反光/轮子/杂物的圆形误当成球
                ok_color = False
                try:
                    sr = sg = sb = sc = 0
                    for dx in (-1, 0, 1):
                        for dy in (-1, 0, 1):
                            try:
                                p = orig.get_pixel(c.x() + dx, c.y() + dy)
                                if isinstance(p, tuple):
                                    r, g, b = p
                                else:
                                    r = g = b = p
                                sr += r; sg += g; sb += b; sc += 1
                            except Exception:
                                pass
                    if sc:
                        avg_r = sr // sc; avg_g = sg // sc; avg_b = sb // sc
                    else:
                        avg_r = avg_g = avg_b = 0
                    ok_color = (avg_g > avg_r + 12 and avg_g > avg_b + 12 and avg_g > 40)
                except Exception:
                    ok_color = False

                if not ok_color:
                    circles = None
                    raise Exception("circle color reject")

                bx = int(c.x() - c.r())
                by = int(c.y() - c.r())
                bw = int(c.r() * 2)
                bh = int(c.r() * 2)
                try:
                    img.draw_rectangle(bx, by, bw, bh, (255, 0, 0))
                    img.draw_cross(c.x(), c.y(), (255, 0, 0))
                except Exception:
                    pass
                obj = {
                    "x": int(c.x()),
                    "y": int(c.y()),
                    "w": bw,
                    "h": bh,
                    "size": int(max(bw, bh)),
                    "method": "circle",
                    "held": False,
                }
        except Exception:
            pass

    # 时间平滑 + 丢失短保持，减小抖动
    if obj is not None:
        if last_obj is not None:
            obj["x"] = int(last_obj["x"] * (1.0 - SMOOTH_ALPHA) + obj["x"] * SMOOTH_ALPHA)
            obj["y"] = int(last_obj["y"] * (1.0 - SMOOTH_ALPHA) + obj["y"] * SMOOTH_ALPHA)
            obj["w"] = int(last_obj["w"] * (1.0 - SMOOTH_ALPHA) + obj["w"] * SMOOTH_ALPHA)
            obj["h"] = int(last_obj["h"] * (1.0 - SMOOTH_ALPHA) + obj["h"] * SMOOTH_ALPHA)
            obj["size"] = int(max(obj["w"], obj["h"]))
        last_obj = obj
        lost_count = 0
    else:
        if last_obj is not None and lost_count < LOST_HOLD_FRAMES:
            obj = last_obj.copy()
            try:
                obj["held"] = True
            except Exception:
                pass
            lost_count += 1
        else:
            last_obj = None

    # 出界判断：优先用黄线边界范围，其次退化为靠近画面边缘
    out_of_field = False
    if obj:
        if field_bounds is not None:
            try:
                # Defensive: tolerate (x0,y0,x1,y1,...) or malformed bounds.
                fx0, fy0, fx1, fy1 = field_bounds[0], field_bounds[1], field_bounds[2], field_bounds[3]
            except Exception:
                fx0 = fy0 = 0
                fx1 = FRAME_W
                fy1 = FRAME_H
            x = int(obj.get("x", 160))
            y = int(obj.get("y", 120))
            inner = int(FIELD_INNER_MARGIN)
            if x <= fx0 + inner or x >= fx1 - inner or y <= fy0 + inner or y >= fy1 - inner:
                out_of_field = True
        else:
            out_of_field = bool(
                obj and (
                    obj["x"] <= OUT_MARGIN
                    or obj["x"] >= 320 - OUT_MARGIN
                    or obj["y"] <= OUT_MARGIN
                    or obj["y"] >= 240 - OUT_MARGIN
                )
            )

    payload = {
        "frame": {"w": 320, "h": 240},
        "object": obj,
        "zone": None,
        "objects_count": 1 if obj else 0,
        "object_out_of_field": bool(out_of_field),
        "field_bounds": list(field_bounds) if field_bounds is not None else None,
        "field_held": bool(field_held),
    }

    json_str = json.dumps(payload)
    # UART.write 在不少固件上只接受 bytes
    try:
        json_bytes = json_str.encode("utf-8")
    except Exception:
        json_bytes = bytes(json_str)

    if WIRE_USE_SP_CRC:
        crc = crc16_ibm(json_bytes)
        try:
            crc_hex = ("%04X" % crc).encode("ascii")
        except Exception:
            crc_hex = bytes("%04X" % crc)
        tx_bytes = b"SP|" + json_bytes + b"|" + crc_hex + b"\n"
    else:
        tx_bytes = json_bytes + b"\n"

    for uid, u in uarts:
        try:
            u.write(tx_bytes)
            uart_ok += 1
        except Exception:
            uart_fail += 1

    # 屏幕可视心跳：优先文字，失败则退化为闪烁方块
    try:
        # 调试输出会很刷屏，这里只在屏幕显示发送计数
        try:
            img.draw_string(2, 2, "sent:{} obj:{}".format(sent_count, 1 if obj else 0), (255, 255, 255), 1)
        except Exception:
            # 兼容部分固件的签名（不支持 scale 或只接受 4 参数）
            img.draw_string(2, 2, "sent:{} obj:{}".format(sent_count, 1 if obj else 0), (255, 255, 255))
        img.draw_cross(160, 120, (0, 255, 0))
    except Exception:
        c = 255 if (sent_count % 10) < 5 else 40
        # 任何屏幕绘图失败都不应影响 UART 发送
        try:
            img.draw_rectangle(2, 2, 24, 16, (c, c, c), 1, True)
        except Exception:
            pass

    sent_count += 1
    now = time.ticks_ms()
    if time.ticks_diff(now, last_heartbeat) >= 1000:
        print(
            "===VISION_ALIVE===",
            "sent=", sent_count,
            "obj=", obj is not None,
            "uarts=", [uid for uid, _ in uarts],
            "uart_write_ok=", uart_ok,
            "uart_write_fail=", uart_fail,
            "avg_center=", avg_center,
        )
        last_heartbeat = now

    time.sleep_ms(40)  # 25Hz
