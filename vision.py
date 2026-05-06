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

print("VISION_SCRIPT_BLUE_SQUARE_V8")


# 调试开关：OpenART 上频繁 print 会显著降帧，反而更容易“球在但识别不出”。
DEBUG_PRINT = True
DEBUG_PRINT_EVERY_MS = 800
_dbg_last_ms = time.ticks_ms()


def _dbg_print(*args):
    """节流打印：避免 OpenART 高频 print 降帧导致漏检。"""
    global _dbg_last_ms
    if not DEBUG_PRINT:
        return
    try:
        now = time.ticks_ms()
        if time.ticks_diff(now, _dbg_last_ms) < int(DEBUG_PRINT_EVERY_MS):
            return
        _dbg_last_ms = now
    except Exception:
        # 若固件不支持 ticks_diff/ticks_ms，就退化为不节流
        pass
    try:
        print(*args)
    except Exception:
        pass


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
AUTO_GAIN = False
# 关闭自动增益以避免颜色漂移影响阈值稳定性（现场测试更稳定）
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
ENABLE_BALL_DETECT = True

# 蓝色方块阈值：现场测试结果
# 推荐使用你提供的阈值（LAB 空间）
# 调整为捕捉偏紫蓝色（场地较亮时）
# 蓝色通常对应 LAB 的 b 为负值（偏蓝），木地板/偏黄背景的 b 往往偏正。
# 这里把 B_max 收紧，减少背景误检；如现场偏紫导致漏检，再放宽到 8~12。
TH_BLUE = [
    # 严格优先：减少木地板/偏黄背景
    (15, 85, -70, 55, -80, 6),
    # 精准阈值（基于你测得的蓝方块）
    (21, 75, -51, 46, -69, 22),
    # 偏紫/光照变化时的宽松备选
    (15, 80, -55, 50, -75, 26),
]
ENABLE_BLUE_BLOCK_DETECT = False

# 白色区域阈值：用于识别物体中间的白色数字/白色标记
TH_WHITE = [
    (70, 100, -8, 8, -8, 8),
    (60, 100, -12, 12, -12, 12),
]
ENABLE_WHITE_DETECT = False

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

# 蓝方块分支的额外门限（专门用来压木地板误检）
# 白色数字会把蓝色区域“挖空”，导致蓝色像素变少、blob 变碎；这里适当放宽门限。
BLUE_PIXELS_THRESHOLD = 6
BLUE_AREA_THRESHOLD = 18
BLUE_MIN_PIXELS = 30

# 关键：木地板/偏黄背景在你日志里常见 b_mean≈11~12，
# 之前的 <=12 会把它们放行；这里收紧到 6。
BLUE_LAB_B_MEAN_MAX = 6   # 主要门槛：b_mean <= 6 认为偏蓝/偏紫蓝
BLUE_LAB_B_HARD_MAX = 10  # 兜底上限：b_mean <= 10 才有资格再结合 a_mean 放行
BLUE_LAB_A_MIN_IF_B_POS = 12  # 当 b_mean 偏正但仍在 HARD_MAX 内时，需要 a_mean 足够大才认为“紫蓝”

# 边缘 LAB 采样（避开中间白色数字）
BLUE_BORDER_B_MEAN_MAX = 6
BLUE_BORDER_MIN_HITS = 4


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


def _square_score(b):
    # 更强调接近正方形与填充率，适用于方块评分
    w = max(1, int(b.w()))
    h = max(1, int(b.h()))
    size = max(w, h)
    if size < MIN_SIZE or size > MAX_SIZE:
        return -1

    aspect = w / float(h)
    if aspect < 0.6 or aspect > 1.6:
        return -1

    fill_ratio = b.pixels() / float(w * h)
    if fill_ratio < 0.12:
        return -1

    square_penalty = abs(aspect - 1.0)
    return b.pixels() * (1.0 - 0.6 * square_penalty) * (0.85 + 0.15 * fill_ratio)


def _blue_square_score(b):
    """蓝方块候选评分：比 _square_score 更严格，用于压假阳性（木地板纹理等）。"""
    try:
        x = int(b.x())
        y = int(b.y())
        w = max(1, int(b.w()))
        h = max(1, int(b.h()))
    except Exception:
        return -1

    size = max(w, h)
    if size < MIN_SIZE or size > MAX_SIZE:
        return -1

    # 排除特别小的碎片（地板纹理常见）
    try:
        if int(b.pixels()) < BLUE_MIN_PIXELS:
            return -1
    except Exception:
        pass

    # 贴边目标更容易是地面/边缘噪声
    try:
        if x <= 1 or y <= 1 or (x + w) >= (FRAME_W - 2) or (y + h) >= (FRAME_H - 2):
            return -1
    except Exception:
        pass

    aspect = w / float(h)
    # 蓝方块更强调接近正方形
    if aspect < 0.72 or aspect > 1.38:
        return -1

    try:
        fill_ratio = b.pixels() / float(w * h)
    except Exception:
        return -1

    # 白色数字会显著降低 fill_ratio（蓝色只剩边缘/一部分），
    # 因此这里放宽 fill_ratio 门限；木地板误检主要靠 LAB 颜色确认压制。
    min_fill = 0.07
    if (w * h) >= 1800:
        min_fill = 0.09
    if (w * h) >= 4500:
        min_fill = 0.11
    if fill_ratio < min_fill:
        return -1

    square_penalty = abs(aspect - 1.0)
    return b.pixels() * (1.0 - 0.75 * square_penalty) * (0.90 + 0.10 * fill_ratio)


def _roi_lab_means(img_src, blob):
    """返回 (l_mean, a_mean, b_mean) 或 None。"""
    try:
        st = img_src.get_statistics(roi=blob.rect())
        l_mean = int(st.l_mean())
        a_mean = int(st.a_mean())
        b_mean = int(st.b_mean())
        return (l_mean, a_mean, b_mean)
    except Exception:
        return None


def _is_blue_by_roi_lab(img_src, blob):
    """基于整块 ROI 的 LAB 均值做蓝色确认（会被白字稀释，但可作为补充）。"""
    means = _roi_lab_means(img_src, blob)
    if means is None:
        return False
    try:
        _l, _a, _b = int(means[0]), int(means[1]), int(means[2])
    except Exception:
        return False

    # 典型蓝/紫蓝：b 应该明显偏负；如果 b 偏正（偏黄），必须非常谨慎
    if _b <= int(BLUE_LAB_B_MEAN_MAX):
        return True
    if _b <= int(BLUE_LAB_B_HARD_MAX) and _a >= int(BLUE_LAB_A_MIN_IF_B_POS):
        return True
    return False


def _is_blue_by_border_lab(img_src, blob):
    """用边缘小 ROI 的 LAB b_mean 确认蓝色（避开中心白色数字）。"""
    try:
        x = int(blob.x())
        y = int(blob.y())
        w = max(1, int(blob.w()))
        h = max(1, int(blob.h()))
    except Exception:
        return False

    # 8 个边缘点（不取中心），每个点用 5x5 ROI 做统计，抗噪声
    pts = [
        (x + 2, y + 2),
        (x + w // 2, y + 2),
        (x + w - 3, y + 2),
        (x + 2, y + h // 2),
        (x + w - 3, y + h // 2),
        (x + 2, y + h - 3),
        (x + w // 2, y + h - 3),
        (x + w - 3, y + h - 3),
    ]
    hits = 0
    total = 0
    for px, py in pts:
        try:
            # 5x5 ROI around point
            rx = max(0, min(FRAME_W - 5, px - 2))
            ry = max(0, min(FRAME_H - 5, py - 2))
            st = img_src.get_statistics(roi=(rx, ry, 5, 5))
            bb = int(st.b_mean())
            total += 1
            if bb <= int(BLUE_BORDER_B_MEAN_MAX):
                hits += 1
        except Exception:
            pass

    return total >= 5 and hits >= int(BLUE_BORDER_MIN_HITS)


def _is_blue_by_border(img_src, blob):
    """用方块边缘而不是中心做蓝色确认，避免白色数字干扰。"""
    try:
        x = int(blob.x())
        y = int(blob.y())
        w = max(1, int(blob.w()))
        h = max(1, int(blob.h()))
    except Exception:
        return False

    # 取四边和四角附近的 9 个点，避开中心白色数字区域
    pts = [
        (x + 2, y + 2),
        (x + w // 2, y + 2),
        (x + w - 3, y + 2),
        (x + 2, y + h // 2),
        (x + w - 3, y + h // 2),
        (x + 2, y + h - 3),
        (x + w // 2, y + h - 3),
        (x + w - 3, y + h - 3),
        (x + w // 2, y + h // 2),
    ]

    blue_hits = 0
    total = 0
    for px, py in pts:
        try:
            p = img_src.get_pixel(px, py)
            if isinstance(p, tuple):
                r, g, b = p
            else:
                r = g = b = p
            total += 1
            # 蓝色确认（容忍偏紫/偏暗，但要明显“蓝占优”）：
            # - 场地木地板通常 R/G 更高，b 不会占优
            # - 蓝方块在偏暗时 b 不一定很高，因此用 dominance 条件
            dom = b - (r if r >= g else g)
            if (b >= 25 and dom >= 6) or (b >= 45 and dom >= 2):
                blue_hits += 1
        except Exception:
            pass

    # 边缘点采样容易受反光/噪声影响；这里只作为“没有 LAB 统计时”的兜底。
    return total >= 5 and blue_hits >= 4


def _is_white_blob(blob):
    try:
        w = max(1, int(blob.w()))
        h = max(1, int(blob.h()))
    except Exception:
        return False
    # 白色通常亮度高、颜色通道差异小
    if w < 2 or h < 2:
        return False
    fill_ratio = blob.pixels() / float(w * h)
    if fill_ratio < 0.10:
        return False
    return True


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

    # 网球检测：优先使用 TH_BALLS 的颜色 blob，再用圆检测做回退
    blobs = []
    obj = None
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
    if ENABLE_BALL_DETECT:
        for th in TH_BALLS:
            try:
                found = orig.find_blobs([th], pixels_threshold=35, area_threshold=35, merge=False)
                if found:
                    for b in found:
                        blobs.append(b)
            except Exception:
                pass
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
                        _dbg_print("blob center rgb:", (avg_r, avg_g, avg_b))
                    break
            except Exception:
                pass

        # 如果没有通过颜色检验的候选，退回最高分候选
        if selected is None:
            selected = candidates[0]
            try:
                rgb = orig.get_pixel(selected.cx(), selected.cy())
                if DEBUG_PRINT:
                    _dbg_print("blob center rgb:", rgb)
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
                    _dbg_print("circle candidate:", c.x(), c.y(), c.r())

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

    # ===== 白色数字/标记检测（优先） =====
    if obj is None and ENABLE_WHITE_DETECT:
      try:
        white_blobs = []
        for th in TH_WHITE:
            try:
                bbs = orig.find_blobs([th], pixels_threshold=10, area_threshold=10, merge=False)
                if bbs:
                    for b in bbs:
                        white_blobs.append(b)
            except Exception:
                pass

        if white_blobs:
            white_blobs = [bb for bb in white_blobs if bb.pixels() < FRAME_AREA * 0.20]
            if DEBUG_PRINT:
                try:
                    _dbg_print("DBG_WHITE_COUNT:", len(white_blobs))
                except Exception:
                    pass
            white_candidates = sorted(white_blobs, key=lambda bb: bb.pixels(), reverse=True)
            if white_candidates:
                sel = white_candidates[0]
                if _is_white_blob(sel):
                    try:
                        img.draw_rectangle(sel.rect(), (255, 255, 255))
                        img.draw_cross(sel.cx(), sel.cy(), (255, 255, 255))
                    except Exception:
                        pass
                    obj = {
                        "x": int(sel.cx()),
                        "y": int(sel.cy()),
                        "w": int(sel.w()),
                        "h": int(sel.h()),
                        "size": int(max(sel.w(), sel.h())),
                        "method": "white_blob",
                        "color": "white",
                    }
      except Exception:
        pass

    # ===== 蓝色方块检测（回退分支） =====
    if obj is None and ENABLE_BLUE_BLOCK_DETECT:
      try:
        blue_cands = []
        for th_i, th in enumerate(TH_BLUE):
            try:
                # 适度提高门限，减少地板纹理/噪声触发
                # 允许 merge：白色数字会让蓝色边缘碎成多块，merge 能把碎块合并成更完整的外接框
                try:
                    bbs = orig.find_blobs([th], pixels_threshold=BLUE_PIXELS_THRESHOLD, area_threshold=BLUE_AREA_THRESHOLD, merge=True)
                except Exception:
                    bbs = orig.find_blobs([th], pixels_threshold=BLUE_PIXELS_THRESHOLD, area_threshold=BLUE_AREA_THRESHOLD, merge=False)
                if bbs:
                    for b in bbs:
                        # 先用更严格的方块评分过滤
                        s = _blue_square_score(b)
                        if s > 0:
                            # 颜色二次确认：优先用“边缘 LAB 统计”避免白色数字影响
                            ok = False
                            try:
                                ok = _is_blue_by_border_lab(orig, b)
                            except Exception:
                                ok = False
                            if not ok:
                                # 退化：整块 ROI 的 LAB 均值（会被白字稀释，但能拒绝偏黄背景）
                                ok = _is_blue_by_roi_lab(orig, b)
                            if not ok:
                                # 最后兜底：RGB 边缘采样（风险更高）
                                # 仅在 ROI-LAB 也满足“基本不偏黄”的前提下才允许 RGB 兜底。
                                if not _is_blue_by_roi_lab(orig, b):
                                    continue
                                if not _is_blue_by_border(orig, b):
                                    continue

                            # 严格阈值略加权，优先选中
                            if th_i == 0:
                                s += 80
                            blue_cands.append((s, b))
            except Exception:
                pass

        if blue_cands:
            # 过滤掉超大blob
            try:
                blue_cands = [(s, bb) for (s, bb) in blue_cands if not (bb.w() >= FRAME_W * 0.9 or bb.h() >= FRAME_H * 0.9) and bb.pixels() < FRAME_AREA * 0.92]
            except Exception:
                pass
            if DEBUG_PRINT:
                try:
                    _dbg_print("DBG_BLUE_COUNT:", len(blue_cands))
                except Exception:
                    pass
            
            # 优先挑“更像方块 + 更像蓝色”的候选
            if blue_cands:
                # 先按严格评分排序（比单纯 pixels 更稳）
                try:
                    blue_cands = sorted(blue_cands, key=lambda it: it[0], reverse=True)
                except Exception:
                    pass

                # 注意：偏紫方块在 RGB 下不一定 "B 占优"，
                # 因此这里不再做二次 RGB 边缘确认（避免误杀真目标）。
                # 木地板误检主要由 LAB 阈值 + ROI b_mean 门限来压。
                sel = blue_cands[0][1] if blue_cands else None

                # 若都没通过边缘确认，则宁可不输出（避免误跟随木地板）
                if sel is None:
                    sel = None
                if DEBUG_PRINT:
                    try:
                        if sel is not None:
                            means = _roi_lab_means(orig, sel)
                            _dbg_print("DBG_BLUE_CAND rect:", sel.rect(), "pixels:", sel.pixels(), "w:", sel.w(), "h:", sel.h(), "lab_mean=", means)
                        else:
                            _dbg_print("DBG_BLUE_CAND: none passed border check")
                    except Exception:
                        pass
                
                if sel is not None:
                    try:
                        img.draw_rectangle(sel.rect(), (0, 0, 255))
                        img.draw_cross(sel.cx(), sel.cy(), (0, 0, 255))
                    except Exception:
                        pass

                    obj = {
                        "x": int(sel.cx()),
                        "y": int(sel.cy()),
                        "w": int(sel.w()),
                        "h": int(sel.h()),
                        "size": int(max(sel.w(), sel.h())),
                        "method": "blue_square",
                        "color": "blue",
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
        "objects_count": (1 if obj else 0),
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
