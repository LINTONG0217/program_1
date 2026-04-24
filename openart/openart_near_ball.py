"""OpenArt mini near-object color classifier with UART output.

Expected UART output examples:
- label=red,conf=0.93,cx=81,cy=64,pixels=2210
- label=green,conf=0.88,cx=77,cy=60,pixels=1840
- none

Tune the LAB thresholds below on your field before use.
"""

import time

try:
	import sensor  # type: ignore
	import image  # type: ignore
except Exception as exc:
	raise SystemExit("sensor/image unavailable: {}".format(exc))

try:
	from machine import UART  # type: ignore
except Exception:
	try:
		import pyb  # type: ignore
		UART = pyb.UART
	except Exception:
		UART = None


UART_ID = 4
UART_BAUDRATE = 115200
FRAME_SIZE = getattr(sensor, "QVGA", None)
PIXFORMAT = getattr(sensor, "RGB565", None)

# Limit the field of view to the lower-middle area where the car is already close
# to the object. This reduces background noise and stabilizes recognition.
ROI = (30, 40, 260, 160)

PIXELS_THRESHOLD = 120
AREA_THRESHOLD = 120
MERGE = True
X_STRIDE = 2
Y_STRIDE = 2
STABLE_FRAMES = 3
SEND_INTERVAL_MS = 120

THRESHOLDS = {
	"red": (25, 75, 15, 70, 15, 70),
	"green": (20, 70, -70, -20, 10, 70),
	"blue": (20, 80, -70, -10, 10, 70),
	"yellow": (40, 90, -10, 20, 40, 90),
}


def ticks_ms():
	fn = getattr(time, "ticks_ms", None)
	if callable(fn):
		return fn()
	return int(time.time() * 1000)


def ticks_diff(a, b):
	fn = getattr(time, "ticks_diff", None)
	if callable(fn):
		return fn(a, b)
	return a - b


def sleep_ms(ms):
	fn = getattr(time, "sleep_ms", None)
	if callable(fn):
		fn(ms)
	else:
		time.sleep(float(ms) / 1000.0)


def init_camera():
	sensor.reset()
	if PIXFORMAT is not None:
		sensor.set_pixformat(PIXFORMAT)
	if FRAME_SIZE is not None:
		sensor.set_framesize(FRAME_SIZE)
	try:
		sensor.set_auto_whitebal(False)
	except Exception:
		pass
	try:
		sensor.set_auto_gain(False)
	except Exception:
		pass
	try:
		sensor.set_auto_exposure(True)
	except Exception:
		pass
	sleep_ms(800)


def open_uart():
	if UART is None:
		return None
	try:
		return UART(UART_ID, UART_BAUDRATE)
	except Exception:
		try:
			return UART(UART_ID)
		except Exception:
			return None


def uart_write_line(uart, text):
	line = "{}\n".format(text)
	if uart is not None:
		try:
			uart.write(line)
		except Exception:
			pass
	print(line.strip())


def blob_score(blob):
	# Favor larger and more compact blobs.
	try:
		return int(blob.pixels()) + int(blob.area()) // 2
	except Exception:
		return 0


def detect_best_blob(img):
	best_label = None
	best_blob = None
	best_score = 0
	for label in THRESHOLDS:
		threshold = THRESHOLDS[label]
		try:
			blobs = img.find_blobs(
				[threshold],
				roi=ROI,
				pixels_threshold=PIXELS_THRESHOLD,
				area_threshold=AREA_THRESHOLD,
				merge=MERGE,
				x_stride=X_STRIDE,
				y_stride=Y_STRIDE,
			)
		except Exception:
			blobs = []
		for blob in blobs:
			score = blob_score(blob)
			if score > best_score:
				best_score = score
				best_label = label
				best_blob = blob
	return best_label, best_blob, best_score


def confidence_from_blob(blob, score):
	pixels = max(1, int(blob.pixels()))
	compact = float(pixels) / max(1.0, float(blob.w()) * float(blob.h()))
	conf = 0.45 + min(0.45, float(score) / 5000.0) + min(0.10, compact * 0.20)
	if conf > 0.99:
		conf = 0.99
	return conf


def draw_overlay(img, label, blob):
	try:
		img.draw_rectangle(blob.rect(), color=(255, 0, 0))
		img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))
		img.draw_string(blob.x(), max(0, blob.y() - 12), label, color=(255, 255, 255))
	except Exception:
		pass


def main():
	init_camera()
	uart = open_uart()
	clock = time.clock()
	last_sent_ms = 0
	last_seen_label = None
	stable_count = 0

	uart_write_line(uart, "openart_near_ball_start")

	while True:
		clock.tick()
		img = sensor.snapshot()
		try:
			img.draw_rectangle(ROI, color=(255, 255, 0))
		except Exception:
			pass

		label, blob, score = detect_best_blob(img)
		now = ticks_ms()

		if label is None or blob is None:
			last_seen_label = None
			stable_count = 0
			if ticks_diff(now, last_sent_ms) >= SEND_INTERVAL_MS:
				uart_write_line(uart, "none")
				last_sent_ms = now
			continue

		draw_overlay(img, label, blob)

		if label == last_seen_label:
			stable_count += 1
		else:
			last_seen_label = label
			stable_count = 1

		if stable_count < STABLE_FRAMES:
			continue

		if ticks_diff(now, last_sent_ms) < SEND_INTERVAL_MS:
			continue

		conf = confidence_from_blob(blob, score)
		uart_write_line(
			uart,
			"label={},conf={:.2f},cx={},cy={},pixels={}".format(
				label,
				conf,
				int(blob.cx()),
				int(blob.cy()),
				int(blob.pixels()),
			),
		)
		last_sent_ms = now


if __name__ == "__main__":
	main()
