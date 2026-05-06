"""Encoder odometry global-position navigation test.

Coordinate convention:
- Start pose is (0, 0), yaw 0.
- Forward is +x, right strafe is +y.
- Field size is 3.2m x 2.4m.
- Target is (1.6, 1.2).

Run:
    python -m mpremote run tests/hardware/manual_encoder_global_nav.py
"""

import math
import time
from machine import Pin, unique_id

from BSP.board_runtime import MainControl
from BSP.motor_actuator import Motor
from BSP.omni_chassis_driver import OmniChassis
from Module import config


FIELD_X_M = 3.2
FIELD_Y_M = 2.4
TARGET_X_M = 1.6
TARGET_Y_M = 1.2

# The test assumes the motor reducer output is about 100 rpm at full command.
# Position is still measured from encoder pulses; this value only caps test speed.
MOTOR_RPM = 100
MAX_SPEED = 38
MIN_SPEED = 9
KP_XY = 34.0
ARRIVE_TOLERANCE_M = 0.045
ARRIVE_STABLE_MS = 500
TIMEOUT_MS = 25000

YAW_KP = 1.2
YAW_KD = 0.10
YAW_MAX_SPEED = 22
YAW_DEADBAND_DEG = 1.0
YAW_SIGN = float(getattr(config, "HEADING_LOCK_YAW_SIGN", 1.0))

PRINT_MS = 200
DISPLAY_MS = 120
LOOP_MS = 20

WHITE = 0xFFFF
BLACK = 0x0000
GRID = 0xCE59
FIELD = 0x07E0
TARGET = 0xF800
ROBOT = 0x001F


def clamp(value, low, high):
	return max(low, min(high, value))


def angle_error(target, current):
	err = float(target) - float(current)
	while err > 180.0:
		err -= 360.0
	while err < -180.0:
		err += 360.0
	return err


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
	print("press {} to start encoder global nav".format(pin_name))
	while key.value() != 0:
		time.sleep_ms(20)
	time.sleep_ms(60)
	while key.value() == 0:
		time.sleep_ms(20)
	print("{} start".format(pin_name))


def build_chassis():
	motor_fl = Motor(*config.MOTOR_FL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fl", False))
	motor_fr = Motor(*config.MOTOR_FR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fr", False))
	motor_bl = Motor(*config.MOTOR_BL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("bl", False))
	motor_br = Motor(*config.MOTOR_BR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("br", False))
	return OmniChassis(motor_fl, motor_fr, motor_bl, motor_br, config=config)


class EncoderPose:
	def __init__(self, pulse_per_meter):
		self.ppm = float(pulse_per_meter or 1.0)
		self.x = 0.0
		self.y = 0.0
		self.distance = 0.0
		self.last_counts = None

	def reset(self, snapshot):
		self.x = 0.0
		self.y = 0.0
		self.distance = 0.0
		self.last_counts = {name: int(snapshot[name]["count"]) for name in ("fl", "fr", "bl", "br")}

	def update(self, snapshot):
		if self.last_counts is None:
			self.reset(snapshot)
			return self.x, self.y

		d = {}
		for name in ("fl", "fr", "bl", "br"):
			count = int(snapshot[name]["count"])
			d[name] = count - self.last_counts[name]
			self.last_counts[name] = count

		# Matches the calibrated hardware direction:
		# forward -> [+, -, +, -], right -> [-, -, +, +].
		dx_pulse = (d["fl"] - d["fr"] + d["bl"] - d["br"]) * 0.25
		dy_pulse = (-d["fl"] - d["fr"] + d["bl"] + d["br"]) * 0.25
		dx = dx_pulse / self.ppm
		dy = dy_pulse / self.ppm
		self.x += dx
		self.y += dy
		self.distance += math.sqrt(dx * dx + dy * dy)
		return self.x, self.y


class FieldDisplay:
	def __init__(self, board):
		self.lcd = getattr(board.lcd, "lcd_dev", None)
		if self.lcd is None:
			self.lcd = getattr(board.lcd, "map", None)
			self.lcd = getattr(self.lcd, "lcd", None)
		self.left = 10
		self.top = 25
		self.width = 220
		self.height = 165
		self.last_px = None
		self.last_py = None
		self.available = self.lcd is not None and hasattr(self.lcd, "line")

	def color(self, fg, bg=WHITE):
		if hasattr(self.lcd, "color"):
			try:
				self.lcd.color(fg, bg)
			except TypeError:
				self.lcd.color(fg)

	def line(self, x1, y1, x2, y2, color):
		if not self.available:
			return
		self.color(color)
		self.lcd.line(int(x1), int(y1), int(x2), int(y2))

	def clear(self):
		if self.available and hasattr(self.lcd, "clear"):
			self.lcd.clear(WHITE)

	def world_to_screen(self, x, y):
		x = clamp(float(x), 0.0, FIELD_X_M)
		y = clamp(float(y), 0.0, FIELD_Y_M)
		px = self.left + int(round(x / FIELD_X_M * self.width))
		py = self.top + self.height - int(round(y / FIELD_Y_M * self.height))
		return px, py

	def draw_circle(self, cx, cy, radius, color):
		r2 = radius * radius
		for dy in range(-radius, radius + 1):
			dx = int(math.sqrt(max(0, r2 - dy * dy)))
			self.line(cx - dx, cy + dy, cx + dx, cy + dy, color)

	def erase_circle(self, cx, cy, radius):
		self.draw_circle(cx, cy, radius + 1, WHITE)

	def draw_static(self):
		if not self.available:
			return
		self.clear()
		self.line(self.left, self.top, self.left + self.width, self.top, FIELD)
		self.line(self.left, self.top + self.height, self.left + self.width, self.top + self.height, FIELD)
		self.line(self.left, self.top, self.left, self.top + self.height, FIELD)
		self.line(self.left + self.width, self.top, self.left + self.width, self.top + self.height, FIELD)

		for i in range(1, 8):
			x = self.left + int(round(i * 0.4 / FIELD_X_M * self.width))
			self.line(x, self.top, x, self.top + self.height, GRID)
		for i in range(1, 6):
			y = self.top + self.height - int(round(i * 0.4 / FIELD_Y_M * self.height))
			self.line(self.left, y, self.left + self.width, y, GRID)

		tx, ty = self.world_to_screen(TARGET_X_M, TARGET_Y_M)
		self.line(tx - 5, ty, tx + 5, ty, TARGET)
		self.line(tx, ty - 5, tx, ty + 5, TARGET)

	def update_robot(self, x, y):
		if not self.available:
			return
		if self.last_px is not None:
			self.erase_circle(self.last_px, self.last_py, 5)
			self.draw_static()
		px, py = self.world_to_screen(x, y)
		self.draw_circle(px, py, 5, ROBOT)
		self.last_px = px
		self.last_py = py


def yaw_from_imu(board):
	data = board.read_imu()
	gyro = data.get("gyro") if data else None
	if gyro and len(gyro) >= 3:
		return float(gyro[2])
	return 0.0


def main():
	print("board uid:", unique_id())
	print("script version: manual_encoder_global_nav_v2")
	print("field=({:.1f},{:.1f}) target=({:.2f},{:.2f}) motor_rpm={}".format(FIELD_X_M, FIELD_Y_M, TARGET_X_M, TARGET_Y_M, MOTOR_RPM))

	board = MainControl(config)
	board.init()
	chassis = build_chassis()
	chassis.attach_feedback(encoders=None, imu=None)

	cfg = getattr(chassis, "cfg", None)
	if cfg:
		cfg.CHASSIS_CMD_DEADBAND = 0
		cfg.CHASSIS_MAX_VX_STEP = 6
		cfg.CHASSIS_MAX_VY_STEP = 6
		cfg.CHASSIS_MAX_VW_STEP = 8
		cfg.CHASSIS_ENABLE_CLOSED_LOOP = False

	display = FieldDisplay(board)
	display.draw_static()

	wait_c14_start()

	pose = EncoderPose(getattr(config, "PULSE_PER_METER", 1.0))
	pose.reset(board.read_encoder_snapshot())

	yaw = 0.0
	last_ms = time.ticks_ms()
	start_ms = last_ms
	last_print = last_ms
	last_display = last_ms
	stable_since = None

	try:
		while True:
			now = time.ticks_ms()
			dt = max(1, time.ticks_diff(now, last_ms)) / 1000.0
			last_ms = now

			gz = yaw_from_imu(board)
			yaw = (yaw + gz * dt) % 360.0
			x, y = pose.update(board.read_encoder_snapshot())

			ex = TARGET_X_M - x
			ey = TARGET_Y_M - y
			dist = math.sqrt(ex * ex + ey * ey)

			if dist <= ARRIVE_TOLERANCE_M:
				chassis.stop()
				if stable_since is None:
					stable_since = now
				elif time.ticks_diff(now, stable_since) >= ARRIVE_STABLE_MS:
					print("target reached x={:.3f} y={:.3f} dist={:.3f} yaw={:.2f}".format(x, y, dist, yaw))
					break
			else:
				stable_since = None
				vx = clamp(ex * KP_XY, -MAX_SPEED, MAX_SPEED)
				vy = clamp(ey * KP_XY, -MAX_SPEED, MAX_SPEED)
				if abs(vx) > 0 and abs(vx) < MIN_SPEED:
					vx = MIN_SPEED if vx > 0 else -MIN_SPEED
				if abs(vy) > 0 and abs(vy) < MIN_SPEED:
					vy = MIN_SPEED if vy > 0 else -MIN_SPEED

				yaw_err = angle_error(0.0, yaw)
				vw = 0.0
				if abs(yaw_err) > YAW_DEADBAND_DEG:
					vw = clamp((yaw_err * YAW_KP + gz * YAW_KD) * YAW_SIGN, -YAW_MAX_SPEED, YAW_MAX_SPEED)
				chassis.move(vx, vy, vw)

			if time.ticks_diff(now, last_display) >= DISPLAY_MS:
				last_display = now
				display.update_robot(x, y)

			if time.ticks_diff(now, last_print) >= PRINT_MS:
				last_print = now
				print("pose x={:.3f} y={:.3f} dist={:.3f} yaw={:.2f} total={:.3f}".format(x, y, dist, yaw, pose.distance))

			if time.ticks_diff(now, start_ms) >= TIMEOUT_MS:
				chassis.stop()
				print("timeout x={:.3f} y={:.3f} dist={:.3f} yaw={:.2f}".format(x, y, dist, yaw))
				break

			time.sleep_ms(LOOP_MS)
	finally:
		chassis.stop()
		display.update_robot(pose.x, pose.y)
		print("encoder global nav done")


if __name__ == "__main__":
	main()
