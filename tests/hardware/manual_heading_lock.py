"""Manual IMU heading-lock test for the main controller.

Run from PC, with the board connected:

    python -m mpremote run tests/hardware/manual_heading_lock.py

The car drives slowly forward while locking the yaw angle measured at start.
Keep the first run slow and safe. If the car corrects in the wrong direction,
flip HEADING_LOCK_YAW_SIGN in Module/config.py.
"""

import time

from machine import Pin

from BSP.board_runtime import MainControl
from BSP.motor_actuator import Motor
from BSP.omni_chassis_driver import OmniChassis
from Module import config

try:
	from seekfree import IMU963RA
except Exception:
	IMU963RA = None


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

	print("press {} to start heading-lock test".format(pin_name))
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


def prepare_chassis(chassis):
	cfg = getattr(chassis, "cfg", None)
	if not cfg:
		return
	cfg.CHASSIS_CMD_DEADBAND = 0
	cfg.CHASSIS_MAX_VX_STEP = 100
	cfg.CHASSIS_MAX_VY_STEP = 100
	cfg.CHASSIS_MAX_VW_STEP = 100
	cfg.CHASSIS_ENABLE_CLOSED_LOOP = False


def gyro_z_dps(imu_data):
	gyro = imu_data.get("gyro") if imu_data else None
	if isinstance(gyro, (tuple, list)) and len(gyro) >= 3:
		return float(gyro[2])
	return 0.0


class DirectIMU963RA:
	def __init__(self):
		self.dev = IMU963RA() if IMU963RA else None
		self.gyro_offset = 0.0

	def _raw(self):
		if self.dev is None:
			return None
		reader = getattr(self.dev, "read", None)
		if callable(reader):
			try:
				reader()
			except Exception:
				pass
		getter = getattr(self.dev, "get", None)
		if callable(getter):
			try:
				return getter()
			except Exception:
				return None
		return None

	def calibrate(self):
		if self.dev is None:
			return
		total = 0.0
		count = int(getattr(config, "GYRO_CALIBRATION_SAMPLES", 100))
		for _ in range(int(getattr(config, "GYRO_CALIBRATION_WARMUP", 20))):
			self._raw()
			time.sleep_ms(5)
		for _ in range(max(1, count)):
			raw = self._raw()
			total += raw[5] if raw and len(raw) > 5 else 0.0
			time.sleep_ms(10)
		self.gyro_offset = total / max(1, count)

	def read(self):
		raw = self._raw()
		if raw and len(raw) > 7:
			gz = (raw[5] - self.gyro_offset) * getattr(config, "GYRO_RAW_TO_DPS", 0.07)
			return {"gyro": (0.0, 0.0, gz), "mag": (raw[6], raw[7], raw[8] if len(raw) > 8 else 0), "ok": True, "raw": raw}
		return {"gyro": None, "mag": None, "ok": False, "raw": raw}


def ensure_imu(board):
	try:
		sample = board.read_imu()
		if isinstance(sample, dict) and sample.get("ok"):
			print("imu source: MainControl")
			return board, board.read_imu, getattr(board.imu, "calibrate", None), getattr(board.imu, "imu", None)
	except Exception:
		pass

	direct = DirectIMU963RA()
	if direct.dev is not None:
		print("imu source: direct seekfree.IMU963RA")
		return direct, direct.read, direct.calibrate, direct.dev

	print("imu source: missing")
	return board, board.read_imu, getattr(board.imu, "calibrate", None), None


def print_startup_diagnostics(imu_owner, imu_read, imu_backend, chassis):
	print("imu backend:", "ok" if imu_backend is not None else "missing")
	print("gyro_offset:", getattr(imu_owner, "gyro_offset", None))
	try:
		sample = imu_read()
	except Exception as e:
		sample = {"error": repr(e)}
	print("imu sample:", sample)
	print("motor pins:")
	print("fl", getattr(chassis.fl, "pwm_pin", "?"), getattr(chassis.fl, "dir_pin", "?"), "rev=", getattr(chassis.fl, "reverse", False))
	print("fr", getattr(chassis.fr, "pwm_pin", "?"), getattr(chassis.fr, "dir_pin", "?"), "rev=", getattr(chassis.fr, "reverse", False))
	print("bl", getattr(chassis.bl, "pwm_pin", "?"), getattr(chassis.bl, "dir_pin", "?"), "rev=", getattr(chassis.bl, "reverse", False))
	print("br", getattr(chassis.br, "pwm_pin", "?"), getattr(chassis.br, "dir_pin", "?"), "rev=", getattr(chassis.br, "reverse", False))


def motor_warmup(chassis, speed=45, ms=600):
	print("motor warmup forward speed=", speed)
	start = time.ticks_ms()
	while time.ticks_diff(time.ticks_ms(), start) < ms:
		chassis.forward(speed)
		time.sleep_ms(20)
	chassis.stop()
	time.sleep_ms(300)


def main():
	board = MainControl(config)
	chassis = build_chassis()
	prepare_chassis(chassis)
	imu_owner, imu_read, imu_calibrate, imu_backend = ensure_imu(board)

	if bool(getattr(config, "HEADING_LOCK_CALIBRATE_IMU", True)) and callable(imu_calibrate):
		print("imu calibrate; keep car still")
		imu_calibrate()
		print("imu calibrate done")

	print_startup_diagnostics(imu_owner, imu_read, imu_backend, chassis)
	wait_c14_start()

	forward_speed = float(getattr(config, "HEADING_LOCK_TEST_FORWARD_SPEED", 45))
	forward_speed *= float(getattr(config, "HEADING_LOCK_FORWARD_SIGN", -1.0))
	kp = float(getattr(config, "HEADING_LOCK_YAW_KP", 1.2))
	max_vw = float(getattr(config, "HEADING_LOCK_YAW_MAX_SPEED", 22))
	sign = float(getattr(config, "HEADING_LOCK_YAW_SIGN", 1.0))

	yaw = 0.0
	target_yaw = 0.0
	last_ms = time.ticks_ms()
	start_ms = last_ms
	last_print = last_ms

	print(
		"heading lock start",
		"vx=", forward_speed,
		"kp=", kp,
		"max_vw=", max_vw,
	)
	print("press Ctrl+C / stop mpremote to end this test")
	motor_warmup(chassis, speed=forward_speed)

	try:
		while True:
			now = time.ticks_ms()
			dt = max(1, time.ticks_diff(now, last_ms)) / 1000.0
			last_ms = now

			imu_data = imu_read()
			gz = gyro_z_dps(imu_data)
			yaw = (yaw + gz * dt) % 360.0
			err = angle_error(target_yaw, yaw)
			vw = clamp(err * kp * sign, -max_vw, max_vw)

			chassis.move(forward_speed, 0, vw)

			if time.ticks_diff(now, last_print) >= 200:
				last_print = now
				status = chassis.get_status()
				wheels = status.get("wheels", {})
				raw = imu_data.get("raw") if isinstance(imu_data, dict) else None
				ok = imu_data.get("ok") if isinstance(imu_data, dict) else None
				print(
					"yaw={:.2f} err={:.2f} gz={:.2f} vx={:.1f} vw={:.2f} imu_ok={} raw_gz={} wheels={}".format(
						yaw,
						err,
						gz,
						forward_speed,
						vw,
						ok,
						raw[5] if isinstance(raw, (tuple, list)) and len(raw) > 5 else None,
						wheels,
					)
				)

			time.sleep_ms(20)
	finally:
		chassis.stop()
		print("heading lock done")


if __name__ == "__main__":
	main()
