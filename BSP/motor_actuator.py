"""电机执行器驱动。"""

from machine import Pin, PWM


try:
	from seekfree import MOTOR_CONTROLLER  # type: ignore
except ImportError:
	MOTOR_CONTROLLER = None


_MOTOR_CHANNEL_MAP = {
	("D4", "D5"): "PWM_D4_DIR_D5",
	("D6", "D7"): "PWM_D6_DIR_D7",
	("C30", "C31"): "PWM_C30_DIR_C31",
	("C28", "C29"): "PWM_C28_DIR_C29",
}


def _config_value(name, default):
	try:
		from Module import config

		return getattr(config, name, default)
	except Exception:
		return default


def _config_bool(name, default):
	return bool(_config_value(name, default))


def _config_float(name, default):
	try:
		return float(_config_value(name, default))
	except Exception:
		return float(default)


class Motor:
	def __init__(self, pwm_pin, dir_pin, freq=1000, pwm_max=1023, reverse=False):
		self.controller = None
		self.pwm = None
		self.dir = None
		self.pwm_pin = str(pwm_pin)
		self.dir_pin = str(dir_pin)
		self.pwm_max = pwm_max
		self.reverse = reverse
		self.speed = 0

		channel_name = _MOTOR_CHANNEL_MAP.get((self.pwm_pin, self.dir_pin))
		if MOTOR_CONTROLLER and channel_name and hasattr(MOTOR_CONTROLLER, channel_name):
			try:
				self.controller = MOTOR_CONTROLLER(getattr(MOTOR_CONTROLLER, channel_name), freq)
			except Exception:
				self.controller = None

		if self.controller is None:
			self.pwm = PWM(Pin(pwm_pin), freq=freq)
			self.dir = Pin(dir_pin, Pin.OUT)

		self.stop()

	def _write_pwm(self, pwm, duty):
		duty = int(max(0, min(self.pwm_max, duty)))
		if hasattr(pwm, "duty_u16"):
			pwm.duty_u16(int(duty * 65535 / self.pwm_max))
		else:
			pwm.duty(duty)

	def set_speed(self, speed):
		speed = max(-100, min(100, speed))
		if not _config_bool("MOTOR_OUTPUT_ENABLE", True):
			speed = 0
		if self.reverse:
			speed = -speed

		limit = abs(_config_float("MOTOR_SPEED_LIMIT_PERCENT", 100.0))
		if limit < 100:
			speed = max(-limit, min(limit, speed))

		step = abs(_config_float("MOTOR_MAX_SPEED_STEP_PERCENT", 0.0))
		if step > 0:
			delta = speed - self.speed
			if delta > step:
				speed = self.speed + step
			elif delta < -step:
				speed = self.speed - step

		self.speed = speed

		if self.controller is not None:
			self.controller.duty(int(speed * self.pwm_max / 100))
			return

		duty = int(abs(speed) * self.pwm_max / 100)
		direction = 1 if speed >= 0 else 0
		self.dir.value(direction)
		self._write_pwm(self.pwm, duty)

	def stop(self):
		self.speed = 0
		if self.controller is not None:
			self.controller.duty(0)
		else:
			self._write_pwm(self.pwm, 0)

	def duty(self, pwm_value):
		limit = abs(int(_config_float("MOTOR_DIRECT_DUTY_LIMIT", self.pwm_max)))
		limit = max(0, min(self.pwm_max, limit))
		pwm_value = int(max(-limit, min(limit, pwm_value)))
		self.set_speed(pwm_value * 100 / self.pwm_max)
