"""避障传感器驱动。"""

import time
from machine import Pin


class IRSensorArray:
	def __init__(self, left_pin, front_pin, right_pin, active_level=0):
		self.left = Pin(left_pin, Pin.IN)
		self.front = Pin(front_pin, Pin.IN)
		self.right = Pin(right_pin, Pin.IN)
		self.active_level = active_level

	def _active(self, pin):
		return pin.value() == self.active_level

	def read(self):
		return {
			"left": self._active(self.left),
			"front": self._active(self.front),
			"right": self._active(self.right),
		}


class UltrasonicSensor:
	def __init__(self, trig_pin, echo_pin):
		self.trig = Pin(trig_pin, Pin.OUT)
		self.echo = Pin(echo_pin, Pin.IN)
		self.trig.value(0)

	def distance_cm(self):
		self.trig.value(0)
		time.sleep_us(2)
		self.trig.value(1)
		time.sleep_us(10)
		self.trig.value(0)

		timeout_us = 30000
		start = time.ticks_us()
		while self.echo.value() == 0:
			if time.ticks_diff(time.ticks_us(), start) > timeout_us:
				return None
		pulse_start = time.ticks_us()

		while self.echo.value() == 1:
			if time.ticks_diff(time.ticks_us(), pulse_start) > timeout_us:
				return None
		pulse_end = time.ticks_us()
		pulse_width = time.ticks_diff(pulse_end, pulse_start)
		return pulse_width / 58.0
