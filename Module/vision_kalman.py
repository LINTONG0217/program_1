"""Lightweight Kalman filtering for vision targets."""

import time


def _clamp(value, min_value, max_value):
	return max(min_value, min(max_value, value))


class Kalman1D:
	def __init__(self, process_var=35.0, measurement_var=80.0):
		self.q = float(process_var)
		self.r = float(measurement_var)
		self.x = 0.0
		self.v = 0.0
		self.p00 = 1.0
		self.p01 = 0.0
		self.p10 = 0.0
		self.p11 = 1.0
		self.ready = False

	def reset(self, value=None):
		self.v = 0.0
		self.p00 = 1.0
		self.p01 = 0.0
		self.p10 = 0.0
		self.p11 = 1.0
		self.ready = value is not None
		if value is not None:
			self.x = float(value)

	def update(self, measurement, dt):
		z = float(measurement)
		if not self.ready:
			self.reset(z)
			return self.x

		dt = _clamp(float(dt), 0.001, 0.20)

		# Predict: x = x + v * dt
		self.x += self.v * dt
		p00 = self.p00 + dt * (self.p10 + self.p01) + dt * dt * self.p11 + self.q
		p01 = self.p01 + dt * self.p11
		p10 = self.p10 + dt * self.p11
		p11 = self.p11 + self.q
		self.p00, self.p01, self.p10, self.p11 = p00, p01, p10, p11

		# Correct with scalar position measurement.
		s = self.p00 + self.r
		if s <= 0:
			return self.x
		k0 = self.p00 / s
		k1 = self.p10 / s
		y = z - self.x
		self.x += k0 * y
		self.v += k1 * y
		p00, p01 = self.p00, self.p01
		self.p00 = (1.0 - k0) * p00
		self.p01 = (1.0 - k0) * p01
		self.p10 = self.p10 - k1 * p00
		self.p11 = self.p11 - k1 * p01
		return self.x


class VisionTargetKalman:
	def __init__(self, process_var=35.0, measurement_var=80.0, reset_ms=350):
		self.filters = {
			"x": Kalman1D(process_var, measurement_var),
			"y": Kalman1D(process_var, measurement_var),
			"w": Kalman1D(process_var, measurement_var),
			"h": Kalman1D(process_var, measurement_var),
			"size": Kalman1D(process_var, measurement_var),
		}
		self.reset_ms = int(reset_ms)
		self.last_ms = None
		self.missed = 0

	def reset(self):
		for key in self.filters:
			self.filters[key].reset()
		self.last_ms = None
		self.missed = 0

	def mark_missing(self):
		self.missed += 1
		if self.missed >= 2:
			self.reset()

	def update(self, target):
		if not target:
			self.mark_missing()
			return None

		now = time.ticks_ms()
		if self.last_ms is None:
			dt = 0.04
		else:
			try:
				gap_ms = time.ticks_diff(now, self.last_ms)
			except Exception:
				gap_ms = now - self.last_ms
			if gap_ms > self.reset_ms:
				self.reset()
				dt = 0.04
			else:
				dt = max(0.001, float(gap_ms) / 1000.0)

		out = dict(target)
		for key in self.filters:
			try:
				value = target.get(key)
			except Exception:
				value = None
			if value is None:
				continue
			out[key] = int(round(self.filters[key].update(value, dt)))

		self.last_ms = now
		self.missed = 0
		return out
