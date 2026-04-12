"""IMU姿态融合模块。"""

try:
	from array import array
except Exception:
	array = None

import math
import time


class IMUFusion:
	def __init__(self, q_angle=0.001, q_gyro=0.003, r_angle=0.03):
		self.q_angle = q_angle
		self.q_gyro = q_gyro
		self.r_angle = r_angle
		self.angle = 0.0
		self.bias = 0.0
		self.P = [[1, 0], [0, 1]]
		self.mag_table = None
		self.is_calibrated = False

	def update(self, gyro_rate, mag_angle, dt):
		self.angle += (gyro_rate - self.bias) * dt
		self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.q_angle)
		self.P[0][1] -= dt * self.P[1][1]
		self.P[1][0] -= dt * self.P[1][1]
		self.P[1][1] += self.q_gyro * dt

		error = mag_angle - self.angle
		if error > 180:
			error -= 360
		if error < -180:
			error += 360

		S = self.P[0][0] + self.r_angle
		k0 = self.P[0][0] / S
		k1 = self.P[1][0] / S

		self.angle += k0 * error
		self.bias += k1 * error

		p00_temp, p01_temp = self.P[0][0], self.P[0][1]
		self.P[0][0] -= k0 * p00_temp
		self.P[0][1] -= k0 * p01_temp
		self.P[1][0] -= k1 * p00_temp
		self.P[1][1] -= k1 * p01_temp

		return self.angle

	def get_mag_angle(self, hx, hy):
		raw_angle = math.degrees(math.atan2(hy, hx))
		if raw_angle < 0:
			raw_angle += 360
		table = self.mag_table
		if table:
			return float(table[int(raw_angle) % 360])
		return raw_angle

	def calibrate_mag(self, imu, motors, duration=5.0):
		print(">>> 开始地磁计校准...")
		if self.mag_table is None:
			if array:
				self.mag_table = array("H", range(360))
			else:
				self.mag_table = list(range(360))
		start_time = time.ticks_ms()
		gyro_angle = 0.0
		for motor in motors:
			motor.duty(3000)

		while time.ticks_diff(time.ticks_ms(), start_time) < duration * 1000:
			imu.read()
			data = imu.get()
			gz = data[5] * 0.07
			gyro_angle += gz * 0.01
			hx, hy = data[6] / 3000.0, data[7] / 3000.0
			mag_raw = math.degrees(math.atan2(hy, hx))
			if mag_raw < 0:
				mag_raw += 360
			self.mag_table[int(mag_raw) % 360] = int(gyro_angle) % 360
			time.sleep(0.01)

		for motor in motors:
			motor.duty(0)
		self.is_calibrated = True
		print(">>> 地磁计校准完成！")
