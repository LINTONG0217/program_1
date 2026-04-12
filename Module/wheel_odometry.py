"""轮速里程计模块。"""

import math


class AdvancedOdometry:
	def __init__(self):
		self.x = 0.0
		self.y = 0.0
		self.yaw_offset = 0.0
		self.local_yaw = 0.0

	def reset(self, current_absolute_yaw):
		self.x = 0.0
		self.y = 0.0
		self.yaw_offset = current_absolute_yaw
		self.local_yaw = 0.0

	def set_pose(self, x, y, world_yaw, current_absolute_yaw):
		self.x = float(x)
		self.y = float(y)
		self.yaw_offset = float(current_absolute_yaw) - float(world_yaw)
		self.local_yaw = float(world_yaw)

	def update(self, current_speeds, current_absolute_yaw, dt=0.01):
		self.local_yaw = current_absolute_yaw - self.yaw_offset

		v_lf, v_rf, v_lr, v_rr = current_speeds

		vx_local = (v_lf + v_rf + v_lr + v_rr) / 4.0
		vy_local = (-v_lf + v_rf + v_lr - v_rr) / 4.0

		if abs(vx_local) < 0.005:
			vx_local = 0.0
		if abs(vy_local) < 0.005:
			vy_local = 0.0

		dx_local = vx_local * dt
		dy_local = vy_local * dt

		rad = math.radians(self.local_yaw)
		cos_a = math.cos(rad)
		sin_a = math.sin(rad)

		self.x += dx_local * cos_a - dy_local * sin_a
		self.y += dx_local * sin_a + dy_local * cos_a

	def get_position(self):
		return self.x, self.y, self.local_yaw
