"""Pose estimation module."""

import time

from Module.imu_attitude_fusion import IMUFusion
from Module.wheel_odometry import AdvancedOdometry


class PoseEstimator:
	def __init__(self, config):
		self.cfg = config
		self.x = 0.0
		self.y = 0.0
		self.yaw = 0.0
		self.vx = 0.0
		self.vy = 0.0
		self.vw = 0.0
		self.last_ms = time.ticks_ms()
		self.fusion = IMUFusion(
			q_angle=getattr(config, "IMU_FUSION_Q_ANGLE", 0.001),
			q_gyro=getattr(config, "IMU_FUSION_Q_GYRO", 0.003),
			r_angle=getattr(config, "IMU_FUSION_R_ANGLE", 0.03),
		)
		self.odometry = AdvancedOdometry()
		self.is_odom_initialized = False
		self._pending_pose_init = None

	def reset(self):
		self.x = 0.0
		self.y = 0.0
		self.yaw = 0.0
		self.vx = 0.0
		self.vy = 0.0
		self.vw = 0.0
		self.last_ms = time.ticks_ms()
		self.fusion.angle = 0.0
		self.fusion.bias = 0.0
		self.odometry = AdvancedOdometry()
		self.is_odom_initialized = False
		self._pending_pose_init = None

	def request_pose_init(self, x, y, yaw):
		self._pending_pose_init = {
			"x": float(x),
			"y": float(y),
			"yaw": float(yaw),
		}

	def set_pose(self, x, y, yaw, current_absolute_yaw=None):
		self.x = float(x)
		self.y = float(y)
		self.yaw = float(yaw)
		if current_absolute_yaw is not None:
			self.odometry.set_pose(self.x, self.y, self.yaw, float(current_absolute_yaw))
			self.is_odom_initialized = True

	def _safe_axis(self, value, index):
		if isinstance(value, (tuple, list)) and len(value) > index:
			return float(value[index])
		return None

	def _body_velocity_from_wheels(self, wheel_speeds):
		fl = float(wheel_speeds.get("fl", 0.0))
		fr = float(wheel_speeds.get("fr", 0.0))
		bl = float(wheel_speeds.get("bl", 0.0))
		br = float(wheel_speeds.get("br", 0.0))
		vx = (fl + fr + bl + br) / 4.0
		vy = (fl - fr - bl + br) / 4.0
		vw = (fl - fr + bl - br) / 4.0
		return vx, vy, vw

	def _safe_mag_angle(self, mag):
		if isinstance(mag, (tuple, list)) and len(mag) >= 2:
			hx = float(mag[0])
			hy = float(mag[1])
			return self.fusion.get_mag_angle(hx, hy)
		return None

	def update(self, wheel_speeds=None, imu_data=None):
		now = time.ticks_ms()
		dt_ms = max(1, min(self.cfg.POSE_MAX_DT_MS, time.ticks_diff(now, self.last_ms)))
		self.last_ms = now
		dt = dt_ms / 1000.0

		wheel_speeds = wheel_speeds or {}
		raw_vx, raw_vy, raw_vw = self._body_velocity_from_wheels(wheel_speeds)
		self.vx = raw_vx * self.cfg.ODOM_LINEAR_SCALE
		self.vy = raw_vy * self.cfg.ODOM_LINEAR_SCALE
		odom_vw = raw_vw * self.cfg.ODOM_ANGULAR_SCALE

		gyro = None
		mag = None
		if imu_data:
			gyro = imu_data.get("gyro")
			mag = imu_data.get("mag")
		gyro_z = self._safe_axis(gyro, 2)
		mag_angle = self._safe_mag_angle(mag)
		if gyro_z is None and mag_angle is None:
			self.vw = odom_vw
			self.yaw = (self.yaw + self.vw * dt) % 360.0
		elif gyro_z is None:
			self.yaw = mag_angle
			self.vw = odom_vw
		else:
			gyro_rate = float(gyro_z)
			self.vw = gyro_rate
			if mag_angle is not None:
				self.yaw = self.fusion.update(gyro_rate, mag_angle, dt) % 360.0
			else:
				self.yaw = (self.yaw + gyro_rate * dt) % 360.0

		wheel_list = [
			float(wheel_speeds.get("fl", 0.0)) * self.cfg.ODOM_LINEAR_SCALE,
			float(wheel_speeds.get("fr", 0.0)) * self.cfg.ODOM_LINEAR_SCALE,
			float(wheel_speeds.get("bl", 0.0)) * self.cfg.ODOM_LINEAR_SCALE,
			float(wheel_speeds.get("br", 0.0)) * self.cfg.ODOM_LINEAR_SCALE,
		]
		if not self.is_odom_initialized:
			if self._pending_pose_init:
				init = self._pending_pose_init
				self.odometry.set_pose(init["x"], init["y"], init["yaw"], self.yaw)
				self.is_odom_initialized = True
				self._pending_pose_init = None
			else:
				self.odometry.reset(self.yaw)
				self.is_odom_initialized = True
		self.odometry.update(wheel_list, self.yaw, dt=dt)
		self.x, self.y, local_yaw = self.odometry.get_position()
		self.yaw = local_yaw

		return self.get_pose()

	def get_pose(self):
		return {
			"x": self.x,
			"y": self.y,
			"yaw": self.yaw,
			"vx": self.vx,
			"vy": self.vy,
			"vw": self.vw,
		}
