"""全向 PID 导航模块。"""

import math


def clamp(value, low, high):
	return max(low, min(high, value))


def normalize_angle_deg(angle):
	while angle > 180:
		angle -= 360
	while angle < -180:
		angle += 360
	return angle


class PID:
	def __init__(self, kp, ki=0.0, kd=0.0, output_limit=None):
		self.kp = kp
		self.ki = ki
		self.kd = kd
		self.output_limit = output_limit
		self.integral = 0.0
		self.last_error = 0.0

	def reset(self):
		self.integral = 0.0
		self.last_error = 0.0

	def update(self, error, dt):
		if dt <= 0:
			dt = 1e-3
		self.integral += error * dt
		derivative = (error - self.last_error) / dt
		self.last_error = error
		output = self.kp * error + self.ki * self.integral + self.kd * derivative
		if self.output_limit is not None:
			output = clamp(output, -self.output_limit, self.output_limit)
		return output


class OmniController:
	def __init__(self, config=None):
		self.cfg = config
		self.x_pos_pid = PID(
			getattr(config, "NAV_POS_KP", 2600.0) if config else 2600.0,
			getattr(config, "NAV_POS_KI", 0.0) if config else 0.0,
			getattr(config, "NAV_POS_KD", 120.0) if config else 120.0,
			getattr(config, "NAV_MAX_LINEAR_CMD", 5500) if config else 5500,
		)
		self.y_pos_pid = PID(
			getattr(config, "NAV_POS_KP", 2600.0) if config else 2600.0,
			getattr(config, "NAV_POS_KI", 0.0) if config else 0.0,
			getattr(config, "NAV_POS_KD", 120.0) if config else 120.0,
			getattr(config, "NAV_MAX_LINEAR_CMD", 5500) if config else 5500,
		)
		self.x_vel_pid = PID(
			getattr(config, "NAV_VEL_KP", 1800.0) if config else 1800.0,
			getattr(config, "NAV_VEL_KI", 0.0) if config else 0.0,
			getattr(config, "NAV_VEL_KD", 40.0) if config else 40.0,
			getattr(config, "NAV_MAX_LINEAR_CMD", 5500) if config else 5500,
		)
		self.y_vel_pid = PID(
			getattr(config, "NAV_VEL_KP", 1800.0) if config else 1800.0,
			getattr(config, "NAV_VEL_KI", 0.0) if config else 0.0,
			getattr(config, "NAV_VEL_KD", 40.0) if config else 40.0,
			getattr(config, "NAV_MAX_LINEAR_CMD", 5500) if config else 5500,
		)
		self.yaw_angle_pid = PID(
			getattr(config, "NAV_YAW_KP", 38.0) if config else 38.0,
			getattr(config, "NAV_YAW_KI", 0.0) if config else 0.0,
			getattr(config, "NAV_YAW_KD", 2.0) if config else 2.0,
			getattr(config, "NAV_BODY_RATE_LIMIT", 180.0) if config else 180.0,
		)
		self.yaw_rate_pid = PID(
			getattr(config, "NAV_YAW_RATE_KP", 280.0) if config else 280.0,
			getattr(config, "NAV_YAW_RATE_KI", 0.0) if config else 0.0,
			getattr(config, "NAV_YAW_RATE_KD", 8.0) if config else 8.0,
			getattr(config, "NAV_MAX_ROTATE_CMD", 2500) if config else 2500,
		)
		self.max_pwm = getattr(config, "NAV_WHEEL_PWM_LIMIT", 10000) if config else 10000
		self.max_linear = getattr(config, "NAV_MAX_LINEAR_CMD", 5500) if config else 5500
		self.max_rotate = getattr(config, "NAV_MAX_ROTATE_CMD", 2500) if config else 2500
		self.wheel_damp = getattr(config, "NAV_WHEEL_DAMP", 20.0) if config else 20.0
		self.body_vel_limit = getattr(config, "NAV_BODY_VEL_LIMIT", 1.2) if config else 1.2
		self.body_rate_limit = getattr(config, "NAV_BODY_RATE_LIMIT", 180.0) if config else 180.0
		self.wheel_feedforward = getattr(config, "NAV_WHEEL_FEEDFORWARD", 1.0) if config else 1.0
		self.last_debug = {}

	def reset(self):
		self.x_pos_pid.reset()
		self.y_pos_pid.reset()
		self.x_vel_pid.reset()
		self.y_vel_pid.reset()
		self.yaw_angle_pid.reset()
		self.yaw_rate_pid.reset()

	def _wheel_to_body(self, wheel_speeds):
		fl = wheel_speeds[0] if len(wheel_speeds) > 0 else 0.0
		fr = wheel_speeds[1] if len(wheel_speeds) > 1 else 0.0
		bl = wheel_speeds[2] if len(wheel_speeds) > 2 else 0.0
		br = wheel_speeds[3] if len(wheel_speeds) > 3 else 0.0
		vx = (fl - fr + bl - br) / 4.0
		vy = (-fl - fr + bl + br) / 4.0
		vw = (fl + fr + bl + br) / 4.0
		return vx, vy, vw

	def _world_to_body(self, vx_world, vy_world, curr_yaw_deg):
		rad = math.radians(curr_yaw_deg)
		cos_a = math.cos(rad)
		sin_a = math.sin(rad)
		vx_body = vx_world * cos_a + vy_world * sin_a
		vy_body = -vx_world * sin_a + vy_world * cos_a
		return vx_body, vy_body

	def update(self, target_pos, curr_pos, target_yaw, curr_yaw, gyro_z, current_wheel_speeds, dt):
		target_x, target_y = target_pos
		curr_x, curr_y = curr_pos
		err_x = target_x - curr_x
		err_y = target_y - curr_y

		vx_world_target = self.x_pos_pid.update(err_x, dt)
		vy_world_target = self.y_pos_pid.update(err_y, dt)
		vx_world_target = clamp(vx_world_target, -self.body_vel_limit, self.body_vel_limit)
		vy_world_target = clamp(vy_world_target, -self.body_vel_limit, self.body_vel_limit)

		vx_body_target, vy_body_target = self._world_to_body(vx_world_target, vy_world_target, curr_yaw)

		body_vx, body_vy, _ = self._wheel_to_body(current_wheel_speeds)
		vx_cmd = self.x_vel_pid.update(vx_body_target - body_vx, dt)
		vy_cmd = self.y_vel_pid.update(vy_body_target - body_vy, dt)
		vx_cmd = clamp(vx_cmd, -self.max_linear, self.max_linear)
		vy_cmd = clamp(vy_cmd, -self.max_linear, self.max_linear)

		yaw_error = normalize_angle_deg(target_yaw - curr_yaw)
		target_yaw_rate = self.yaw_angle_pid.update(yaw_error, dt)
		target_yaw_rate = clamp(target_yaw_rate, -self.body_rate_limit, self.body_rate_limit)
		vw_cmd = self.yaw_rate_pid.update(target_yaw_rate - gyro_z, dt)
		vw_cmd = clamp(vw_cmd, -self.max_rotate, self.max_rotate)

		wheel_targets = [
			vx_cmd + vy_cmd + vw_cmd,
			-vx_cmd + vy_cmd + vw_cmd,
			vx_cmd - vy_cmd + vw_cmd,
			-vx_cmd - vy_cmd + vw_cmd,
		]

		max_abs_target = max(1.0, max(abs(value) for value in wheel_targets))
		scale = 1.0
		if max_abs_target > self.max_pwm:
			scale = self.max_pwm / max_abs_target
			wheel_targets = [value * scale for value in wheel_targets]

		outputs = []
		for index, target in enumerate(wheel_targets):
			current_speed = current_wheel_speeds[index] if index < len(current_wheel_speeds) else 0.0
			output = target * self.wheel_feedforward - current_speed * self.wheel_damp
			outputs.append(clamp(output, -self.max_pwm, self.max_pwm))

		self.last_debug = {
			"err_x": err_x,
			"err_y": err_y,
			"yaw_error": yaw_error,
			"vx_world_target": vx_world_target,
			"vy_world_target": vy_world_target,
			"vx_body_target": vx_body_target,
			"vy_body_target": vy_body_target,
			"vx_cmd": vx_cmd,
			"vy_cmd": vy_cmd,
			"vw_cmd": vw_cmd,
			"scale": scale,
		}
		return outputs
