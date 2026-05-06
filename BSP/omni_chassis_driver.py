"""全向底盘驱动。"""

import time


def clamp(value, low, high):
	return max(low, min(high, value))


class WheelPID:
	def __init__(self, kp, ki, kd, limit):
		self.kp = kp
		self.ki = ki
		self.kd = kd
		self.limit = abs(limit)
		self.integral = 0.0
		self.last_error = 0.0
		self.last_ms = None

	def reset(self):
		self.integral = 0.0
		self.last_error = 0.0
		self.last_ms = None

	def update(self, error):
		now = time.ticks_ms()
		dt = 0.0
		if self.last_ms is not None:
			dt = time.ticks_diff(now, self.last_ms) / 1000.0

		derivative = 0.0
		if dt > 0:
			self.integral += error * dt
			derivative = (error - self.last_error) / dt

		self.last_error = error
		self.last_ms = now
		output = self.kp * error + self.ki * self.integral + self.kd * derivative
		return clamp(output, -self.limit, self.limit)


class OmniChassis:
	def __init__(self, motor_fl, motor_fr, motor_bl, motor_br, config=None):
		self.fl = motor_fl
		self.fr = motor_fr
		self.bl = motor_bl
		self.br = motor_br
		self.cfg = config
		self.last_vx = 0
		self.last_vy = 0
		self.last_vw = 0
		self.last_command_ms = time.ticks_ms()
		self.encoders = {}
		self.imu = None
		self.closed_loop_enabled = False
		self.wheel_targets = {"fl": 0.0, "fr": 0.0, "bl": 0.0, "br": 0.0}
		self.wheel_feedback = {"fl": 0.0, "fr": 0.0, "bl": 0.0, "br": 0.0}
		base_trim = {"fl": 1.0, "fr": 1.0, "bl": 1.0, "br": 1.0}
		cfg_trim = getattr(config, "WHEEL_TRIM", None) if config else None
		if isinstance(cfg_trim, dict):
			base_trim.update(cfg_trim)
		self.wheel_trim = dict(base_trim)
		self.wheel_trim_fwd = dict(base_trim)
		self.wheel_trim_rev = dict(base_trim)
		cfg_trim_fwd = getattr(config, "WHEEL_TRIM_FWD", None) if config else None
		cfg_trim_rev = getattr(config, "WHEEL_TRIM_REV", None) if config else None
		if isinstance(cfg_trim_fwd, dict):
			self.wheel_trim_fwd.update(cfg_trim_fwd)
		if isinstance(cfg_trim_rev, dict):
			self.wheel_trim_rev.update(cfg_trim_rev)
		self.debug_last_ms = 0
		
		# 参照 C 代码框架，改为使用底盘级动力学解算 PID (处理整体的 vx, vy, vw)
		self.vx_pid = WheelPID(getattr(config, "WHEEL_PID_P", 0.0), getattr(config, "WHEEL_PID_I", 0.0), getattr(config, "WHEEL_PID_D", 0.0), getattr(config, "WHEEL_PID_LIMIT", 0.0)) if config else WheelPID(0.0, 0.0, 0.0, 0.0)
		self.vy_pid = WheelPID(getattr(config, "WHEEL_PID_P", 0.0), getattr(config, "WHEEL_PID_I", 0.0), getattr(config, "WHEEL_PID_D", 0.0), getattr(config, "WHEEL_PID_LIMIT", 0.0)) if config else WheelPID(0.0, 0.0, 0.0, 0.0)
		self.vw_pid = WheelPID(getattr(config, "WHEEL_PID_P", 0.0), getattr(config, "WHEEL_PID_I", 0.0), getattr(config, "WHEEL_PID_D", 0.0), getattr(config, "WHEEL_PID_LIMIT", 0.0)) if config else WheelPID(0.0, 0.0, 0.0, 0.0)

	def attach_feedback(self, encoders=None, imu=None):
		self.encoders = encoders or {}
		self.imu = imu
		self.closed_loop_enabled = bool(
			self.cfg
			and getattr(self.cfg, "CHASSIS_ENABLE_CLOSED_LOOP", False)
			and self.encoders
			and all(getattr(self.encoders.get(name), "available", False) for name in ("fl", "fr", "bl", "br"))
		)
		if not self.closed_loop_enabled:
			self.vx_pid.reset()
			self.vy_pid.reset()
			self.vw_pid.reset()

	def _deadband(self, value):
		if not self.cfg:
			return value
		threshold = getattr(self.cfg, "CHASSIS_CMD_DEADBAND", 0)
		if abs(value) < threshold:
			return 0
		return value

	def _step_limit(self, target, current, limit):
		if limit <= 0:
			return target

		delta = target - current
		if delta > limit:
			return current + limit
		if delta < -limit:
			return current - limit
		return target

	def _shape_command(self, vx, vy, vw):
		if not self.cfg:
			return vx, vy, vw

		vx = self._deadband(vx)
		vy = self._deadband(vy)
		vw = self._deadband(vw)
		vx = self._step_limit(vx, self.last_vx, getattr(self.cfg, "CHASSIS_MAX_VX_STEP", 0))
		vy = self._step_limit(vy, self.last_vy, getattr(self.cfg, "CHASSIS_MAX_VY_STEP", 0))
		vw = self._step_limit(vw, self.last_vw, getattr(self.cfg, "CHASSIS_MAX_VW_STEP", 0))
		return vx, vy, vw

	def move(self, vx, vy, vw):
		vx, vy, vw = self._shape_command(vx, vy, vw)
		self.last_vx = vx
		self.last_vy = vy
		self.last_vw = vw
		self.last_command_ms = time.ticks_ms()

		drive_vx = vx * float(getattr(self.cfg, "CHASSIS_VX_SIGN", 1.0)) if self.cfg else vx
		drive_vy = vy * float(getattr(self.cfg, "CHASSIS_VY_SIGN", 1.0)) if self.cfg else vy
		drive_vw = vw * float(getattr(self.cfg, "CHASSIS_VW_SIGN", 1.0)) if self.cfg else vw

		v_fl = drive_vx + drive_vy + drive_vw
		v_fr = -drive_vx + drive_vy + drive_vw
		v_bl = drive_vx - drive_vy + drive_vw
		v_br = -drive_vx - drive_vy + drive_vw
		self.wheel_targets = {"fl": v_fl, "fr": v_fr, "bl": v_bl, "br": v_br}

		max_v = max(abs(v_fl), abs(v_fr), abs(v_bl), abs(v_br))
		if max_v > 100:
			scale = 100 / max_v
			v_fl *= scale
			v_fr *= scale
			v_bl *= scale
			v_br *= scale
			self.wheel_targets = {"fl": v_fl, "fr": v_fr, "bl": v_bl, "br": v_br}

		commands = {"fl": v_fl, "fr": v_fr, "bl": v_bl, "br": v_br}
		if self.closed_loop_enabled:
			measured = {}
			alpha = getattr(self.cfg, "WHEEL_SPEED_FILTER", 0.35)
			# 获取每个轮子的实际速度
			for name in ["fl", "fr", "bl", "br"]:
				speed = float(self.encoders[name].read_speed())
				measured[name] = self.wheel_feedback[name] * (1.0 - alpha) + speed * alpha
				self.wheel_feedback[name] = measured[name]

			# 1. 估算当前底盘整体速度 (EstimateSpeed 逆运动学)
			now_vx = (measured["fl"] - measured["fr"] + measured["bl"] - measured["br"]) * 0.25
			now_vy = (measured["fl"] + measured["fr"] - measured["bl"] - measured["br"]) * 0.25
			now_vw = (measured["fl"] + measured["fr"] + measured["bl"] + measured["br"]) * 0.25

			# 2. 计算底盘各轴动态输出 (ChassisOutput_Dynamic)
			force_x = self.vx_pid.update(drive_vx - now_vx)
			force_y = self.vy_pid.update(drive_vy - now_vy)
			torque_w = self.vw_pid.update(drive_vw - now_vw)

			# 3. 动力学重分配 + 阻尼 (模拟 C 代码力矩分配逻辑与阻尼项)
			damp = getattr(self.cfg, "WHEEL_SPEED_LIMIT_DAMP", 0.035)
			
			# 前馈预估(由于我们输出占空比而非直接发送电流) + 独立计算闭环控制量 + 极小阻尼控制
			# 注意: 配置中已针对 fr, br 进行硬件极性翻转，所以解算矩阵保持常规形式
			cmd_fl = v_fl + force_x + force_y + torque_w + damp * (v_fl - measured["fl"])
			cmd_fr = v_fr - force_x + force_y + torque_w + damp * (v_fr - measured["fr"])
			cmd_bl = v_bl + force_x - force_y + torque_w + damp * (v_bl - measured["bl"])
			cmd_br = v_br - force_x - force_y + torque_w + damp * (v_br - measured["br"])

			commands["fl"] = clamp(cmd_fl, -100, 100)
			commands["fr"] = clamp(cmd_fr, -100, 100)
			commands["bl"] = clamp(cmd_bl, -100, 100)
			commands["br"] = clamp(cmd_br, -100, 100)
		else:
			self.wheel_feedback = dict(self.wheel_targets)

		active_trim = self.wheel_trim
		if abs(vy) < 1e-6 and abs(vw) < 1e-6:
			if vx >= 0:
				active_trim = self.wheel_trim_fwd
			else:
				active_trim = self.wheel_trim_rev

		cmd_fl = clamp(commands["fl"] * float(active_trim.get("fl", 1.0)), -100, 100)
		cmd_fr = clamp(commands["fr"] * float(active_trim.get("fr", 1.0)), -100, 100)
		cmd_bl = clamp(commands["bl"] * float(active_trim.get("bl", 1.0)), -100, 100)
		cmd_br = clamp(commands["br"] * float(active_trim.get("br", 1.0)), -100, 100)

		self.fl.set_speed(cmd_fl)
		self.fr.set_speed(cmd_fr)
		self.bl.set_speed(cmd_bl)
		self.br.set_speed(cmd_br)

		if self.cfg and getattr(self.cfg, "CHASSIS_DEBUG_PRINT_ENABLE", False):
			now = time.ticks_ms()
			interval = int(getattr(self.cfg, "CHASSIS_DEBUG_PRINT_MS", 200))
			if time.ticks_diff(now, self.debug_last_ms) >= interval:
				self.debug_last_ms = now
				run_motors = max(abs(vx), abs(vy), abs(vw)) > 0
				print(
					"run_motors:",
					run_motors,
					"pwms_raw:",
					[int(commands["fl"]), int(commands["fr"]), int(commands["bl"]), int(commands["br"])],
					"pwms_out:",
					[int(cmd_fl), int(cmd_fr), int(cmd_bl), int(cmd_br)],
				)

	def forward(self, speed):
		self.move(speed, 0, 0)

	def strafe(self, speed):
		self.move(0, speed, 0)

	def rotate(self, speed):
		self.last_vx = 0
		self.last_vy = 0
		self.move(0, 0, speed)

	def stop(self):
		self.last_vx = 0
		self.last_vy = 0
		self.last_vw = 0
		self.last_command_ms = time.ticks_ms()
		self.wheel_targets = {"fl": 0.0, "fr": 0.0, "bl": 0.0, "br": 0.0}
		self.fl.stop()
		self.fr.stop()
		self.bl.stop()
		self.br.stop()
		self.vx_pid.reset()
		self.vy_pid.reset()
		self.vw_pid.reset()

	def check_watchdog(self):
		if not self.cfg:
			return False

		timeout_ms = getattr(self.cfg, "CHASSIS_COMMAND_TIMEOUT_MS", 0)
		if timeout_ms <= 0:
			return False

		if time.ticks_diff(time.ticks_ms(), self.last_command_ms) > timeout_ms:
			self.stop()
			return True
		return False

	def get_status(self):
		return {
			"vx": self.last_vx,
			"vy": self.last_vy,
			"vw": self.last_vw,
			"closed_loop": self.closed_loop_enabled,
			"wheel_trim": dict(self.wheel_trim),
			"wheel_trim_fwd": dict(self.wheel_trim_fwd),
			"wheel_trim_rev": dict(self.wheel_trim_rev),
			"wheels": {
				"fl": getattr(self.fl, "speed", 0),
				"fr": getattr(self.fr, "speed", 0),
				"bl": getattr(self.bl, "speed", 0),
				"br": getattr(self.br, "speed", 0),
			},
			"wheel_feedback": self.get_wheel_feedback(),
		}

	def get_wheel_feedback(self):
		return dict(self.wheel_feedback)
