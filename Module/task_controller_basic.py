"""基础任务控制器。"""

import time


def clamp(value, min_value, max_value):
	return max(min_value, min(max_value, value))


class PIDController:
	def __init__(self, kp, ki=0.0, kd=0.0, output_limit=100):
		self.kp = kp
		self.ki = ki
		self.kd = kd
		self.output_limit = abs(output_limit)
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

		output = self.kp * error + self.ki * self.integral + self.kd * derivative
		self.last_error = error
		self.last_ms = now
		return clamp(output, -self.output_limit, self.output_limit)


class SmartCarController:
	STATE_SEARCH_OBJECT = "search_object"
	STATE_APPROACH_OBJECT = "approach_object"
	STATE_SEARCH_ZONE = "search_zone"
	STATE_ALIGN_PUSH = "align_push"
	STATE_PUSH = "push"
	STATE_AFTER_PUSH_RETREAT = "after_push_retreat"

	def __init__(self, chassis, vision, config):
		try:
			print("SmartCarController: __init__ enter")
		except Exception:
			pass
		self.chassis = chassis
		self.vision = vision
		self.cfg = config
		self.state = self.STATE_SEARCH_OBJECT
		self.push_start_ms = None
		self.retreat_until_ms = None
		self.last_cmd_vx = 0.0
		self.last_cmd_vy = 0.0
		self._dbg_last_ms = 0
		self._dbg_last_stale = None
		self._search_phase_until_ms = None
		self._search_rotating = True
		self._stale_search_until_ms = None
		self._stale_search_retry_after_ms = None

		self.center_pid = PIDController(
			config.PID_CENTER_P,
			config.PID_CENTER_I,
			config.PID_CENTER_D,
			config.MAX_LATERAL_SPEED,
		)
		self.dist_pid = PIDController(
			config.PID_DIST_P,
			config.PID_DIST_I,
			config.PID_DIST_D,
			config.APPROACH_MAX_SPEED,
		)
		self.push_align_pid = PIDController(
			config.PID_PUSH_ALIGN_P,
			config.PID_PUSH_ALIGN_I,
			config.PID_PUSH_ALIGN_D,
			config.MAX_LATERAL_SPEED,
		)
		try:
			print("SmartCarController: __init__ ok")
		except Exception:
			pass
		# 连续帧确认计数器，避免单帧误触发停车
		self._approach_ok_count = 0
		self._approach_stall_start_ms = None
		self._approach_progress_start_ms = None
		self._approach_progress_best_size = None
		# 目标确认计数器，避免误检导致突然直走
		self._target_confirm_count = 0

	def _set_state(self, new_state):
		if self.state != new_state:
			try:
				src = getattr(self, "_vision_source", None)
			except Exception:
				src = None
			if src:
				print("state ->", new_state, "src=", src)
			else:
				print("state ->", new_state)
			self.state = new_state
			self.center_pid.reset()
			self.dist_pid.reset()
			self.push_align_pid.reset()
			self.last_cmd_vx = 0.0
			self.last_cmd_vy = 0.0
			self._target_confirm_count = 0
			self._search_phase_until_ms = None
			self._search_rotating = True
			if new_state == self.STATE_APPROACH_OBJECT:
				self._approach_stall_start_ms = None
				self._approach_progress_start_ms = None
				self._approach_progress_best_size = None
			if new_state == self.STATE_SEARCH_OBJECT:
				self._stale_search_until_ms = None

	def _vision_link_ok(self):
		"""视觉链路是否“有数据且未过期”。

		注意：search 状态旋转会依赖这个判断，避免视觉断连时小车一直转圈。
		"""
		try:
			last_update = getattr(self.vision, "last_update_ms", None)
			if last_update is None:
				return False
			age_ms = time.ticks_diff(time.ticks_ms(), last_update)
			max_age = int(getattr(self.cfg, "VISION_MAX_AGE_MS", int(getattr(self.cfg, "FRAME_TIMEOUT_MS", 500)) * 2))
			return age_ms <= max_age
		except Exception:
			return False

	def _search_object(self):
		# 视觉断帧时：做“限时脉冲旋转”尝试重新抓取，避免一直停在 search。
		if not self._vision_link_ok():
			now = time.ticks_ms()
			retry_pause_ms = int(getattr(self.cfg, "STALE_SEARCH_RETRY_PAUSE_MS", 1200))
			if self._stale_search_retry_after_ms is not None:
				if time.ticks_diff(now, self._stale_search_retry_after_ms) < 0:
					self.chassis.stop()
					return
				self._stale_search_retry_after_ms = None
			total_ms = int(getattr(self.cfg, "STALE_SEARCH_TOTAL_MS", 1500))
			if self._stale_search_until_ms is None:
				self._stale_search_until_ms = time.ticks_add(now, max(200, total_ms))
			if time.ticks_diff(self._stale_search_until_ms, now) <= 0:
				self.chassis.stop()
				self._stale_search_until_ms = None
				self._stale_search_retry_after_ms = time.ticks_add(now, max(200, retry_pause_ms))
				return
			pulse_enable = True
			rot_speed = float(getattr(self.cfg, "STALE_SEARCH_ROT_SPEED", getattr(self.cfg, "SEARCH_ROT_SPEED", 0)))
			rotate_ms = int(getattr(self.cfg, "STALE_SEARCH_ROTATE_MS", 160))
			pause_ms = int(getattr(self.cfg, "STALE_SEARCH_PAUSE_MS", 160))
			rotate_ms = max(20, rotate_ms)
			pause_ms = max(20, pause_ms)
			if self._search_phase_until_ms is None or time.ticks_diff(now, self._search_phase_until_ms) >= 0:
				self._search_rotating = not bool(self._search_rotating)
				phase_ms = rotate_ms if self._search_rotating else pause_ms
				self._search_phase_until_ms = time.ticks_add(now, phase_ms)
			if self._search_rotating:
				self.chassis.rotate(rot_speed)
			else:
				self.chassis.stop()
			return
		else:
			self._stale_search_until_ms = None
			self._stale_search_retry_after_ms = None

		pulse_enable = bool(getattr(self.cfg, "SEARCH_PULSE_ENABLE", False))
		if not pulse_enable:
			self.chassis.rotate(self.cfg.SEARCH_ROT_SPEED)
			return

		now = time.ticks_ms()
		rotate_ms = int(getattr(self.cfg, "SEARCH_ROTATE_MS", 200))
		pause_ms = int(getattr(self.cfg, "SEARCH_PAUSE_MS", 80))
		rotate_ms = max(20, rotate_ms)
		pause_ms = max(20, pause_ms)

		if self._search_phase_until_ms is None or time.ticks_diff(now, self._search_phase_until_ms) >= 0:
			self._search_rotating = not bool(self._search_rotating)
			phase_ms = rotate_ms if self._search_rotating else pause_ms
			self._search_phase_until_ms = time.ticks_add(now, phase_ms)

		if self._search_rotating:
			self.chassis.rotate(self.cfg.SEARCH_ROT_SPEED)
		else:
			self.chassis.stop()

	def _approach_object(self, target):
		offset_x = self._signed_offset_x(target)
		center_error = 0 if abs(offset_x) < self.cfg.CENTER_DEADBAND else offset_x
		size = int(target.get("size", 0))
		dist_error = self.cfg.TARGET_OBJECT_SIZE - size
		try:
			method = target.get("method") if isinstance(target, dict) else None
		except Exception:
			method = None
		try:
			is_held = bool(target.get("held", False)) if isinstance(target, dict) else False
		except Exception:
			is_held = False

		# 先判断：若目标尺寸已经达到或超过阈值，则要求连续多帧确认后停止
		if target.get("size", 0) >= getattr(self.cfg, "TARGET_OBJECT_SIZE", 0):
			self._approach_ok_count += 1
			if self._approach_ok_count >= int(getattr(self.cfg, "APPROACH_STOP_FRAMES", 2)):
				self.chassis.stop()
				self.last_cmd_vx = 0.0
				self.last_cmd_vy = 0.0
				self._approach_ok_count = 0
				return "arrived"
		else:
			# 未满足连续帧条件则继续靠近并清零计数
			self._approach_ok_count = 0

		if bool(getattr(self.cfg, "OBJECT_STOP_MODE", False)):
			stop_size_margin = int(getattr(self.cfg, "APPROACH_STOP_SIZE_MARGIN", 6))
			stop_center_db = int(getattr(self.cfg, "APPROACH_CENTER_STOP_DEADBAND", max(10, self.cfg.CENTER_DEADBAND)))
			if dist_error <= stop_size_margin and abs(target["offset_x"]) <= stop_center_db:
				self.chassis.stop()
				self.last_cmd_vx = 0.0
				self.last_cmd_vy = 0.0
				self._approach_ok_count = 0
				return "arrived"

		vy = self.center_pid.update(center_error)
		vx = self.dist_pid.update(dist_error)
		# 基本限幅
		vx = clamp(vx, 0, self.cfg.APPROACH_MAX_SPEED)

		# 安全：目标偏离中心太大时，不允许向前冲，先横移把目标拉回中心。
		strafe_only_px = int(getattr(self.cfg, "APPROACH_STRAFE_ONLY_PX", 0))
		if strafe_only_px > 0 and abs(offset_x) >= strafe_only_px:
			vx = 0

		# 抗抖的 held 目标：属于“上一帧复用”，为安全起见禁止向前，只允许横移。
		if bool(target.get("held", False)):
			vx = 0

		# 圆检测回退更容易误检，默认禁止向前（只允许横移找回 blob 目标）。
		if method == "circle":
			vx = 0

		# 目标太小：更容易是噪声/误检，禁止向前，避免突然直走
		min_size = int(getattr(self.cfg, "APPROACH_FORWARD_MIN_SIZE", 0))
		if min_size > 0 and size < min_size:
			vx = 0

		# 更严格：只有当目标基本居中时才允许向前，否则只横移。
		forward_center_px = int(getattr(self.cfg, "APPROACH_FORWARD_CENTER_PX", 0))
		if forward_center_px > 0 and abs(offset_x) > forward_center_px:
			vx = 0

		# 靠近时更强刹车逻辑：使用二次缩放以更激进减速
		brake_dist = int(getattr(self.cfg, "APPROACH_BRAKE_DIST", 40))
		brake_min = int(getattr(self.cfg, "APPROACH_BRAKE_MIN_SPEED", 6))
		if dist_error < brake_dist and brake_dist > 0:
			# scale in [0,1]
			scale = max(0.0, float(dist_error) / float(brake_dist))
			# 使用平方缩放以更快减速（当接近时速度更小）
			scaled = scale * scale
			max_allowed = max(brake_min, int(self.cfg.APPROACH_MAX_SPEED * scaled))
			if vx > max_allowed:
				vx = max_allowed
			# 当速度非常小可以直接停止，避免低速冲顶
			if vx <= brake_min:
				vx = 0
		if not bool(getattr(self.cfg, "OBJECT_STOP_MODE", False)):
			min_v = float(getattr(self.cfg, "APPROACH_MIN_SPEED", 0))
			if vx > 0 and vx < min_v:
				vx = min_v

		a = float(getattr(self.cfg, "APPROACH_CMD_SMOOTH_ALPHA", 0.55))
		a = clamp(a, 0.05, 1.0)
		vx = self.last_cmd_vx * (1.0 - a) + vx * a
		vy = self.last_cmd_vy * (1.0 - a) + vy * a
		self.last_cmd_vx = vx
		self.last_cmd_vy = vy

		# Progress watchdog: if we command forward for a while but target size never increases,
		# we are likely stuck or tracking a false/static object.
		now = time.ticks_ms()
		progress_vx_min = float(getattr(self.cfg, "APPROACH_PROGRESS_VX_MIN", 8))
		timeout_ms = int(getattr(self.cfg, "APPROACH_PROGRESS_TIMEOUT_MS", 900))
		size_eps = int(getattr(self.cfg, "APPROACH_PROGRESS_SIZE_EPS", 2))
		progress_enable = (not is_held) and (method != "circle") and (dist_error > 0) and (vx >= progress_vx_min)
		if progress_enable:
			if self._approach_progress_start_ms is None or self._approach_progress_best_size is None:
				self._approach_progress_start_ms = now
				self._approach_progress_best_size = int(size)
			else:
				if int(size) >= int(self._approach_progress_best_size) + int(size_eps):
					self._approach_progress_best_size = int(size)
					self._approach_progress_start_ms = now
				elif time.ticks_diff(now, self._approach_progress_start_ms) >= timeout_ms:
					self.chassis.stop()
					self.last_cmd_vx = 0.0
					self.last_cmd_vy = 0.0
					self._approach_progress_start_ms = None
					self._approach_progress_best_size = None
					return "stall"
		else:
			self._approach_progress_start_ms = None
			self._approach_progress_best_size = None

		# Stall watchdog: if we keep generating near-zero commands in APPROACH,
		# it usually means we're stuck due to thresholds/held target jitter.
		# Fallback to search to re-acquire a clean target.
		# (is_held already computed above)
		cmd_deadband = float(getattr(self.cfg, "APPROACH_CMD_DEADBAND", 0.8))
		stalled = (not is_held) and (abs(vx) <= cmd_deadband) and (abs(vy) <= cmd_deadband) and (dist_error > 0)
		if stalled:
			if self._approach_stall_start_ms is None:
				self._approach_stall_start_ms = now
			stall_ms = int(getattr(self.cfg, "APPROACH_STALL_TIMEOUT_MS", 700))
			if time.ticks_diff(now, self._approach_stall_start_ms) >= stall_ms:
				self.chassis.stop()
				self.last_cmd_vx = 0.0
				self.last_cmd_vy = 0.0
				self._approach_stall_start_ms = None
				return "stall"
		else:
			self._approach_stall_start_ms = None

		self.chassis.move(vx, vy, 0)
		return "continue"

	def _vision_x_sign(self):
		"""Return sign for offset_x, supporting dual vision.

		- near uses VISION_OFFSET_X_SIGN
		- far uses VISION2_OFFSET_X_SIGN (fallbacks to VISION_OFFSET_X_SIGN)
		"""
		base = float(getattr(self.cfg, "VISION_OFFSET_X_SIGN", 1))
		src = getattr(self, "_vision_source", None)
		if src == "far":
			return float(getattr(self.cfg, "VISION2_OFFSET_X_SIGN", base))
		return base

	def _signed_offset_x(self, target):
		if not target:
			return 0
		sign = self._vision_x_sign()
		try:
			return int(float(target.get("offset_x", 0)) * float(sign))
		except Exception:
			return int(target.get("offset_x", 0) or 0)

	def _search_zone(self, target):
		offset_x = self._signed_offset_x(target)
		center_error = 0 if abs(offset_x) < self.cfg.CENTER_DEADBAND else offset_x
		vy = self.center_pid.update(center_error)
		self.chassis.move(10, vy, self.cfg.SEARCH_ZONE_ROT_SPEED)

	def _align_push(self, target, zone):
		pair_error = self._signed_offset_x(zone) - self._signed_offset_x(target)
		if abs(pair_error) <= self.cfg.ZONE_ALIGN_DEADBAND:
			self.push_start_ms = time.ticks_ms()
			self._set_state(self.STATE_PUSH)
			return

		vy = self.push_align_pid.update(pair_error)
		object_correction = self.center_pid.update(self._signed_offset_x(target))
		vy = clamp(vy + object_correction * 0.3, -self.cfg.MAX_LATERAL_SPEED, self.cfg.MAX_LATERAL_SPEED)
		self.chassis.move(self.cfg.ALIGN_FORWARD_SPEED, vy, 0)

	def _push(self, target, zone):
		frame = getattr(self, "_last_frame", None)
		if frame and bool(frame.get("object_out_of_field", False)):
			self.chassis.stop()
			self._start_retreat_then_search()
			self.push_start_ms = None
			return

		require_target = bool(getattr(self.cfg, "PUSH_REQUIRE_TARGET", False))
		try:
			target_held = bool(target.get("held", False)) if isinstance(target, dict) else False
		except Exception:
			target_held = False
		try:
			method = target.get("method") if isinstance(target, dict) else None
		except Exception:
			method = None
		try:
			target_size = int(target.get("size", 0)) if isinstance(target, dict) else 0
		except Exception:
			target_size = 0
		min_size = int(getattr(self.cfg, "PUSH_ENTER_MIN_SIZE", 0))
		valid_target = bool(target) and (not target_held) and (target_size >= max(0, min_size)) and (method != "circle")
		if require_target and (not valid_target):
			self.chassis.stop()
			self._set_state(self.STATE_SEARCH_OBJECT)
			self.push_start_ms = None
			self._search_object()
			return

		now = time.ticks_ms()
		if self.push_start_ms is None:
			self.push_start_ms = now

		if time.ticks_diff(now, self.push_start_ms) >= self.cfg.PUSH_TIME_MS:
			self.chassis.stop()
			self._start_retreat_then_search()
			self.push_start_ms = None
			return

		if zone and target:
			pair_error = self._signed_offset_x(zone) - self._signed_offset_x(target)
		elif target:
			pair_error = self._signed_offset_x(target)
		else:
			pair_error = 0

		vy = self.push_align_pid.update(pair_error)
		self.chassis.move(self.cfg.PUSH_SPEED, vy, 0)

	def _start_retreat_then_search(self):
		enable = bool(getattr(self.cfg, "AFTER_PUSH_RETREAT_ENABLE", False))
		ms = int(getattr(self.cfg, "AFTER_PUSH_RETREAT_MS", 0))
		if (not enable) or ms <= 0:
			self._set_state(self.STATE_SEARCH_OBJECT)
			return
		self.retreat_until_ms = time.ticks_add(time.ticks_ms(), ms)
		self._set_state(self.STATE_AFTER_PUSH_RETREAT)

	def _after_push_retreat_step(self):
		if self.retreat_until_ms is None:
			self._set_state(self.STATE_SEARCH_OBJECT)
			self._search_object()
			return
		vx = float(getattr(self.cfg, "AFTER_PUSH_RETREAT_SPEED", -20))
		self.chassis.move(vx, 0, 0)
		if time.ticks_diff(self.retreat_until_ms, time.ticks_ms()) <= 0:
			self.retreat_until_ms = None
			self.chassis.stop()
			self._set_state(self.STATE_SEARCH_OBJECT)
			self._search_object()

	def update(self):
		frame = self.vision.get_latest_frame(timeout_ms=self.cfg.FRAME_TIMEOUT_MS)
		self._last_frame = frame
		# DualVisionReceiver attaches "source": "near"/"far".
		try:
			self._vision_source = frame.get("source") if isinstance(frame, dict) else None
		except Exception:
			self._vision_source = None
		# 安全：若视觉数据过期（连接/串口挂起），立即停止并回到搜索状态
		try:
			last_update = getattr(self.vision, "last_update_ms", None)
			age_ms = None
			if last_update is not None:
				age_ms = time.ticks_diff(time.ticks_ms(), last_update)
			max_age = int(getattr(self.cfg, "VISION_MAX_AGE_MS", int(getattr(self.cfg, "FRAME_TIMEOUT_MS", 500)) * 2))
			# 若无视觉数据或年龄超过阈值，强制停止以防失控
			if age_ms is None or age_ms > max_age:
				debug_enable = bool(getattr(self.cfg, "VISION_DEBUG_ENABLE", False))
				if debug_enable:
					# 只在 stale 状态变化时提示一次，避免刷屏卡串口
					if self._dbg_last_stale is not True:
						print("VISION STALE: age_ms=", age_ms, "max_age_ms=", max_age, "-> stopping and search")
				self._dbg_last_stale = True
				self.chassis.stop()
				self._set_state(self.STATE_SEARCH_OBJECT)
				# Do one search step even when vision is stale. _search_object() will
				# use a limited pulse-rotation window to try to re-acquire safely.
				self._search_object()
				return
			self._dbg_last_stale = False
		except Exception:
			pass
		# 可选调试打印：默认关闭，避免高频 print 影响串口连接
		debug_enable = bool(getattr(self.cfg, "VISION_DEBUG_ENABLE", False))
		debug_interval = int(getattr(self.cfg, "VISION_DEBUG_PRINT_MS", 250))
		now_ms = time.ticks_ms()
		should_print = debug_enable and (time.ticks_diff(now_ms, self._dbg_last_ms) >= debug_interval)
		if should_print:
			self._dbg_last_ms = now_ms
			try:
				age_ms = None
				last_update = getattr(self.vision, "last_update_ms", None)
				if last_update is not None:
					age_ms = time.ticks_diff(now_ms, last_update)
				target_dbg = None
				held_dbg = None
				size_dbg = None
				off_dbg = None
				if isinstance(frame, dict):
					obj = frame.get("object")
					if isinstance(obj, dict):
						target_dbg = True
						held_dbg = bool(obj.get("held", False))
						size_dbg = obj.get("size")
						off_dbg = obj.get("offset_x")
				print(
					"VISION_RX:",
					"got" if frame else "none",
					"src=", getattr(self, "_vision_source", None),
					"age_ms=", age_ms,
					"state=", self.state,
					"obj=", target_dbg,
					"held=", held_dbg,
					"size=", size_dbg,
					"offset_x=", off_dbg,
				)
			except Exception:
				pass
		# 增加目标丢失缓冲，避免短时丢失导致状态机反复切换
		if not hasattr(self, "_target_lost_ms"):
			self._target_lost_ms = 0
		target = frame["object"] if frame else None
		zone = frame["zone"] if frame else None
		if target:
			self._target_lost_ms = 0
		else:
			self._target_lost_ms += self.cfg.LOOP_DELAY_MS
			# 允许目标丢失缓冲：默认关闭（0ms），需要抗抖再按现场开启
			hold_ms = int(getattr(self.cfg, "TARGET_LOST_HOLD_MS", 0))
			if hold_ms > 0 and self._target_lost_ms < hold_ms:
				last_target = getattr(self, "_last_target", None)
				# Safety: reused target is considered "held" (stale), so we won't move forward blindly.
				if isinstance(last_target, dict):
					target = dict(last_target)
					try:
						target["held"] = True
					except Exception:
						pass
				else:
					target = last_target
		self._last_target = target if target else getattr(self, "_last_target", None)

		# If we only have a held (stale/reused) target, do not stay in non-search states.
		# Otherwise it can get stuck in APPROACH with vx=vy=0 and never reacquire.
		try:
			target_held = bool(target.get("held", False)) if isinstance(target, dict) else False
		except Exception:
			target_held = False
		if target_held and self.state != self.STATE_SEARCH_OBJECT:
			self.chassis.stop()
			debug_enable = bool(getattr(self.cfg, "VISION_DEBUG_ENABLE", False))
			if debug_enable:
				try:
					print("TARGET_HELD: fallback to search_object")
				except Exception:
					pass
			self._set_state(self.STATE_SEARCH_OBJECT)
			self._search_object()
			return

		# 详细目标信息默认不打印（需要时打开 VISION_DEBUG_ENABLE）
		if self.state == self.STATE_SEARCH_OBJECT:
			if target and (not bool(target.get("held", False))):
				# 连续确认，避免单帧误检
				min_size = int(getattr(self.cfg, "APPROACH_FORWARD_MIN_SIZE", 0))
				size = int(target.get("size", 0))
				if min_size > 0 and size < min_size:
					self._target_confirm_count = 0
				else:
					self._target_confirm_count += 1
				confirm = int(getattr(self.cfg, "TARGET_CONFIRM_FRAMES", 1))
				if self._target_confirm_count >= max(1, confirm):
					self._set_state(self.STATE_APPROACH_OBJECT)
					return
			else:
				self._target_confirm_count = 0
			self._search_object()
			return

		if self.state == self.STATE_APPROACH_OBJECT:
			if not target:
				self._set_state(self.STATE_SEARCH_OBJECT)
				self._search_object()
				return

			# 仅停车模式：只做“追球并在球前停下”，不进入找区/推送流程。
			# 这样可避免接近球后因 zone 不稳定而切状态导致原地乱转。
			if bool(getattr(self.cfg, "OBJECT_STOP_MODE", False)):
				status = self._approach_object(target)
				if status == "stall":
					self._set_state(self.STATE_SEARCH_OBJECT)
					self._search_object()
				return

			if target["size"] >= self.cfg.TARGET_OBJECT_SIZE:
				if zone:
					self._set_state(self.STATE_ALIGN_PUSH)
				else:
					self._set_state(self.STATE_SEARCH_ZONE)
			else:
				status = self._approach_object(target)
				if status == "stall":
					self._set_state(self.STATE_SEARCH_OBJECT)
					self._search_object()
				return

		if self.state == self.STATE_SEARCH_ZONE:
			if not target:
				self._set_state(self.STATE_SEARCH_OBJECT)
				self._search_object()
				return

			if zone:
				self._set_state(self.STATE_ALIGN_PUSH)
			else:
				self._search_zone(target)
				return

		if self.state == self.STATE_ALIGN_PUSH:
			if not target:
				self._set_state(self.STATE_SEARCH_OBJECT)
				self._search_object()
				return

			if not zone:
				self._set_state(self.STATE_SEARCH_ZONE)
				self._search_zone(target)
				return

			self._align_push(target, zone)
			return

		if self.state == self.STATE_PUSH:
			self._push(target, zone)
			return

		if self.state == self.STATE_AFTER_PUSH_RETREAT:
			self._after_push_retreat_step()
			return

	def run_forever(self):
		while True:
			try:
				self.update()
			except Exception as e:
				import sys
				print("CRASH IN UPDATE", repr(e))
				sys.print_exception(e)
				raise
			time.sleep_ms(self.cfg.LOOP_DELAY_MS)
