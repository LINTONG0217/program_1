"""比赛任务控制器。"""

import math
import time

from Module.task_controller_basic import SmartCarController, clamp


class CompetitionController(SmartCarController):
	STATE_AVOID = "avoid"
	STATE_WAIT_BOTH_READY = "wait_both_ready"
	STATE_COOP_PUSH = "coop_push"
	STATE_AFTER_PUSH_RETREAT = "after_push_retreat"
	STATE_RETURN_HOME = "return_home"
	STATE_WAIT_NEXT_TASK = "wait_next_task"
	STATE_MISSION_COMPLETE = "mission_complete"

	def __init__(self, chassis, vision, obstacle_sensor, config, ultrasonic=None, pose_provider=None, remote_ready_fn=None, remote_home_fn=None):
		try:
			print("CompetitionController: __init__ enter")
		except Exception:
			pass
		try:
			super().__init__(chassis, vision, config)
			try:
				print("CompetitionController: after super")
			except Exception:
				pass
		except Exception as e:
			try:
				import sys
				print("CompetitionController: super init failed:", repr(e))
				sys.print_exception(e)
			except Exception:
				pass
			raise
		self.obstacle_sensor = obstacle_sensor
		self.ultrasonic = ultrasonic
		self.pose_provider = pose_provider
		self.remote_ready_fn = remote_ready_fn
		self.remote_home_fn = remote_home_fn
		self.avoid_until_ms = None
		self.avoid_vx = 0
		self.avoid_vy = 0
		self.avoid_vw = 0
		self.return_start_ms = None
		self.wait_next_task_start_ms = None
		self.search_empty_start_ms = None
		self.push_target_lost_count = 0
		self.completed_push_count = 0
		self.retreat_until_ms = None
		self.mission_finish_pending = False
		self.mission_return_stable_ms = None
		self.drop_points = []

		if callable(self.remote_ready_fn) and bool(getattr(self.cfg, "DUAL_REQUIRE_SLAVE_READY", True)):
			self._set_state(self.STATE_WAIT_BOTH_READY)

	def _target_is_important(self, target):
		return bool(target and target["size"] >= self.cfg.OBJECT_LOCK_SIZE)

	def _get_pose(self):
		if not callable(self.pose_provider):
			return None
		try:
			return self.pose_provider()
		except Exception:
			return None

	def _field_center(self):
		return float(self.cfg.FIELD_SIZE_X_M) * 0.5, float(self.cfg.FIELD_SIZE_Y_M) * 0.5

	def _in_center_zone(self, pose):
		if not pose:
			return False
		cx, cy = self._field_center()
		half = float(getattr(self.cfg, "CENTER_ZONE_SIZE_M", 1.0)) * 0.5
		x = float(pose.get("x", 0.0))
		y = float(pose.get("y", 0.0))
		return abs(x - cx) <= half and abs(y - cy) <= half

	def _drop_spacing_ok(self, x, y):
		min_spacing = float(getattr(self.cfg, "MIN_DROP_SPACING_M", 0.10))
		for item in self.drop_points:
			dx = x - item["x"]
			dy = y - item["y"]
			if math.sqrt(dx * dx + dy * dy) < min_spacing:
				return False
		return True

	def _record_drop_result(self):
		pose = self._get_pose()
		if not pose:
			print("drop check: pose unavailable")
			return

		x = float(pose.get("x", 0.0))
		y = float(pose.get("y", 0.0))
		in_center = self._in_center_zone(pose)
		spacing_ok = self._drop_spacing_ok(x, y)

		if in_center and spacing_ok:
			self.drop_points.append({"x": x, "y": y, "ts": time.ticks_ms()})
			max_keep = int(getattr(self.cfg, "MAX_DROPS_TRACKED", 16))
			if len(self.drop_points) > max_keep:
				self.drop_points = self.drop_points[-max_keep:]
			print("drop check: ok", "x={:.2f}".format(x), "y={:.2f}".format(y), "count=", len(self.drop_points))
			return

		print(
			"drop check: fail",
			"in_center=" + str(in_center),
			"spacing_ok=" + str(spacing_ok),
			"x={:.2f}".format(x),
			"y={:.2f}".format(y),
		)

	def _in_start_zone(self, pose):
		if not pose:
			return False
		x = float(pose.get("x", 0.0))
		y = float(pose.get("y", 0.0))
		return (
			float(getattr(self.cfg, "START_ZONE_X_MIN", 0.0)) <= x <= float(getattr(self.cfg, "START_ZONE_X_MAX", 0.5))
			and float(getattr(self.cfg, "START_ZONE_Y_MIN", 0.0)) <= y <= float(getattr(self.cfg, "START_ZONE_Y_MAX", 0.5))
		)

	def _wait_both_ready_step(self):
		ready = True if not callable(self.remote_ready_fn) else bool(self.remote_ready_fn())
		if ready:
			self._set_state(self.STATE_SEARCH_OBJECT)
			return
		self.chassis.stop()

	def _object_is_out(self, frame):
		if not frame:
			return False
		if bool(frame.get("object_out_of_field", False)):
			return True

		target = frame.get("object")
		if not target:
			return False

		margin = int(getattr(self.cfg, "OBJECT_OUT_MARGIN_PX", 14))
		x = int(target.get("x", self.cfg.FRAME_WIDTH // 2))
		y = int(target.get("y", self.cfg.FRAME_HEIGHT // 2))
		w = int(frame.get("frame", {}).get("w", self.cfg.FRAME_WIDTH))
		h = int(frame.get("frame", {}).get("h", self.cfg.FRAME_HEIGHT))
		return x <= margin or x >= w - margin or y <= margin or y >= h - margin

	def _start_search_again(self):
		self.push_target_lost_count = 0
		if callable(self.remote_ready_fn) and bool(getattr(self.cfg, "DUAL_REQUIRE_SLAVE_READY", True)):
			self._set_state(self.STATE_WAIT_BOTH_READY)
		else:
			self.wait_next_task_start_ms = time.ticks_ms()
			self._set_state(self.STATE_WAIT_NEXT_TASK)

	def _start_retreat_then_search(self):
		"""推球完成后短暂后退回场内，再继续搜下一个。"""
		enable = bool(getattr(self.cfg, "AFTER_PUSH_RETREAT_ENABLE", False))
		ms = int(getattr(self.cfg, "AFTER_PUSH_RETREAT_MS", 0))
		if (not enable) or ms <= 0:
			self._start_search_again()
			return
		self.retreat_until_ms = time.ticks_add(time.ticks_ms(), ms)
		self._set_state(self.STATE_AFTER_PUSH_RETREAT)

	def _after_push_retreat_step(self):
		if self.retreat_until_ms is None:
			self._start_search_again()
			return
		vx = float(getattr(self.cfg, "AFTER_PUSH_RETREAT_SPEED", -20))
		self.chassis.move(vx, 0, 0)
		if time.ticks_diff(self.retreat_until_ms, time.ticks_ms()) <= 0:
			self.retreat_until_ms = None
			self.chassis.stop()
			self._start_search_again()

	def _maybe_finish_mission(self, target):
		if self.state in (self.STATE_RETURN_HOME, self.STATE_MISSION_COMPLETE):
			return False

		if target:
			self.search_empty_start_ms = None
			return False

		if self.state != self.STATE_SEARCH_OBJECT:
			self.search_empty_start_ms = None
			return False

		now = time.ticks_ms()
		if self.search_empty_start_ms is None:
			self.search_empty_start_ms = now
			return False

		confirm_ms = int(getattr(self.cfg, "MISSION_EMPTY_CONFIRM_MS", 5000))
		if time.ticks_diff(now, self.search_empty_start_ms) < confirm_ms:
			return False

		if self.completed_push_count <= 0:
			# 启动阶段防误判：至少完成过一次有效推送后才允许“清场完成”。
			return False

		print("mission clear confirmed", "push_count=", self.completed_push_count)
		self.mission_finish_pending = True
		if bool(getattr(self.cfg, "RETURN_HOME_ENABLE", True)):
			self.return_start_ms = now
			self._set_state(self.STATE_RETURN_HOME)
		else:
			self._set_state(self.STATE_MISSION_COMPLETE)
		return True

	def _mission_return_confirm_step(self):
		pose = self._get_pose()
		local_ok = self._in_start_zone(pose)

		remote_fn = getattr(self, "remote_home_fn", None)
		if callable(remote_fn):
			remote_ok = bool(remote_fn())
		elif callable(self.remote_ready_fn) and bool(getattr(self.cfg, "DUAL_REQUIRE_SLAVE_READY", True)):
			# 没有远端位姿时，至少要求链路在线。
			remote_ok = bool(self.remote_ready_fn())
		else:
			remote_ok = True

		if not (local_ok and remote_ok):
			self.mission_return_stable_ms = None
			return False

		now = time.ticks_ms()
		if self.mission_return_stable_ms is None:
			self.mission_return_stable_ms = now
			return False

		stable_ms = int(getattr(self.cfg, "MISSION_RETURN_STABLE_MS", 1200))
		if time.ticks_diff(now, self.mission_return_stable_ms) < stable_ms:
			return False

		return True

	def _obstacle_detected(self):
		state = self.obstacle_sensor.read() if self.obstacle_sensor else {"left": False, "front": False, "right": False}
		distance_hit = False
		if self.ultrasonic:
			distance = self.ultrasonic.distance_cm()
			distance_hit = bool(distance is not None and distance <= self.cfg.OBSTACLE_DISTANCE_CM)
		return state, distance_hit

	def _plan_avoid(self, obstacle_state):
		left = obstacle_state.get("left")
		front = obstacle_state.get("front")
		right = obstacle_state.get("right")

		if front and left and right:
			self.avoid_vx = self.cfg.AVOID_REVERSE_SPEED
			self.avoid_vy = 0
			self.avoid_vw = self.cfg.AVOID_ROTATE_SPEED
		elif front and left:
			self.avoid_vx = self.cfg.AVOID_REVERSE_SPEED
			self.avoid_vy = self.cfg.AVOID_STRAFE_SPEED
			self.avoid_vw = self.cfg.AVOID_ROTATE_SPEED
		elif front and right:
			self.avoid_vx = self.cfg.AVOID_REVERSE_SPEED
			self.avoid_vy = -self.cfg.AVOID_STRAFE_SPEED
			self.avoid_vw = -self.cfg.AVOID_ROTATE_SPEED
		elif front:
			self.avoid_vx = self.cfg.AVOID_REVERSE_SPEED
			self.avoid_vy = self.cfg.AVOID_STRAFE_SPEED
			self.avoid_vw = 0
		elif left:
			self.avoid_vx = 0
			self.avoid_vy = self.cfg.AVOID_STRAFE_SPEED
			self.avoid_vw = self.cfg.AVOID_ROTATE_SPEED // 2
		elif right:
			self.avoid_vx = 0
			self.avoid_vy = -self.cfg.AVOID_STRAFE_SPEED
			self.avoid_vw = -self.cfg.AVOID_ROTATE_SPEED // 2
		else:
			self.avoid_vx = self.cfg.AVOID_REVERSE_SPEED
			self.avoid_vy = 0
			self.avoid_vw = self.cfg.AVOID_ROTATE_SPEED

		self.avoid_until_ms = time.ticks_add(time.ticks_ms(), self.cfg.AVOID_ACTION_MS)
		self._set_state(self.STATE_AVOID)

	def _do_avoid(self):
		self.chassis.move(self.avoid_vx, self.avoid_vy, self.avoid_vw)
		if self.avoid_until_ms is None:
			self._set_state(self.STATE_SEARCH_OBJECT)
			return True

		if time.ticks_diff(self.avoid_until_ms, time.ticks_ms()) <= 0:
			self.avoid_until_ms = None
			self.chassis.stop()
			self._set_state(self.STATE_SEARCH_OBJECT)
			return True
		return False

	def _coop_push(self, frame):
		target = frame.get("object") if frame else None
		if target and bool(target.get("held", False)):
			target = None
		zone = frame.get("zone") if frame else None
		now = time.ticks_ms()
		if self.push_start_ms is None:
			self.push_start_ms = now
			self.push_target_lost_count = 0

		if self._object_is_out(frame):
			self.chassis.stop()
			self.completed_push_count += 1
			print("object out detected", "count=", self.completed_push_count)
			self.push_start_ms = None
			self._start_retreat_then_search()
			return

		if not target:
			self.push_target_lost_count += 1
			# 更安全：目标丢失时先停住，避免“看不到球还一直直走”
			if bool(getattr(self.cfg, "PUSH_REQUIRE_TARGET", True)):
				self.chassis.stop()
			if self.push_target_lost_count >= int(getattr(self.cfg, "PUSH_LOST_CONFIRM_FRAMES", 3)):
				self.chassis.stop()
				self.completed_push_count += 1
				print("push target lost confirmed", "count=", self.completed_push_count)
				self.push_start_ms = None
				self._start_retreat_then_search()
				return
		else:
			self.push_target_lost_count = 0

		if time.ticks_diff(now, self.push_start_ms) >= self.cfg.PUSH_TIME_MS:
			self.chassis.stop()
			self.completed_push_count += 1
			print("push cycle done", "count=", self.completed_push_count)
			self.push_start_ms = None
			self._start_retreat_then_search()
			return

		obstacle_state, distance_hit = self._obstacle_detected()

		if zone and target:
			pair_error = zone["offset_x"] - target["offset_x"]
		elif target:
			pair_error = target["offset_x"]
		else:
			pair_error = 0

		vy = self.push_align_pid.update(pair_error)
		safe_lateral = float(getattr(self.cfg, "ASSIST_SAFE_LATERAL_LIMIT", 15))
		vy = clamp(vy, -safe_lateral, safe_lateral)
		vx = self.cfg.PUSH_SPEED

		if obstacle_state.get("front") or distance_hit:
			vx = clamp(self.cfg.PUSH_SPEED * 0.6, 0, 100)
			if obstacle_state.get("left"):
				vy = self.cfg.AVOID_STRAFE_SPEED * 0.4
			elif obstacle_state.get("right"):
				vy = -self.cfg.AVOID_STRAFE_SPEED * 0.4

		self.chassis.move(vx, vy, 0)

	def _return_home_step(self):
		now = time.ticks_ms()
		if self.return_start_ms is None:
			self.return_start_ms = now

		pose = self._get_pose()
		if pose and self._in_start_zone(pose):
			self.chassis.stop()
			if self.mission_finish_pending:
				if self._mission_return_confirm_step():
					self._set_state(self.STATE_MISSION_COMPLETE)
			else:
				self.wait_next_task_start_ms = now
				self._set_state(self.STATE_WAIT_NEXT_TASK)
			return
		self.mission_return_stable_ms = None

		timeout_ms = int(getattr(self.cfg, "RETURN_HOME_TIMEOUT_MS", 8000))
		if time.ticks_diff(now, self.return_start_ms) >= timeout_ms:
			print("return home timeout")
			self.chassis.stop()
			self.wait_next_task_start_ms = now
			self._set_state(self.STATE_WAIT_NEXT_TASK)
			return

		if not pose:
			self.chassis.move(-15, -10, 0)
			return

		target_x = float(getattr(self.cfg, "HOME_TARGET_X", 0.25))
		target_y = float(getattr(self.cfg, "HOME_TARGET_Y", 0.25))
		ex = target_x - float(pose.get("x", 0.0))
		ey = target_y - float(pose.get("y", 0.0))
		kp = float(getattr(self.cfg, "RETURN_HOME_KP", 120.0))
		max_speed = float(getattr(self.cfg, "RETURN_HOME_MAX_SPEED", 45))
		vx = clamp(ex * kp, -max_speed, max_speed)
		vy = clamp(ey * kp, -max_speed, max_speed)
		self.chassis.move(vx, vy, 0)

	def _wait_next_task_step(self):
		self.chassis.stop()
		if self.wait_next_task_start_ms is None:
			self.wait_next_task_start_ms = time.ticks_ms()
			return

		if time.ticks_diff(time.ticks_ms(), self.wait_next_task_start_ms) < int(getattr(self.cfg, "NEXT_TASK_WAIT_MS", 600)):
			return

		if callable(self.remote_ready_fn) and bool(getattr(self.cfg, "DUAL_REQUIRE_SLAVE_READY", True)):
			self._set_state(self.STATE_WAIT_BOTH_READY)
		else:
			self._set_state(self.STATE_SEARCH_OBJECT)

	def _set_state(self, new_state):
		old = self.state
		super()._set_state(new_state)
		if old == new_state:
			return

		if new_state != self.STATE_RETURN_HOME:
			self.return_start_ms = None
		if new_state != self.STATE_WAIT_NEXT_TASK:
			self.wait_next_task_start_ms = None
		if new_state != self.STATE_AFTER_PUSH_RETREAT:
			self.retreat_until_ms = None
		if new_state != self.STATE_SEARCH_OBJECT:
			self.search_empty_start_ms = None
		if new_state != self.STATE_RETURN_HOME:
			self.mission_return_stable_ms = None

		if new_state == self.STATE_WAIT_BOTH_READY:
			self.push_start_ms = None
			self.push_target_lost_count = 0
			self.retreat_until_ms = None

		if new_state == self.STATE_COOP_PUSH and self.push_start_ms is None:
			self.push_start_ms = time.ticks_ms()
			self.push_target_lost_count = 0
			self.retreat_until_ms = None

	def update(self):
		if self.state == self.STATE_MISSION_COMPLETE:
			self.chassis.stop()
			return

		if self.state == self.STATE_WAIT_BOTH_READY:
			self._wait_both_ready_step()
			return

		if self.state == self.STATE_COOP_PUSH:
			frame = self.vision.get_latest_frame(timeout_ms=self.cfg.FRAME_TIMEOUT_MS)
			self._coop_push(frame or {})
			return

		if self.state == self.STATE_AFTER_PUSH_RETREAT:
			self._after_push_retreat_step()
			return

		if self.state == self.STATE_RETURN_HOME:
			self._return_home_step()
			return

		if self.state == self.STATE_WAIT_NEXT_TASK:
			self._wait_next_task_step()
			return

		frame = self.vision.get_latest_frame(timeout_ms=self.cfg.FRAME_TIMEOUT_MS)
		target = frame["object"] if frame else None
		if target and bool(target.get("held", False)):
			target = None

		if self._maybe_finish_mission(target):
			return

		if self.state == self.STATE_AVOID:
			if self._do_avoid():
				return
			return

		obstacle_state, distance_hit = self._obstacle_detected()
		obstacle_hit = obstacle_state.get("front") or obstacle_state.get("left") or obstacle_state.get("right") or distance_hit

		if obstacle_hit and not self._target_is_important(target):
			self._plan_avoid(obstacle_state)
			self._do_avoid()
			return

		super().update()
		# 清场模式：不依赖 zone，接近到阈值后直接进入协同推送。
		if self.state in (self.STATE_SEARCH_ZONE, self.STATE_ALIGN_PUSH):
			if target and target.get("size", 0) >= int(getattr(self.cfg, "TARGET_OBJECT_SIZE", 90)):
				self._set_state(self.STATE_COOP_PUSH)
				return
			min_push = int(getattr(self.cfg, "PUSH_ENTER_MIN_SIZE", 0))
			if bool(getattr(self.cfg, "COMPETITION_DIRECT_PUSH_NO_ZONE", True)) and target and int(target.get("size", 0)) >= max(0, min_push):
				self._set_state(self.STATE_COOP_PUSH)
				return

		if self.state == self.STATE_PUSH:
			self._set_state(self.STATE_COOP_PUSH)
