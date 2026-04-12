"""Lidar 轻量推送控制器。

基于 SmartCarController，仅补齐“无 zone 也可推送”的状态跳转，
用于 competition/basic fallback 或 single 模式。
"""

from Module.task_controller_basic import SmartCarController


class LidarSmartPushController(SmartCarController):
	def update(self):
		SmartCarController.update(self)

		if self.state == self.STATE_PUSH:
			return

		frame = getattr(self, "_last_frame", None)
		target = frame.get("object") if isinstance(frame, dict) else None
		if not isinstance(target, dict):
			return

		if bool(target.get("held", False)):
			return

		size = int(target.get("size", 0) or 0)
		min_size = int(getattr(self.cfg, "PUSH_ENTER_MIN_SIZE", 0))
		trigger_size = int(getattr(self.cfg, "TARGET_OBJECT_SIZE", 0))
		need = max(1, min_size, trigger_size)

		if size < need:
			return

		if self.state in (self.STATE_SEARCH_ZONE, self.STATE_ALIGN_PUSH):
			self.push_start_ms = None
			self._set_state(self.STATE_PUSH)
