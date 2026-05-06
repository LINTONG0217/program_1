import unittest

from Module import task_controller_competition as competition_module
from Module.task_controller_competition import CompetitionController


class CompetitionCenterNavigationTest(unittest.TestCase):
	def setUp(self):
		self.controller = object.__new__(CompetitionController)

	class _Config:
		FIELD_SIZE_X_M = 3.2
		FIELD_SIZE_Y_M = 2.8
		CENTER_FIRST_ENABLE = True
		CENTER_YAW_DEADBAND_DEG = 1.5
		CENTER_YAW_KP = 1.2
		CENTER_YAW_MAX_SPEED = 24.0
		CENTER_YAW_SIGN = 1.0
		CENTER_LOCK_CURRENT_YAW = True
		CENTER_NAV_KP = 90.0
		CENTER_NAV_MAX_SPEED = 38.0
		CENTER_NAV_MIN_SPEED = 10.0
		CENTER_NAV_TOLERANCE_M = 0.08
		CENTER_NAV_STABLE_MS = 250
		CENTER_NAV_TIMEOUT_MS = 10000
		CENTER_NAV_AXIS_SEQUENCE_ENABLE = True
		CENTER_YAW_TOLERANCE_DEG = 5.0
		HEADING_LOCK_YAW_SIGN = 1.0
		PUSH_CARDINAL_MODE = "nearest_boundary"

	class _Chassis:
		def __init__(self):
			self.moves = []
			self.stopped = False

		def move(self, vx, vy, vw):
			self.moves.append((vx, vy, vw))

		def stop(self):
			self.stopped = True

	def test_center_target_defaults_to_field_center(self):
		self.controller.cfg = self._Config()

		self.assertEqual(self.controller._center_target(), (1.6, 1.4))

	def test_angle_error_wraps_short_way(self):
		self.assertAlmostEqual(self.controller._angle_error(5.0, 355.0), 10.0)
		self.assertAlmostEqual(self.controller._angle_error(355.0, 5.0), -10.0)

	def test_world_to_body_respects_yaw(self):
		vx, vy = self.controller._world_to_body(1.0, 0.0, 90.0)

		self.assertAlmostEqual(vx, 0.0, places=6)
		self.assertAlmostEqual(vy, -1.0, places=6)

	def test_heading_lock_vw_respects_deadband_and_sign(self):
		self.controller.cfg = self._Config()

		vw, err = self.controller._heading_lock_vw(0.0, 359.0, "CENTER")
		self.assertEqual(vw, 0.0)
		self.assertAlmostEqual(err, 1.0)

		vw, err = self.controller._heading_lock_vw(0.0, 10.0, "CENTER")
		self.assertAlmostEqual(vw, -12.0)
		self.assertAlmostEqual(err, -10.0)

		self.controller.cfg.CENTER_YAW_SIGN = -1.0
		vw, err = self.controller._heading_lock_vw(0.0, 10.0, "CENTER")
		self.assertAlmostEqual(vw, 12.0)
		self.assertAlmostEqual(err, -10.0)

	def test_snap_cardinal_yaw(self):
		self.controller.cfg = self._Config()

		self.assertAlmostEqual(self.controller._snap_cardinal_yaw(44.0), 0.0)
		self.assertAlmostEqual(self.controller._snap_cardinal_yaw(46.0), 90.0)
		self.assertAlmostEqual(self.controller._snap_cardinal_yaw(181.0), 180.0)
		self.assertAlmostEqual(self.controller._snap_cardinal_yaw(315.0), 0.0)

	def test_nearest_boundary_yaw(self):
		self.controller.cfg = self._Config()

		self.assertAlmostEqual(self.controller._nearest_boundary_yaw({"x": 0.2, "y": 1.4}), 180.0)
		self.assertAlmostEqual(self.controller._nearest_boundary_yaw({"x": 3.0, "y": 1.4}), 0.0)
		self.assertAlmostEqual(self.controller._nearest_boundary_yaw({"x": 1.6, "y": 0.2}), 270.0)
		self.assertAlmostEqual(self.controller._nearest_boundary_yaw({"x": 1.6, "y": 2.6}), 90.0)

	def test_after_push_returns_to_center_when_center_enabled(self):
		self.controller.cfg = self._Config()
		self.controller.remote_ready_fn = None
		self.controller.push_target_lost_count = 3

		self.controller._set_state = lambda state: setattr(self.controller, "state", state)

		self.controller._start_search_again()

		self.assertEqual(self.controller.state, CompetitionController.STATE_GO_CENTER)

	def test_go_center_runs_x_leg_before_y_leg(self):
		old_ticks_ms = getattr(competition_module.time, "ticks_ms", None)
		old_ticks_diff = getattr(competition_module.time, "ticks_diff", None)
		competition_module.time.ticks_ms = lambda: 1000
		competition_module.time.ticks_diff = lambda now, start: now - start
		try:
			self.controller.cfg = self._Config()
			self.controller.cfg.CENTER_TARGET_X = 1.0
			self.controller.cfg.CENTER_TARGET_Y = 1.5
			self.controller.chassis = self._Chassis()
			self.controller._center_pose_warned = False
			self.controller.center_nav_start_ms = None
			self.controller.center_yaw_lock = None
			self.controller.center_stable_start_ms = None
			self.controller._get_pose = lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0}

			self.controller._go_center_step()
			vx, vy, vw = self.controller.chassis.moves[-1]
			self.assertGreater(vx, 0.0)
			self.assertEqual(vy, 0.0)
			self.assertEqual(vw, 0.0)

			self.controller.chassis = self._Chassis()
			self.controller._get_pose = lambda: {"x": 1.0, "y": 0.0, "yaw": 0.0}

			self.controller._go_center_step()
			vx, vy, vw = self.controller.chassis.moves[-1]
			self.assertEqual(vx, 0.0)
			self.assertGreater(vy, 0.0)
			self.assertEqual(vw, 0.0)
		finally:
			if old_ticks_ms is None:
				del competition_module.time.ticks_ms
			else:
				competition_module.time.ticks_ms = old_ticks_ms
			if old_ticks_diff is None:
				del competition_module.time.ticks_diff
			else:
				competition_module.time.ticks_diff = old_ticks_diff


if __name__ == "__main__":
	unittest.main()
