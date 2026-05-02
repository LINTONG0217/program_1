import unittest

from Module.task_controller_competition import CompetitionController


class CompetitionCenterNavigationTest(unittest.TestCase):
	def setUp(self):
		self.controller = object.__new__(CompetitionController)

	class _Config:
		FIELD_SIZE_X_M = 3.2
		FIELD_SIZE_Y_M = 2.8
		CENTER_YAW_DEADBAND_DEG = 1.5
		CENTER_YAW_KP = 1.2
		CENTER_YAW_MAX_SPEED = 24.0
		CENTER_YAW_SIGN = 1.0
		PUSH_CARDINAL_MODE = "nearest_boundary"

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


if __name__ == "__main__":
	unittest.main()
