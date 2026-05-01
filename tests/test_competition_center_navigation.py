import unittest

from Module.task_controller_competition import CompetitionController


class CompetitionCenterNavigationTest(unittest.TestCase):
	def setUp(self):
		self.controller = object.__new__(CompetitionController)

	class _Config:
		FIELD_SIZE_X_M = 3.2
		FIELD_SIZE_Y_M = 2.8

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


if __name__ == "__main__":
	unittest.main()
