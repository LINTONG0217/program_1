import unittest

from Module import config


class FieldConfigTest(unittest.TestCase):
	def test_field_size_and_center_target(self):
		self.assertAlmostEqual(config.FIELD_SIZE_X_M, 3.2)
		self.assertAlmostEqual(config.FIELD_SIZE_Y_M, 2.8)
		self.assertAlmostEqual(config.CENTER_TARGET_X, 1.0)
		self.assertAlmostEqual(config.CENTER_TARGET_Y, 1.5)

	def test_competition_center_flow_enabled(self):
		self.assertFalse(config.COMPETITION_FORCE_BASIC_CONTROLLER)
		self.assertTrue(config.CENTER_FIRST_ENABLE)
		self.assertTrue(config.CENTER_LOCK_CURRENT_YAW)
		self.assertTrue(config.CENTER_NAV_AXIS_SEQUENCE_ENABLE)
		self.assertEqual(config.CHASSIS_VY_SIGN, -1.0)

	def test_heading_lock_signs_are_consistent(self):
		self.assertEqual(config.HEADING_LOCK_YAW_SIGN, 1.0)
		self.assertEqual(config.CENTER_YAW_SIGN, config.HEADING_LOCK_YAW_SIGN)
		self.assertEqual(config.PUSH_YAW_SIGN, config.HEADING_LOCK_YAW_SIGN)


if __name__ == "__main__":
	unittest.main()
