import unittest

from Module import config


class FieldConfigTest(unittest.TestCase):
	def test_field_size_and_center_target(self):
		self.assertAlmostEqual(config.FIELD_SIZE_X_M, 3.2)
		self.assertAlmostEqual(config.FIELD_SIZE_Y_M, 2.8)
		self.assertAlmostEqual(config.CENTER_TARGET_X, 1.6)
		self.assertAlmostEqual(config.CENTER_TARGET_Y, 1.4)

	def test_competition_center_flow_enabled(self):
		self.assertFalse(config.COMPETITION_FORCE_BASIC_CONTROLLER)
		self.assertTrue(config.CENTER_FIRST_ENABLE)


if __name__ == "__main__":
	unittest.main()
