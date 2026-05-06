import unittest

from Module.omni_pid_navigation import OmniController


class OmniPidNavigationTest(unittest.TestCase):
	def test_wheel_to_body_matches_chassis_mix(self):
		controller = OmniController()

		self.assertEqual(controller._wheel_to_body([1.0, -1.0, 1.0, -1.0]), (1.0, 0.0, 0.0))
		self.assertEqual(controller._wheel_to_body([-1.0, -1.0, 1.0, 1.0]), (0.0, 1.0, 0.0))
		self.assertEqual(controller._wheel_to_body([1.0, 1.0, 1.0, 1.0]), (0.0, 0.0, 1.0))


if __name__ == "__main__":
	unittest.main()
