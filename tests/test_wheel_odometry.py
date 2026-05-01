import unittest

from Module.wheel_odometry import AdvancedOdometry


class WheelOdometryTest(unittest.TestCase):
	def test_forward_motion_updates_x_and_distance(self):
		odom = AdvancedOdometry()
		odom.reset(0.0)

		odom.update([1.0, 1.0, 1.0, 1.0], 0.0, dt=1.0)

		x, y, yaw = odom.get_position()
		self.assertAlmostEqual(x, 1.0)
		self.assertAlmostEqual(y, 0.0)
		self.assertAlmostEqual(yaw, 0.0)
		self.assertAlmostEqual(odom.get_distance(), 1.0)

	def test_strafe_motion_updates_positive_y(self):
		odom = AdvancedOdometry()
		odom.reset(0.0)

		odom.update([1.0, -1.0, -1.0, 1.0], 0.0, dt=1.0)

		x, y, _ = odom.get_position()
		self.assertAlmostEqual(x, 0.0)
		self.assertAlmostEqual(y, 1.0)
		self.assertAlmostEqual(odom.get_distance(), 1.0)

	def test_yaw_rotates_local_motion_into_world(self):
		odom = AdvancedOdometry()
		odom.reset(0.0)

		odom.update([1.0, 1.0, 1.0, 1.0], 90.0, dt=1.0)

		x, y, _ = odom.get_position()
		self.assertAlmostEqual(x, 0.0, places=6)
		self.assertAlmostEqual(y, 1.0, places=6)


if __name__ == "__main__":
	unittest.main()
