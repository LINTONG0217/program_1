import time
import unittest


if not hasattr(time, "ticks_ms"):
	time.ticks_ms = lambda: int(time.time() * 1000)
	time.ticks_diff = lambda now, then: now - then

from BSP.omni_chassis_driver import OmniChassis


class FakeMotor:
	def __init__(self):
		self.speed = 0

	def set_speed(self, speed):
		self.speed = speed

	def stop(self):
		self.speed = 0


class Config:
	CHASSIS_VX_SIGN = 1.0
	CHASSIS_VY_SIGN = -1.0
	CHASSIS_VW_SIGN = 1.0
	CHASSIS_CMD_DEADBAND = 0
	CHASSIS_MAX_VX_STEP = 0
	CHASSIS_MAX_VY_STEP = 0
	CHASSIS_MAX_VW_STEP = 0
	CHASSIS_ENABLE_CLOSED_LOOP = False


class ChassisDriverMixTest(unittest.TestCase):
	def build_chassis(self):
		return OmniChassis(FakeMotor(), FakeMotor(), FakeMotor(), FakeMotor(), config=Config)

	def assert_targets(self, chassis, expected):
		actual = [
			chassis.wheel_targets["fl"],
			chassis.wheel_targets["fr"],
			chassis.wheel_targets["bl"],
			chassis.wheel_targets["br"],
		]
		self.assertEqual(actual, expected)

	def test_forward_matches_manual_chassis_raw(self):
		chassis = self.build_chassis()
		chassis.forward(55)
		self.assert_targets(chassis, [55, -55, 55, -55])

	def test_strafe_right_matches_manual_chassis_raw(self):
		chassis = self.build_chassis()
		chassis.strafe(55)
		self.assert_targets(chassis, [-55, -55, 55, 55])

	def test_rotate_cw_matches_manual_chassis_raw(self):
		chassis = self.build_chassis()
		chassis.rotate(45)
		self.assert_targets(chassis, [45, 45, 45, 45])


if __name__ == "__main__":
	unittest.main()
