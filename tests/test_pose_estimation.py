import unittest

import Module.pose_estimation as pose_estimation


class _Clock:
	def __init__(self):
		self.now = 0

	def ticks_ms(self):
		return self.now

	def ticks_diff(self, a, b):
		return a - b


class _Config:
	POSE_MAX_DT_MS = 100
	ODOM_LINEAR_SCALE = 1.0
	ODOM_ANGULAR_SCALE = 1.0
	IMU_FUSION_Q_ANGLE = 0.001
	IMU_FUSION_Q_GYRO = 0.003
	IMU_FUSION_R_ANGLE = 0.03


class PoseEstimationTest(unittest.TestCase):
	def setUp(self):
		self._old_ticks_ms = getattr(pose_estimation.time, "ticks_ms", None)
		self._old_ticks_diff = getattr(pose_estimation.time, "ticks_diff", None)
		self.clock = _Clock()
		pose_estimation.time.ticks_ms = self.clock.ticks_ms
		pose_estimation.time.ticks_diff = self.clock.ticks_diff

	def tearDown(self):
		if self._old_ticks_ms is not None:
			pose_estimation.time.ticks_ms = self._old_ticks_ms
		if self._old_ticks_diff is not None:
			pose_estimation.time.ticks_diff = self._old_ticks_diff

	def test_pose_tracks_encoder_distance(self):
		estimator = pose_estimation.PoseEstimator(_Config())

		self.clock.now = 100
		pose = estimator.update({"fl": 1.0, "fr": 1.0, "bl": 1.0, "br": 1.0}, None)

		self.assertAlmostEqual(pose["x"], 0.1)
		self.assertAlmostEqual(pose["y"], 0.0)
		self.assertAlmostEqual(pose["distance_m"], 0.1)


if __name__ == "__main__":
	unittest.main()
