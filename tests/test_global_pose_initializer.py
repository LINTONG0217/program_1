import unittest

from Module.global_pose_initializer import _world_corner_xy


class GlobalPoseInitializerTest(unittest.TestCase):
	class _Config:
		FIELD_SIZE_X_M = 3.2
		FIELD_SIZE_Y_M = 2.8

	def test_world_corner_uses_configured_field_size(self):
		cfg = self._Config()

		self.assertEqual(_world_corner_xy(cfg, "BL"), (0.0, 0.0))
		self.assertEqual(_world_corner_xy(cfg, "BR"), (3.2, 0.0))
		self.assertEqual(_world_corner_xy(cfg, "TL"), (0.0, 2.8))
		self.assertEqual(_world_corner_xy(cfg, "TR"), (3.2, 2.8))


if __name__ == "__main__":
	unittest.main()
