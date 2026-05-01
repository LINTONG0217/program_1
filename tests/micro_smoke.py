"""MicroPython-friendly smoke checks.

This file intentionally does not import unittest. Run it only when you want a
quick board-side check:

    python -m mpremote run tests/micro_smoke.py

The normal tests/test_*.py files are CPython-only and should be run on the PC.
"""

try:
	import os
	import sys

	_here = os.path.dirname(os.path.abspath(__file__))
	_root = os.path.dirname(_here)
	if _root not in sys.path:
		sys.path.insert(0, _root)
except Exception:
	pass

from Module import config
from Module.wheel_odometry import AdvancedOdometry


def check(name, condition):
	if not condition:
		raise AssertionError(name)
	print("OK", name)


def almost_equal(a, b, eps=1e-6):
	return abs(float(a) - float(b)) <= eps


def main():
	check("field x", almost_equal(config.FIELD_SIZE_X_M, 3.2))
	check("field y", almost_equal(config.FIELD_SIZE_Y_M, 2.8))
	check("center x", almost_equal(config.CENTER_TARGET_X, 1.6))
	check("center y", almost_equal(config.CENTER_TARGET_Y, 1.4))

	odom = AdvancedOdometry()
	odom.reset(0.0)
	odom.update([1.0, 1.0, 1.0, 1.0], 0.0, dt=1.0)
	x, y, _ = odom.get_position()
	check("odom forward x", almost_equal(x, 1.0))
	check("odom forward y", almost_equal(y, 0.0))
	check("odom distance", almost_equal(odom.get_distance(), 1.0))

	odom.reset(0.0)
	odom.update([1.0, -1.0, -1.0, 1.0], 0.0, dt=1.0)
	x, y, _ = odom.get_position()
	check("odom strafe x", almost_equal(x, 0.0))
	check("odom strafe y", almost_equal(y, 1.0))

	print("MICRO_SMOKE_OK")


if __name__ == "__main__":
	main()
