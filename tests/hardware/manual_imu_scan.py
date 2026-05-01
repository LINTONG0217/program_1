"""Scan available IMU classes on the main controller.

Run:
    python -m mpremote run tests/hardware/manual_imu_scan.py

If it finds a working IMU class, copy that class name to Module/config.py:
    IMU_CLASS_NAME = "IMU963RA"
"""

import time

from machine import Pin

try:
	from Module import config
except Exception:
	config = None


MODULES = ("seekfree", "imu", "smartcar")
CANDIDATES = (
	"IMU963RX",
	"IMU963RA",
	"IMU963RB",
	"IMU660RA",
	"IMU660RB",
	"ICM20602",
	"IMU",
)


def get_pin_args():
	pins = getattr(config, "IMU_PINS", ("C10", "C12", "C13")) if config else ("C10", "C12", "C13")
	if not isinstance(pins, (tuple, list)):
		pins = (pins,)
	out = [()]
	out.append(tuple(pins))
	try:
		out.append(tuple(Pin(p) for p in pins))
	except Exception as e:
		print("Pin(...) build error:", repr(e))
	return out


def try_read(dev):
	reader = getattr(dev, "read", None)
	if callable(reader):
		try:
			reader()
		except Exception as e:
			print("  read() error:", repr(e))

	getter = getattr(dev, "get", None)
	if callable(getter):
		try:
			return getter()
		except Exception as e:
			print("  get() error:", repr(e))

	for name in ("gyro", "get_gyro", "read_gyro", "mag", "get_mag", "read_mag"):
		fn = getattr(dev, name, None)
		if callable(fn):
			try:
				print(" ", name, "=>", fn())
			except Exception as e:
				print(" ", name, "error:", repr(e))
	return None


def main():
	print("IMU scan start")
	for module_name in MODULES:
		try:
			mod = __import__(module_name)
			print("module ok:", module_name)
		except Exception as e:
			print("module missing:", module_name, repr(e))
			continue

		try:
			names = dir(mod)
			print("module names:", names)
		except Exception:
			names = ()

		for class_name in CANDIDATES:
			cls = getattr(mod, class_name, None)
			if cls is None:
				continue
			print("candidate:", module_name + "." + class_name)
			dev = None
			for args in get_pin_args():
				try:
					dev = cls(*args)
					print("  init ok args=", args)
					break
				except Exception as e:
					print("  init error args=", args, repr(e))
			if dev is None:
				continue

			for _ in range(3):
				raw = try_read(dev)
				print("  sample:", raw)
				time.sleep_ms(100)
	print("IMU scan done")


if __name__ == "__main__":
	main()
