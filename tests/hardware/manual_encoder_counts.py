"""Manual encoder count test for the main controller.

Run from PC, with the board connected:

    python -m mpremote run tests/hardware/manual_encoder_counts.py

Keep wheels off the ground for the first run. Press C14 to start, then rotate
wheels by hand or command a small motor test separately. The script prints raw
count deltas and speed for each encoder.
"""

import time

from machine import Pin

from BSP.board_runtime import MainControl
from Module import config


NAMES = ("fl", "fr", "bl", "br")


def wait_c14_start():
	if not bool(getattr(config, "TEST_WAIT_C14_ENABLE", True)):
		return
	pin_name = getattr(config, "NAV_KEY_PIN", "C14")
	key = Pin(pin_name, Pin.IN)
	pull_up = getattr(Pin, "PULL_UP_47K", None)
	if pull_up is not None:
		try:
			key = Pin(pin_name, Pin.IN, pull_up)
		except Exception:
			pass

	print("press {} to start encoder test".format(pin_name))
	while key.value() != 0:
		time.sleep_ms(20)
	time.sleep_ms(60)
	while key.value() == 0:
		time.sleep_ms(20)
	print("{} start".format(pin_name))


def make_key():
	pin_name = getattr(config, "NAV_KEY_PIN", "C14")
	key = Pin(pin_name, Pin.IN)
	pull_up = getattr(Pin, "PULL_UP_47K", None)
	if pull_up is not None:
		try:
			key = Pin(pin_name, Pin.IN, pull_up)
		except Exception:
			pass
	return key


def snapshot(board):
	return board.read_encoder_snapshot()


def main():
	board = MainControl(config)
	wait_c14_start()
	key = make_key()

	print("encoder pins:")
	for name in NAMES:
		enc = board.encoders[name]
		print(
			name,
			"pins=", enc.pin_a, enc.pin_b,
			"reverse=", enc.reverse,
			"available=", enc.available,
		)

	base = snapshot(board)
	last_print = time.ticks_ms()
	print("rotate each wheel; press C14 again to reset zero")

	try:
		while True:
			now = time.ticks_ms()
			if key.value() == 0:
				base = snapshot(board)
				print("encoder zero reset")
				time.sleep_ms(300)
				while key.value() == 0:
					time.sleep_ms(20)

			if time.ticks_diff(now, last_print) >= 200:
				last_print = now
				data = snapshot(board)
				parts = []
				for name in NAMES:
					count = data[name]["count"]
					speed = data[name]["speed"]
					delta = count - base[name]["count"]
					parts.append("{}:cnt={} d={} spd={:.1f}".format(name, count, delta, speed))
				print(" | ".join(parts))
			time.sleep_ms(20)
	except KeyboardInterrupt:
		print("encoder test stopped")


if __name__ == "__main__":
	main()
