"""Manual encoder count test for the main controller.

Run from PC, with the board connected:

    python -m mpremote run tests/hardware/manual_encoder_counts.py

Keep wheels off the ground for the first run. Press C14 to start, then rotate
each wheel exactly one turn by hand. The script prints net signed pulses for
direction checks and absolute pulses for comparing one-turn pulse counts.
"""

import time

from machine import Pin

from Module import config

try:
	from BSP.board_runtime import MainControl
except Exception as e:
	MainControl = None
	MAINCONTROL_IMPORT_ERROR = e


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
	if MainControl is None:
		print("MainControl import failed:", repr(MAINCONTROL_IMPORT_ERROR))
		print("Please upload/sync BSP/board_runtime.py and BSP/device_adapters.py to the board first.")
		return
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

	data = snapshot(board)
	pulse_totals = {}
	abs_pulse_totals = {}
	for name in NAMES:
		pulse_totals[name] = 0
		abs_pulse_totals[name] = 0
	last_print = time.ticks_ms()
	cpr = float(getattr(config, "ENCODER_CPR", 0) or 0)
	print("net changes with direction; abs only increases; compare abs after one turn; press C14 to reset")

	try:
		while True:
			now = time.ticks_ms()
			if key.value() == 0:
				data = snapshot(board)
				for name in NAMES:
					pulse_totals[name] = 0
					abs_pulse_totals[name] = 0
				print("encoder pulse totals reset")
				time.sleep_ms(300)
				while key.value() == 0:
					time.sleep_ms(20)

			data = snapshot(board)
			for name in NAMES:
				count = int(data[name]["count"])
				pulse_totals[name] += count
				abs_pulse_totals[name] += abs(count)

			if time.ticks_diff(now, last_print) >= 200:
				last_print = now
				parts = []
				for name in NAMES:
					count = data[name]["count"]
					speed = data[name]["speed"]
					total = pulse_totals[name]
					abs_total = abs_pulse_totals[name]
					if cpr > 0:
						parts.append("{}:cnt={} net={} abs={} rev={:.2f} spd={:.1f}".format(name, count, total, abs_total, abs_total / cpr, speed))
					else:
						parts.append("{}:cnt={} net={} abs={} spd={:.1f}".format(name, count, total, abs_total, speed))
				print(" | ".join(parts))
			time.sleep_ms(20)
	except KeyboardInterrupt:
		print("encoder test stopped")


if __name__ == "__main__":
	main()
