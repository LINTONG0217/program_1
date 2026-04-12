"""UART loopback self-test for the main controller.

How to use:
- Pick a UART ID + TX/RX pins you want to verify.
- Physically short that TX and RX together (TX<->RX) AND connect GND.
- Run this script; if mapping is correct, it will read back what it writes.

This helps distinguish:
- main-board UART/pin-mux issues
vs
- OpenART wiring / wrong OpenART TX pin.
"""

import time
from machine import Pin, UART
from Module import config


def _try_fpioa_register_uart(uid, tx, rx):
	try:
		from fpioa_manager import fm  # type: ignore
	except Exception:
		return
	try:
		fpioa = getattr(fm, "fpioa", None)
		tx_func = getattr(fpioa, "UART{}_TX".format(uid), None)
		rx_func = getattr(fpioa, "UART{}_RX".format(uid), None)
		if tx_func is not None:
			try:
				fm.register(tx, tx_func, force=True)
			except Exception:
				pass
		if rx_func is not None:
			try:
				fm.register(rx, rx_func, force=True)
			except Exception:
				pass
	except Exception:
		return


def open_uart(uid, tx, rx, baud):
	_try_fpioa_register_uart(uid, tx, rx)
	tries = [
		lambda: UART(uid, baudrate=baud, tx=Pin(tx), rx=Pin(rx), timeout=50, read_buf_len=512),
		lambda: UART(uid, baudrate=baud, tx=tx, rx=rx, timeout=50, read_buf_len=512),
		lambda: UART(uid, baudrate=baud, tx=Pin(tx), rx=Pin(rx)),
		lambda: UART(uid, baudrate=baud, tx=tx, rx=rx),
		lambda: UART(uid, baudrate=baud),
		lambda: UART(uid, baud),
	]
	err = None
	for f in tries:
		try:
			return f(), None
		except Exception as e:
			err = e
	return None, err


def drain(u):
	read_fn = getattr(u, "read", None)
	if not callable(read_fn):
		return
	try:
		read_fn(1024)
	except Exception:
		pass


def main():
	# Default: verify the configured FAR vision UART (UART7 D22/D23)
	uid = getattr(config, "VISION2_UART_ID", 7)
	tx = getattr(config, "VISION2_TX_PIN", "D22")
	rx = getattr(config, "VISION2_RX_PIN", "D23")
	baud = getattr(config, "VISION2_BAUDRATE", 115200)

	print("loopback test uart={} baud={} tx={} rx={}".format(uid, baud, tx, rx))
	print("PLEASE short TX<->RX on the board, and connect GND.")

	u, err = open_uart(uid, tx, rx, baud)
	if u is None:
		print("open uart failed:", err)
		return

	drain(u)
	counter = 0
	last_ms = time.ticks_ms()
	while True:
		counter += 1
		msg = "PING {}\n".format(counter)
		try:
			u.write(msg.encode("utf-8"))
		except Exception as e:
			print("write failed:", e)

		# read back anything
		data = None
		readline_fn = getattr(u, "readline", None)
		read_fn = getattr(u, "read", None)
		if callable(readline_fn):
			try:
				data = readline_fn()
			except Exception:
				data = None
		if not data and callable(read_fn):
			try:
				data = read_fn(128)
			except Exception:
				data = None
		if data:
			try:
				print("RX:", data.decode("utf-8", "ignore"))
			except Exception:
				print("RX raw:", data)

		now = time.ticks_ms()
		if time.ticks_diff(now, last_ms) >= 1000:
			print("tick... sent", counter)
			last_ms = now
		time.sleep_ms(200)


if __name__ == "__main__":
	main()
