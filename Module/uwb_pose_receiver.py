"""UWB(DW3000) pose receiver.

This is a small UART line receiver that parses either:
- CSV:  "x,y,yaw\n" (meters, meters, degrees)
- JSON: '{"x":0.12,"y":0.34,"yaw":90.0}\n'

It intentionally avoids depending on a specific DW3000 vendor protocol.
"""

import time

try:
	import ujson as json
except ImportError:
	import json

from machine import Pin, UART


class UWBPoseReceiver:
	def __init__(self, uart_id, baudrate, tx_pin=None, rx_pin=None, fmt="csv"):
		self.fmt = str(fmt or "csv").strip().lower()
		if tx_pin is None and rx_pin is None:
			self.uart = UART(uart_id, baudrate=baudrate)
		else:
			self.uart = UART(uart_id, baudrate=baudrate, tx=Pin(int(tx_pin)), rx=Pin(int(rx_pin)))

		self.last_pose = None
		self.last_pose_ms = None

	def _parse_csv(self, line):
		# Allow spaces, optional labels like "x=..".
		parts = [p.strip() for p in line.replace(" ", "").split(",") if p.strip()]
		if len(parts) < 3:
			return None
		try:
			x = float(parts[0].split("=")[-1])
			y = float(parts[1].split("=")[-1])
			yaw = float(parts[2].split("=")[-1])
		except Exception:
			return None
		return {"x": x, "y": y, "yaw": yaw}

	def _parse_json(self, line):
		try:
			obj = json.loads(line)
		except Exception:
			return None
		if not isinstance(obj, dict):
			return None
		try:
			x = float(obj.get("x"))
			y = float(obj.get("y"))
			yaw = float(obj.get("yaw"))
		except Exception:
			return None
		return {"x": x, "y": y, "yaw": yaw}

	def read_pose(self):
		"""Read at most one line and return the most recent pose.

		Returns a dict with keys x,y,yaw,ts when a new pose is parsed.
		"""
		line = self.uart.readline()
		if not line:
			return None

		if isinstance(line, bytes):
			try:
				line = line.decode("utf-8")
			except Exception:
				line = line.decode("latin1")
		line = (line or "").strip()
		if not line:
			return None

		pose = None
		if self.fmt == "json":
			pose = self._parse_json(line)
		else:
			pose = self._parse_csv(line)

		if not pose:
			return None

		now = time.ticks_ms()
		pose["ts"] = now
		self.last_pose = pose
		self.last_pose_ms = now
		return pose

	def get_pose(self):
		"""Return last cached pose (may be stale)."""
		return self.last_pose

	def age_ms(self):
		if self.last_pose_ms is None:
			return None
		return time.ticks_diff(time.ticks_ms(), self.last_pose_ms)
