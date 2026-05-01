"""双车通信模块。"""

import time
try:
	import ustruct as struct
except ImportError:
	import struct

from machine import Pin, UART

from Module.rm_binary_protocol import (
	KIND_COMMAND,
	KIND_HEARTBEAT,
	KIND_STATUS,
	code_to_state,
	pack_frame,
	state_to_code,
	unpack_frames,
)


def clamp(value, low, high):
	return max(low, min(high, value))


class CarLink:
	CMD_FMT = "<Bhhh"
	STATUS_FMT = "<BHBhhhhhh"
	HEARTBEAT_FMT = "<BH"

	def __init__(self, uart_id, baudrate, tx_pin, rx_pin, config=None):
		self.uart = UART(uart_id, baudrate=baudrate, tx=Pin(tx_pin), rx=Pin(rx_pin))
		self.cfg = config
		self.last_rx_ms = None
		self.last_packet = None
		self.last_remote_status = None
		self.last_remote_status_ms = None
		self.last_remote_command = None
		self.last_remote_command_ms = None
		self.last_remote_heartbeat = None
		self.last_remote_heartbeat_ms = None
		self.rx_buffer = b""
		self.rx_queue = []
		self.last_tx_ms = None

	def _write(self, data):
		self.last_tx_ms = time.ticks_ms()
		self.uart.write(data)

	def _read_into_buffer(self):
		reader = getattr(self.uart, "read", None)
		chunk = None
		if callable(reader):
			try:
				chunk = reader()
			except TypeError:
				chunk = reader(self.cfg.PROTO_MAX_PAYLOAD + 16) if self.cfg else reader(80)
		if chunk is None:
			chunk = self.uart.readline()

		if not chunk:
			return

		if isinstance(chunk, str):
			chunk = chunk.encode("latin1")
		self.rx_buffer += bytes(chunk)
		max_payload = getattr(self.cfg, "PROTO_MAX_PAYLOAD", 64)
		packets, self.rx_buffer = unpack_frames(self.rx_buffer, max_payload=max_payload)
		if packets:
			max_q = int(getattr(self.cfg, "CAR_LINK_MAX_QUEUE", 8))
			self.rx_queue.extend(packets[-max_q:])
			if len(self.rx_queue) > max_q:
				self.rx_queue = self.rx_queue[-max_q:]

	def _pop_packet(self, kind=None):
		self._read_into_buffer()
		if not self.rx_queue:
			return None
		if kind is None:
			return self.rx_queue.pop(0)

		for index, packet in enumerate(self.rx_queue):
			if packet["kind"] == kind:
				return self.rx_queue.pop(index)
		return None

	def send_kind(self, kind, seq=0, payload=b""):
		self._write(pack_frame(int(kind), int(seq), payload))

	def read_kind(self, kind):
		data = self._pop_packet(int(kind))
		if not data:
			return None
		self.last_rx_ms = time.ticks_ms()
		self.last_packet = data
		return data

	def send_command(self, vx, vy, vw, state="run", seq=0):
		payload = struct.pack(
			self.CMD_FMT,
			state_to_code(state),
			int(clamp(vx, -100, 100)),
			int(clamp(vy, -100, 100)),
			int(clamp(vw, -100, 100)),
		)
		self._write(pack_frame(KIND_COMMAND, seq, payload))

	def send_status(self, state="idle", seq=0, status=None, online=True):
		status = status or {}
		wheels = status.get("wheel_feedback", {})
		pose = status.get("pose", {})
		payload = struct.pack(
			self.STATUS_FMT,
			state_to_code(state),
			int(seq) & 0xFFFF,
			1 if online else 0,
			int(pose.get("x", 0.0) * 1000),
			int(pose.get("y", 0.0) * 1000),
			int(pose.get("yaw", 0.0) * 1000),
			int(wheels.get("fl", 0)),
			int(wheels.get("fr", 0)),
			int(wheels.get("bl", 0)),
			int(wheels.get("br", 0)),
		)
		self._write(pack_frame(KIND_STATUS, seq, payload))

	def send_heartbeat(self, seq=0, state="idle"):
		payload = struct.pack(self.HEARTBEAT_FMT, state_to_code(state), int(seq) & 0xFFFF)
		self._write(pack_frame(KIND_HEARTBEAT, seq, payload))

	def send_stop(self, seq=0):
		self.send_command(0, 0, 0, state="stop", seq=seq)

	def read_packet(self):
		packet = self._pop_packet()
		if not packet:
			return None

		self.last_rx_ms = time.ticks_ms()
		self.last_packet = packet
		return packet

	def read_command(self):
		data = self._pop_packet(KIND_COMMAND)
		if not data:
			return None

		state, vx, vy, vw = struct.unpack(self.CMD_FMT, data["payload"])
		self.last_rx_ms = time.ticks_ms()
		self.last_packet = data

		cmd = {
			"seq": int(data["seq"]),
			"state": code_to_state(state),
			"vx": int(vx),
			"vy": int(vy),
			"vw": int(vw),
			"ts": time.ticks_ms(),
		}
		self.last_remote_command = cmd
		self.last_remote_command_ms = self.last_rx_ms
		return cmd

	def read_status(self):
		data = self._pop_packet(KIND_STATUS)
		if not data:
			return None

		unpacked = struct.unpack(self.STATUS_FMT, data["payload"])
		state, ack_seq, online, px, py, yaw, fl, fr, bl, br = unpacked
		self.last_rx_ms = time.ticks_ms()
		self.last_packet = data

		status = {
			"seq": int(data["seq"]),
			"state": code_to_state(state),
			"online": bool(online),
			"ack": int(ack_seq),
			"status": {
				"pose": {"x": px / 1000.0, "y": py / 1000.0, "yaw": yaw / 1000.0},
				"wheel_feedback": {"fl": fl, "fr": fr, "bl": bl, "br": br},
			},
			"ts": time.ticks_ms(),
		}
		self.last_remote_status = status
		self.last_remote_status_ms = self.last_rx_ms
		return status

	def read_heartbeat(self):
		data = self._pop_packet(KIND_HEARTBEAT)
		if not data:
			return None

		state, ack_seq = struct.unpack(self.HEARTBEAT_FMT, data["payload"])
		self.last_rx_ms = time.ticks_ms()
		self.last_packet = data
		hb = {
			"seq": int(data["seq"]),
			"state": code_to_state(state),
			"ack": int(ack_seq),
			"ts": time.ticks_ms(),
		}
		self.last_remote_heartbeat = hb
		self.last_remote_heartbeat_ms = self.last_rx_ms
		return hb

	def is_timed_out(self, timeout_ms):
		if self.last_rx_ms is None:
			return False
		return time.ticks_diff(time.ticks_ms(), self.last_rx_ms) > timeout_ms
