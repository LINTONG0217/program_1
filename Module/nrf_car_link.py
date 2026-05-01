"""NRF24L01 transport for dual-car link.

This module reuses the existing rm_binary_protocol frame format, but sends each
frame as a single NRF payload (<= 32 bytes).

To keep this repo library-agnostic, NRFCarLink operates on a duck-typed `radio`
object (e.g. MicroPython nrf24l01.NRF24L01 instance).
"""

import time

try:
	import ustruct as struct
except ImportError:
	import struct

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


class NRFCarLink:
	CMD_FMT = "<Bhhh"
	STATUS_FMT = "<BHBhhhhhh"
	HEARTBEAT_FMT = "<BH"

	def __init__(self, radio, config=None):
		self.radio = radio
		self.cfg = config
		self.last_rx_ms = None
		self.last_packet = None
		self.last_remote_status = None
		self.last_remote_status_ms = None
		self.last_remote_command = None
		self.last_remote_command_ms = None
		self.last_remote_heartbeat = None
		self.last_remote_heartbeat_ms = None
		self.rx_queue = []
		self.last_tx_ms = None

	def _packet_size(self):
		return int(getattr(self.cfg, "CAR_LINK_NRF_PAYLOAD_SIZE", 32) if self.cfg else 32)

	def _max_proto_payload(self):
		# rm_binary_protocol.max_payload is the payload length (excluding header+crc).
		# NRF payload is limited, but current CMD/STATUS/HEARTBEAT all fit within 22.
		return int(getattr(self.cfg, "CAR_LINK_PROTO_MAX_PAYLOAD", 22) if self.cfg else 22)

	def _radio_any(self):
		any_fn = getattr(self.radio, "any", None)
		if callable(any_fn):
			try:
				return int(any_fn())
			except Exception:
				try:
					return 1 if any_fn() else 0
				except Exception:
					return 0
		return 0

	def _radio_recv(self):
		recv_fn = getattr(self.radio, "recv", None)
		if callable(recv_fn):
			return recv_fn()
		# Some libs use `read()`
		read_fn = getattr(self.radio, "read", None)
		if callable(read_fn):
			return read_fn()
		return None

	def _radio_send(self, data):
		send_fn = getattr(self.radio, "send", None)
		if callable(send_fn):
			return send_fn(data)
		write_fn = getattr(self.radio, "write", None)
		if callable(write_fn):
			return write_fn(data)
		raise AttributeError("radio object has no send/write")

	def _enter_tx(self):
		stop_listen = getattr(self.radio, "stop_listening", None)
		if callable(stop_listen):
			stop_listen()

	def _enter_rx(self):
		start_listen = getattr(self.radio, "start_listening", None)
		if callable(start_listen):
			start_listen()

	def _write(self, data):
		data = bytes(data or b"")
		packet_size = self._packet_size()
		if len(data) > packet_size:
			raise ValueError("NRF payload overflow: %d > %d" % (len(data), packet_size))

		self.last_tx_ms = time.ticks_ms()
		self._enter_tx()
		self._radio_send(data)
		self._enter_rx()

	def _read_into_queue(self):
		while self._radio_any():
			raw = self._radio_recv()
			if not raw:
				break
			if isinstance(raw, str):
				raw = raw.encode("latin1")

			packets, _ = unpack_frames(bytes(raw), max_payload=self._max_proto_payload())
			if packets:
				max_q = int(getattr(self.cfg, "CAR_LINK_MAX_QUEUE", 8) if self.cfg else 8)
				self.rx_queue.extend(packets[-max_q:])
				if len(self.rx_queue) > max_q:
					self.rx_queue = self.rx_queue[-max_q:]

	def _pop_packet(self, kind=None):
		self._read_into_queue()
		if not self.rx_queue:
			return None
		if kind is None:
			return self.rx_queue.pop(0)

		for index, packet in enumerate(self.rx_queue):
			if packet.get("kind") == kind:
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


def build_nrf24_radio(cfg):
	"""Best-effort helper to construct a radio if `nrf24l01` driver exists.

	This is intentionally conservative; if it can't build the radio, it raises.
	"""
	try:
		from machine import Pin, SPI
	except Exception as e:
		raise ImportError("machine.SPI/Pin unavailable: %s" % e)

	try:
		nrf24l01 = __import__("nrf24l01")
		NRF24L01 = getattr(nrf24l01, "NRF24L01")
	except Exception as e:
		raise ImportError("nrf24l01 driver not found: %s" % e)

	spi_id = int(getattr(cfg, "CAR_LINK_NRF_SPI_ID", 1))
	baudrate = int(getattr(cfg, "CAR_LINK_NRF_SPI_BAUDRATE", 4_000_000))

	sck = getattr(cfg, "CAR_LINK_NRF_SCK_PIN", None)
	mosi = getattr(cfg, "CAR_LINK_NRF_MOSI_PIN", None)
	miso = getattr(cfg, "CAR_LINK_NRF_MISO_PIN", None)
	if sck is not None and mosi is not None and miso is not None:
		spi = SPI(
			spi_id,
			baudrate=baudrate,
			sck=Pin(int(sck)),
			mosi=Pin(int(mosi)),
			miso=Pin(int(miso)),
		)
	else:
		spi = SPI(spi_id, baudrate=baudrate)

	ce_pin = getattr(cfg, "CAR_LINK_NRF_CE_PIN", None)
	csn_pin = getattr(cfg, "CAR_LINK_NRF_CSN_PIN", None)
	if ce_pin is None or csn_pin is None:
		raise ValueError("CAR_LINK_NRF_CE_PIN/CAR_LINK_NRF_CSN_PIN must be set")

	payload_size = int(getattr(cfg, "CAR_LINK_NRF_PAYLOAD_SIZE", 32))
	radio = NRF24L01(spi, Pin(int(csn_pin)), Pin(int(ce_pin)), payload_size=payload_size)

	channel = int(getattr(cfg, "CAR_LINK_NRF_CHANNEL", 76))
	set_channel = getattr(radio, "set_channel", None)
	if callable(set_channel):
		set_channel(channel)

	set_retries = getattr(radio, "set_retries", None)
	if callable(set_retries):
		delay_us = int(getattr(cfg, "CAR_LINK_NRF_RETRY_DELAY_US", 4000))
		count = int(getattr(cfg, "CAR_LINK_NRF_RETRY_COUNT", 10))
		try:
			set_retries(delay_us, count)
		except TypeError:
			set_retries(count, delay_us)

	tx = bytes(getattr(cfg, "CAR_LINK_NRF_PIPE_TX"))
	rx = bytes(getattr(cfg, "CAR_LINK_NRF_PIPE_RX"))
	radio.open_tx_pipe(tx)
	radio.open_rx_pipe(1, rx)
	radio.start_listening()
	return radio
