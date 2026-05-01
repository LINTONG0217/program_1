"""Wireless file update protocol over rm_binary_protocol frames.

Constraints:
- NRF24 payload is 32 bytes.
- rm_binary_protocol frame overhead is 10 bytes.
- So protocol payload must be <= 22 bytes to fit in a single NRF packet.

This module implements a small stop-and-wait file transfer with ACK/NAK.
"""

import time

try:
	import ubinascii as binascii
except ImportError:
	import binascii

try:
	import ustruct as struct
except ImportError:
	import struct

try:
	import uos as os
except ImportError:
	import os

from Module.rm_binary_protocol import (
	KIND_FILE_ACK,
	KIND_FILE_BEGIN,
	KIND_FILE_DATA,
	KIND_FILE_END,
	KIND_FILE_NAK,
	KIND_FILE_PATH,
)


STAGE_PATH = 0
STAGE_DATA = 1
STAGE_DONE = 2

_ACK_FMT = "<BBI"  # txid, stage, value
_NAK_FMT = "<BBI"  # txid, stage, expected_value
_BEGIN_HDR_FMT = "<BIIB"  # txid, size, crc32, path_len
_PATH_HDR_FMT = "<BH"  # txid, path_offset
_DATA_HDR_FMT = "<BI"  # txid, data_offset
_END_FMT = "<BI"  # txid, crc32


def _crc32_update(crc, data):
	try:
		crc = binascii.crc32(data, crc)
	except TypeError:
		# Some ports only accept one argument
		crc = binascii.crc32((struct.pack("<I", crc & 0xFFFFFFFF) + data))
	return crc & 0xFFFFFFFF


def _get_flash_root():
	try:
		os.listdir("/flash")
		return "/flash"
	except Exception:
		return "."


def _mkdir_p(path):
	if not path:
		return
	parts = []
	for part in path.replace("\\", "/").split("/"):
		if not part:
			continue
		parts.append(part)
		current = "/".join(parts)
		try:
			os.mkdir(current)
		except Exception:
			pass


def _dirname(path):
	path = path.replace("\\", "/")
	if "/" not in path:
		return ""
	return path.rsplit("/", 1)[0]


def _is_safe_relpath(relpath):
	relpath = str(relpath or "").replace("\\", "/")	
	if not relpath or relpath.startswith("/"):
		return False
	if ":" in relpath:
		return False
	for part in relpath.split("/"):
		if part in ("", ".", ".."):
			return False
	return True


class FileUpdateReceiver:
	def __init__(self, link, config=None, verbose=True):
		self.link = link
		self.cfg = config
		self.verbose = bool(verbose)
		self.reset()

	def reset(self):
		self.active = False
		self.txid = None
		self.file_size = 0
		self.file_crc32 = 0
		self.path_len = 0
		self.path_bytes = b""
		self.dest_rel = None
		self.tmp_path = None
		self._fh = None
		self.data_offset = 0
		self._crc_running = 0

	def _log(self, *args):
		if self.verbose:
			print("[update]", *args)

	def _send_ack(self, txid, stage, value, seq=0):
		payload = struct.pack(_ACK_FMT, int(txid) & 0xFF, int(stage) & 0xFF, int(value) & 0xFFFFFFFF)
		self.link.send_kind(KIND_FILE_ACK, seq=seq, payload=payload)

	def _send_nak(self, txid, stage, expected, seq=0):
		payload = struct.pack(_NAK_FMT, int(txid) & 0xFF, int(stage) & 0xFF, int(expected) & 0xFFFFFFFF)
		self.link.send_kind(KIND_FILE_NAK, seq=seq, payload=payload)

	def _close_fh(self):
		try:
			if self._fh:
				self._fh.close()
		except Exception:
			pass
		self._fh = None

	def _cleanup_tmp(self):
		self._close_fh()
		if self.tmp_path:
			try:
				os.remove(self.tmp_path)
			except Exception:
				pass
		self.tmp_path = None

	def _apply_file(self):
		root = _get_flash_root()
		dest_abs = (root.rstrip("/") + "/" + self.dest_rel).replace("//", "/")
		parent = _dirname(dest_abs)
		if parent:
			_mkdir_p(parent)

		bak = dest_abs + ".bak"
		try:
			os.remove(bak)
		except Exception:
			pass

		try:
			os.rename(dest_abs, bak)
		except Exception:
			pass

		os.rename(self.tmp_path, dest_abs)
		try:
			os.remove(bak)
		except Exception:
			pass

	def poll(self, max_packets=8):
		"""Process incoming update packets.

		Returns:
		- None: no update activity
		- dict: event info {"event": "done"|"progress"|"error", ...}
		"""
		events = []
		for _ in range(int(max_packets)):
			pkt = (
				self.link.read_kind(KIND_FILE_BEGIN)
				or self.link.read_kind(KIND_FILE_PATH)
				or self.link.read_kind(KIND_FILE_DATA)
				or self.link.read_kind(KIND_FILE_END)
			)
			if not pkt:
				break

			kind = int(pkt.get("kind"))
			seq = int(pkt.get("seq", 0))
			payload = pkt.get("payload") or b""

			if kind == KIND_FILE_BEGIN:
				events.append(self._on_begin(payload, seq))
			elif kind == KIND_FILE_PATH:
				events.append(self._on_path(payload, seq))
			elif kind == KIND_FILE_DATA:
				events.append(self._on_data(payload, seq))
			elif kind == KIND_FILE_END:
				events.append(self._on_end(payload, seq))

		# Return the last meaningful event
		for evt in reversed(events):
			if evt:
				return evt
		return None

	def _on_begin(self, payload, seq):
		hdr_size = struct.calcsize(_BEGIN_HDR_FMT)
		if len(payload) < hdr_size:
			return {"event": "error", "reason": "begin_short"}

		txid, size, crc32, path_len = struct.unpack(_BEGIN_HDR_FMT, payload[:hdr_size])
		path_chunk = payload[hdr_size:]

		self._cleanup_tmp()
		self.reset()
		self.active = True
		self.txid = int(txid)
		self.file_size = int(size)
		self.file_crc32 = int(crc32) & 0xFFFFFFFF
		self.path_len = int(path_len)
		self.path_bytes = bytes(path_chunk[: self.path_len])
		self.data_offset = 0
		self._crc_running = 0

		self._log("begin", "txid=", self.txid, "size=", self.file_size, "path_len=", self.path_len)
		next_path = len(self.path_bytes)
		self._send_ack(self.txid, STAGE_PATH, next_path, seq=seq)
		return {"event": "progress", "stage": "begin", "txid": self.txid}

	def _on_path(self, payload, seq):
		hdr_size = struct.calcsize(_PATH_HDR_FMT)
		if len(payload) < hdr_size:
			return {"event": "error", "reason": "path_short"}

		txid, offset = struct.unpack(_PATH_HDR_FMT, payload[:hdr_size])
		txid = int(txid)
		offset = int(offset)
		chunk = payload[hdr_size:]

		if not self.active or txid != self.txid:
			self._send_nak(txid, STAGE_PATH, 0, seq=seq)
			return {"event": "error", "reason": "path_wrong_txid"}

		if offset != len(self.path_bytes):
			self._send_nak(self.txid, STAGE_PATH, len(self.path_bytes), seq=seq)
			return None

		need = max(0, self.path_len - len(self.path_bytes))
		self.path_bytes += bytes(chunk[:need])
		next_path = len(self.path_bytes)
		self._send_ack(self.txid, STAGE_PATH, next_path, seq=seq)

		if len(self.path_bytes) < self.path_len:
			return None

		try:
			path = self.path_bytes.decode("utf-8")
		except Exception:
			path = self.path_bytes.decode("latin1")
		path = path.replace("\\", "/")
		if not _is_safe_relpath(path):
			self._log("reject path:", path)
			self._send_nak(self.txid, STAGE_PATH, next_path, seq=seq)
			self.reset()
			return {"event": "error", "reason": "unsafe_path"}

		self.dest_rel = path
		root = _get_flash_root()
		_mkdir_p((root.rstrip("/") + "/.update_tmp").replace("//", "/"))
		self.tmp_path = (root.rstrip("/") + "/.update_tmp/" + path.replace("/", "_") + "." + str(self.txid) + ".tmp").replace("//", "/")

		self._fh = open(self.tmp_path, "wb")
		self._log("path ok:", self.dest_rel)
		self._send_ack(self.txid, STAGE_DATA, 0, seq=seq)
		return {"event": "progress", "stage": "path", "path": self.dest_rel}

	def _on_data(self, payload, seq):
		hdr_size = struct.calcsize(_DATA_HDR_FMT)
		if len(payload) < hdr_size:
			return {"event": "error", "reason": "data_short"}

		txid, offset = struct.unpack(_DATA_HDR_FMT, payload[:hdr_size])
		txid = int(txid)
		offset = int(offset)
		chunk = payload[hdr_size:]

		if not self.active or txid != self.txid or not self._fh:
			self._send_nak(txid, STAGE_DATA, 0, seq=seq)
			return None

		if offset != self.data_offset:
			self._send_nak(self.txid, STAGE_DATA, self.data_offset, seq=seq)
			return None

		if not chunk:
			self._send_ack(self.txid, STAGE_DATA, self.data_offset, seq=seq)
			return None

		remaining = max(0, self.file_size - self.data_offset)
		chunk = chunk[:remaining]
		try:
			self._fh.write(chunk)
		except Exception as e:
			self._log("write failed:", e)
			self._cleanup_tmp()
			self.reset()
			return {"event": "error", "reason": "write_failed"}

		self.data_offset += len(chunk)
		self._crc_running = _crc32_update(self._crc_running, chunk)
		self._send_ack(self.txid, STAGE_DATA, self.data_offset, seq=seq)

		return None

	def _on_end(self, payload, seq):
		hdr_size = struct.calcsize(_END_FMT)
		if len(payload) < hdr_size:
			return {"event": "error", "reason": "end_short"}

		txid, crc32 = struct.unpack(_END_FMT, payload[:hdr_size])
		txid = int(txid)
		crc32 = int(crc32) & 0xFFFFFFFF

		if not self.active or txid != self.txid:
			self._send_nak(txid, STAGE_DONE, 0, seq=seq)
			return None

		self._close_fh()

		if self.data_offset != self.file_size:
			self._log("size mismatch", self.data_offset, "!=", self.file_size)
			self._cleanup_tmp()
			self.reset()
			self._send_nak(txid, STAGE_DONE, self.data_offset, seq=seq)
			return {"event": "error", "reason": "size_mismatch"}

		expected = self.file_crc32
		got = self._crc_running & 0xFFFFFFFF
		if crc32 != expected or got != expected:
			self._log("crc mismatch", "expected=", expected, "got=", got, "end=", crc32)
			self._cleanup_tmp()
			self.reset()
			self._send_nak(txid, STAGE_DONE, 0, seq=seq)
			return {"event": "error", "reason": "crc_mismatch"}

		try:
			self._apply_file()
		except Exception as e:
			self._log("apply failed:", e)
			self._cleanup_tmp()
			self.reset()
			self._send_nak(txid, STAGE_DONE, 0, seq=seq)
			return {"event": "error", "reason": "apply_failed"}

		self._log("done:", self.dest_rel)
		self._send_ack(txid, STAGE_DONE, 0, seq=seq)
		path = self.dest_rel
		self.reset()
		return {"event": "done", "path": path}


class FileUpdateSender:
	def __init__(self, link, config=None, verbose=True):
		self.link = link
		self.cfg = config
		self.verbose = bool(verbose)
		self.seq = 0
		self.txid = 1

	def _log(self, *args):
		if self.verbose:
			print("[update]", *args)

	def _next_txid(self):
		self.txid = (int(self.txid) + 1) & 0xFF
		if self.txid == 0:
			self.txid = 1
		return self.txid

	def _wait_ack(self, want_txid, want_stage, want_value=None, timeout_ms=800):
		start = time.ticks_ms()
		while time.ticks_diff(time.ticks_ms(), start) <= int(timeout_ms):
			ack = self.link.read_kind(KIND_FILE_ACK)
			if ack:
				payload = ack.get("payload") or b""
				if len(payload) >= struct.calcsize(_ACK_FMT):
					txid, stage, value = struct.unpack(_ACK_FMT, payload[: struct.calcsize(_ACK_FMT)])
					txid = int(txid)
					stage = int(stage)
					value = int(value)
					if txid == int(want_txid) and stage == int(want_stage):
						if want_value is None or int(want_value) == value:
							return True, value
			nak = self.link.read_kind(KIND_FILE_NAK)
			if nak:
				payload = nak.get("payload") or b""
				if len(payload) >= struct.calcsize(_NAK_FMT):
					txid, stage, expected = struct.unpack(_NAK_FMT, payload[: struct.calcsize(_NAK_FMT)])
					if int(txid) == int(want_txid) and int(stage) == int(want_stage):
						return False, int(expected)
			if hasattr(time, "sleep_ms"):
				time.sleep_ms(1)
		return False, None

	def _send_with_retry(self, kind, payload, want_txid, want_stage, want_value=None, retries=10):
		for attempt in range(int(retries)):
			self.link.send_kind(kind, seq=self.seq, payload=payload)
			ok, value = self._wait_ack(want_txid, want_stage, want_value=want_value)
			self.seq = (self.seq + 1) & 0xFFFF
			if ok:
				return True, value
			# If receiver says "expected offset", caller will resend with new offset
			self._log("retry", attempt + 1, "kind", kind, "expect", value)
		return False, None

	def send_file(self, relpath, data_bytes):
		if not _is_safe_relpath(relpath):
			raise ValueError("unsafe path: %r" % relpath)
		path_bytes = relpath.encode("utf-8")
		path_len = len(path_bytes)
		if path_len > 250:
			raise ValueError("path too long")

		size = len(data_bytes)
		crc = 0
		crc = _crc32_update(crc, data_bytes)

		txid = self._next_txid()
		hdr = struct.pack(_BEGIN_HDR_FMT, txid, int(size) & 0xFFFFFFFF, int(crc) & 0xFFFFFFFF, path_len)
		first_path_max = 22 - len(hdr)
		first_path = path_bytes[:first_path_max]
		payload_begin = hdr + first_path

		ok, next_path = self._send_with_retry(KIND_FILE_BEGIN, payload_begin, txid, STAGE_PATH)
		if not ok:
			raise OSError("begin not ack")

		offset = int(next_path or 0)
		while offset < path_len:
			chunk = path_bytes[offset: offset + (22 - struct.calcsize(_PATH_HDR_FMT))]
			payload = struct.pack(_PATH_HDR_FMT, txid, offset) + chunk
			ok, next_path = self._send_with_retry(KIND_FILE_PATH, payload, txid, STAGE_PATH)
			if not ok and next_path is not None:
				offset = int(next_path)
				continue
			if not ok:
				raise OSError("path not ack")
			offset = int(next_path)

		# Receiver sends STAGE_DATA ack with value=0 when ready
		ok, _ = self._wait_ack(txid, STAGE_DATA, want_value=0)
		if not ok:
			raise OSError("no data-ready ack")

		offset = 0
		chunk_max = 22 - struct.calcsize(_DATA_HDR_FMT)
		while offset < size:
			chunk = data_bytes[offset: offset + chunk_max]
			payload = struct.pack(_DATA_HDR_FMT, txid, offset) + chunk
			ok, next_off = self._send_with_retry(KIND_FILE_DATA, payload, txid, STAGE_DATA)
			if not ok and next_off is not None:
				offset = int(next_off)
				continue
			if not ok:
				raise OSError("data not ack")
			offset = int(next_off)

		payload_end = struct.pack(_END_FMT, txid, int(crc) & 0xFFFFFFFF)
		ok, _ = self._send_with_retry(KIND_FILE_END, payload_end, txid, STAGE_DONE, want_value=0)
		if not ok:
			raise OSError("end not ack")
		return True

	def send_file_from_fs(self, relpath, abs_path, read_chunk=512):
		# Read file into memory in a bounded way (still ends up in RAM for crc calculation).
		# For typical MicroPython project sizes this is ok; if you need huge files, we can stream+crc.
		with open(abs_path, "rb") as f:
			data = f.read()
		return self.send_file(relpath, data)


def walk_files(root, base_rel=""):
	root = root.rstrip("/")
	try:
		entries = os.listdir(root)
	except Exception:
		return

	for name in entries:
		if name in (".update_tmp",):
			continue
		abs_path = root + "/" + name
		rel = (base_rel + "/" + name).lstrip("/") if base_rel else name
		try:
			st = os.stat(abs_path)
		except Exception:
			continue

		# MicroPython: stat[0] bitmask, dirs often have 0x4000 set.
		is_dir = False
		try:
			is_dir = bool(st[0] & 0x4000)
		except Exception:
			is_dir = False

		if is_dir:
			for item in walk_files(abs_path, rel):
				yield item
		else:
			yield rel.replace("\\", "/"), abs_path
