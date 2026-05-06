"""串口视觉接收模块。"""

try:
	import ujson as json
except ImportError:
	import json

import time
from machine import Pin, UART


def _crc16_ibm(data_bytes, init=0xFFFF):
	"""CRC-16/IBM(ARC), init=0xFFFF, poly=0xA001.

	与 OpenART 端 vision.py 的实现保持一致，用于 UART 行协议校验：
	SP|<json>|<crc16_hex>\n
	注意：这是 bitwise 实现（不需要 256 表），速度对短 JSON 足够。
	"""
	crc = int(init) & 0xFFFF
	for b in data_bytes:
		crc ^= int(b) & 0xFF
		for _ in range(8):
			if crc & 1:
				crc = (crc >> 1) ^ 0xA001
			else:
				crc >>= 1
			crc &= 0xFFFF
	return crc


def _parse_sp_line(line):
	"""Parse one UART line.

	Supported formats:
	- New (preferred):  SP|<json>|<crc16_hex>
	- Legacy:           <json>

	Return (parsed_json_or_None, status_str).

	Note:
	- The vision sender is expected to send a JSON object (dict).
	- In practice, UART noise can produce lines that still parse as valid JSON
	  but with the wrong type (e.g. an integer). Callers should validate type.
	"""
	if not line:
		return None, "empty"
	line = line.strip()
	if not line:
		return None, "empty"

	# 容忍串口噪声：有时一行前面会混入其它字符，但包含有效的 "SP|..."。
	# 只在行首不是 SP| 时才尝试对齐到首次出现的位置。
	if not line.startswith("SP|"):
		try:
			idx = line.find("SP|")
			if idx > 0:
				line = line[idx:]
		except Exception:
			pass

	# 新协议：SP|<json>|<crc>
	if line.startswith("SP|"):
		try:
			parts = line.split("|", 2)
			if len(parts) != 3:
				return None, "sp_format"
			tag, json_part, crc_part = parts
			if tag != "SP":
				return None, "sp_tag"
			crc_part = crc_part.strip()
			if not crc_part:
				return None, "sp_crc_missing"
			# 容忍 crc 后面夹杂的 \r/空格/噪声字符：仅保留 0-9a-fA-F
			try:
				crc_part = "".join([c for c in crc_part if ("0" <= c <= "9") or ("a" <= c <= "f") or ("A" <= c <= "F")])
			except Exception:
				pass
			if not crc_part:
				return None, "sp_crc_missing"
			# 计算 CRC
			calc = _crc16_ibm(json_part.encode("utf-8"))
			exp = int(crc_part, 16) & 0xFFFF
			if calc != exp:
				return None, "crc"
			try:
				return json.loads(json_part), "ok"
			except Exception:
				return None, "json"
		except Exception:
			return None, "sp_parse"

	# 旧协议：整行 JSON
	try:
		return json.loads(line), "legacy_ok"
	except Exception:
		return None, "legacy_json"


def _try_fpioa_register_uart(uid, tx_pin, rx_pin):
	"""Best-effort FPIOA mapping for firmwares that require it (e.g. K210/MaixPy).

	If `fpioa_manager` is unavailable, this is a no-op.
	"""
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
				fm.register(tx_pin, tx_func, force=True)
			except Exception:
				pass
		if rx_func is not None:
			try:
				fm.register(rx_pin, rx_func, force=True)
			except Exception:
				pass
	except Exception:
		return


class VisionReceiver:
	def __init__(self, uart_id, baudrate, tx_pin, rx_pin, frame_width=320, frame_height=240, debug_print=False, label=None):
		self.frame_width = frame_width
		self.frame_height = frame_height
		self.debug_print = bool(debug_print)
		self.label = str(label) if label is not None else None
		self.uart_id = uart_id
		self.last_frame = None
		self.last_update_ms = time.ticks_ms()
		self._rx_buf = ""
		self._rx_chunk_bytes = 0
		self._rx_stat_total = 0
		self._rx_stat_ok = 0
		self._rx_stat_crc_fail = 0
		self._rx_stat_parse_fail = 0
		self._rx_stat_last_print_ms = time.ticks_ms()
		self.uart = self._create_uart(uart_id, baudrate, tx_pin, rx_pin)

	def age_ms(self):
		try:
			return time.ticks_diff(time.ticks_ms(), self.last_update_ms)
		except Exception:
			return None

	def rx_pending(self):
		"""Return UART pending bytes (if supported)."""
		any_fn = getattr(self.uart, "any", None)
		if callable(any_fn):
			try:
				return any_fn()
			except Exception:
				return None
		return None

	def rx_stat(self):
		"""Return a snapshot of RX statistics."""
		return {
			"chunk_bytes": self._rx_chunk_bytes,
			"total": self._rx_stat_total,
			"ok": self._rx_stat_ok,
			"crc_fail": self._rx_stat_crc_fail,
			"parse_fail": self._rx_stat_parse_fail,
		}

	def _create_uart(self, uart_id, baudrate, tx_pin, rx_pin):
		# 兼容不同固件的 UART 构造签名。
		# 注意：当明确配置了 uart_id 时，不要静默回退到其它 UART 号，否则会造成
		# “看似打开成功但一直收不到数据”的假象。
		candidates = [uart_id] if uart_id is not None else [5, 4, 3, 2, 1]
		tried = []
		for uid in candidates:
			if uid in tried:
				continue
			tried.append(uid)
			_try_fpioa_register_uart(uid, tx_pin, rx_pin)
			try:
				return UART(uid, baudrate=baudrate, tx=Pin(tx_pin), rx=Pin(rx_pin))
			except Exception:
				pass
			try:
				# 有些固件的 UART 接受 tx/rx 直接传 pin id（含字符串名），不需要 Pin(...) 包装
				return UART(uid, baudrate=baudrate, tx=tx_pin, rx=rx_pin)
			except Exception:
				pass
			try:
				return UART(uid, baudrate=baudrate)
			except Exception:
				pass
			try:
				return UART(uid, baudrate)
			except Exception:
				pass
			# 最后一种：部分实现支持 init 重新配置
			try:
				u = UART(uid)
			except Exception:
				continue
			init_fn = getattr(u, "init", None)
			if callable(init_fn):
				try:
					init_fn(baudrate=baudrate, tx=Pin(tx_pin), rx=Pin(rx_pin))
				except Exception:
					try:
						init_fn(baudrate=baudrate, tx=tx_pin, rx=rx_pin)
					except Exception:
						pass
					try:
						init_fn(baudrate)
					except Exception:
						pass
			return u

		raise ValueError("no usable UART id for vision receiver: {}".format(tried))

	def _normalize_target(self, target):
		if not target:
			return None

		x = int(target.get("x", self.frame_width // 2))
		y = int(target.get("y", self.frame_height // 2))
		w = int(target.get("w", target.get("width", 0)))
		h = int(target.get("h", target.get("height", 0)))
		size = int(target.get("size", max(w, h)))

		return {
			"x": x,
			"y": y,
			"w": w,
			"h": h,
			"size": size,
			"offset_x": x - self.frame_width // 2,
			"offset_y": y - self.frame_height // 2,
			"held": bool(target.get("held", False)),
			"method": target.get("method"),
			"color": target.get("color"),
		}

	def _normalize_frame(self, raw):
		# 防御：串口噪声可能导致 json.loads 得到 int/list 等非 dict。
		if not isinstance(raw, dict):
			return None
		frame = raw.get("frame", {})
		width = int(frame.get("w", self.frame_width))
		height = int(frame.get("h", self.frame_height))
		self.frame_width = width
		self.frame_height = height

		objects_count = raw.get("objects_count")
		if objects_count is None:
			objects = raw.get("objects")
			if isinstance(objects, list):
				objects_count = len(objects)
			else:
				objects_count = 1 if (raw.get("object") or raw.get("target")) else 0

		return {
			"frame": {"w": width, "h": height},
			"object": self._normalize_target(raw.get("object") or raw.get("target")),
			"zone": self._normalize_target(raw.get("zone") or raw.get("goal")),
			"field": self._normalize_target(raw.get("field") or raw.get("border") or raw.get("boundary")),
			"object_out_of_field": bool(raw.get("object_out_of_field", False)),
			"objects_count": int(objects_count),
			"ts": time.ticks_ms(),
		}

	def read_frame(self):
		# 优先使用 any/read 方式。
		# 重要：不少固件的 UART.readline() 会阻塞等待 '\n'，会把主循环卡死。
		# 因此：只要 any() 可用，就完全不走 readline() 路径。
		chunk = None
		any_fn = getattr(self.uart, "any", None)
		read_fn = getattr(self.uart, "read", None)
		if callable(any_fn) and callable(read_fn):
			try:
				n = any_fn()
				if not n or n <= 0:
					return None
				chunk = read_fn(n)
			except Exception:
				return None
		else:
			# 仅在缺少 any/read 支持时才尝试 readline（某些端口可能是非阻塞实现）
			readline_fn = getattr(self.uart, "readline", None)
			if callable(readline_fn):
				try:
					chunk = readline_fn()
				except Exception:
					chunk = None

		if not chunk:
			return None

		try:
			self._rx_chunk_bytes += len(chunk)
		except Exception:
			pass

		if isinstance(chunk, bytes):
			# 注意：如果这里 decode 失败，不能用 str(bytes) 回退，
			# 否则会得到 "b'...'") 形式的字符串，污染协议行并导致大量 parse_fail。
			try:
				chunk = chunk.decode("utf-8")
			except Exception:
				# 部分固件支持 errors 参数；不支持则再回退
				try:
					chunk = chunk.decode("utf-8", "ignore")
				except Exception:
					try:
						chunk = chunk.decode("latin-1")
					except Exception:
						chunk = ""

			# 过滤常见的 NUL 噪声
			try:
				if "\x00" in chunk:
					chunk = chunk.replace("\x00", "")
			except Exception:
				pass

		self._rx_buf += chunk
		# 防止异常情况下（一直没读到 \n）缓冲无限增长
		if len(self._rx_buf) > 4096:
			self._rx_buf = self._rx_buf[-2048:]

		# 只在拿到一整行后再解析 JSON
		if "\n" not in self._rx_buf:
			return None

		line, self._rx_buf = self._rx_buf.split("\n", 1)
		line = line.strip()
		if not line:
			return None

		if self.debug_print:
			if self.label:
				print("VISION_RX({}):".format(self.label), line)
			else:
				print("VISION_RX:", line)

		self._rx_stat_total += 1
		parsed, status = _parse_sp_line(line)
		# 必须是 dict 才算有效帧；否则宁可丢弃，避免 .get() 崩溃。
		if isinstance(parsed, dict):
			self._rx_stat_ok += 1
		else:
			if status == "crc":
				self._rx_stat_crc_fail += 1
			else:
				self._rx_stat_parse_fail += 1
			# CRC/解析失败时不更新 last_frame
			if self.debug_print:
				now = time.ticks_ms()
				if time.ticks_diff(now, self._rx_stat_last_print_ms) >= 1000:
					print(
						"VISION_RX_STAT({}):".format(self.label) if self.label else "VISION_RX_STAT:",
						"total=", self._rx_stat_total,
						"ok=", self._rx_stat_ok,
						"crc_fail=", self._rx_stat_crc_fail,
						"parse_fail=", self._rx_stat_parse_fail,
					)
					self._rx_stat_last_print_ms = now
			return None

		if self.debug_print:
			now = time.ticks_ms()
			if time.ticks_diff(now, self._rx_stat_last_print_ms) >= 1000:
				print(
					"VISION_RX_STAT({}):".format(self.label) if self.label else "VISION_RX_STAT:",
					"total=", self._rx_stat_total,
					"ok=", self._rx_stat_ok,
					"crc_fail=", self._rx_stat_crc_fail,
					"parse_fail=", self._rx_stat_parse_fail,
				)
				self._rx_stat_last_print_ms = now

		normalized = self._normalize_frame(parsed)
		if not normalized:
			# 类型不对/字段缺失等，视为解析失败但不中断主循环
			self._rx_stat_parse_fail += 1
			return None
		self.last_frame = normalized
		self.last_update_ms = self.last_frame["ts"]
		return self.last_frame

	def get_latest_frame(self, timeout_ms=500):
		frame = self.read_frame()
		if frame:
			return frame

		if not self.last_frame:
			return None

		now = time.ticks_ms()
		age_ms = time.ticks_diff(now, self.last_update_ms)
		if age_ms > timeout_ms:
			return None

		# If we're reusing a cached frame (no new UART line yet), mark targets as held
		# after a short delay to prevent blindly driving on stale detections.
		# Keep this delay small but >0 to tolerate partial UART lines.
		cached_hold_ms = 120
		try:
			cached_hold_ms = int(min(cached_hold_ms, max(0, timeout_ms // 2)))
		except Exception:
			cached_hold_ms = 120
		if age_ms >= cached_hold_ms:
			try:
				obj = self.last_frame.get("object")
				if isinstance(obj, dict):
					obj["held"] = True
				zone = self.last_frame.get("zone")
				if isinstance(zone, dict):
					zone["held"] = True
			except Exception:
				pass

		return self.last_frame
