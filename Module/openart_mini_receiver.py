"""Small OpenArt UART receiver for memory-limited MicroPython tests."""

import time


def ticks_ms():
	fn = getattr(time, "ticks_ms", None)
	if callable(fn):
		return fn()
	return int(time.time() * 1000)


def ticks_diff(a, b):
	fn = getattr(time, "ticks_diff", None)
	if callable(fn):
		return fn(a, b)
	return a - b


class OpenArtMiniReceiver:
	def __init__(self, config):
		self.cfg = config
		self.uart = None
		self._buf = bytearray()
		self.last_result = None
		self.last_update_ms = 0
		if bool(getattr(config, "OPENART_MINI_ENABLE", False)):
			self.uart = self._create_uart()

	def _create_uart(self):
		try:
			from machine import Pin, UART
		except Exception:
			try:
				import pyb  # type: ignore
				Pin = None
				UART = pyb.UART
			except Exception:
				return None
		uart_id = int(getattr(self.cfg, "OPENART_MINI_UART_ID", 4))
		baudrate = int(getattr(self.cfg, "OPENART_MINI_BAUDRATE", 115200))
		tx_pin = getattr(self.cfg, "OPENART_MINI_TX_PIN", None)
		rx_pin = getattr(self.cfg, "OPENART_MINI_RX_PIN", None)
		if tx_pin is not None and rx_pin is not None and Pin is not None:
			try:
				return UART(uart_id, baudrate=baudrate, tx=Pin(tx_pin), rx=Pin(rx_pin))
			except Exception:
				try:
					return UART(uart_id, baudrate, tx=Pin(tx_pin), rx=Pin(rx_pin))
				except Exception:
					pass
		try:
			return UART(uart_id, baudrate)
		except Exception:
			try:
				return UART(uart_id)
			except Exception:
				return None

	def _crc16_ibm(self, data_bytes, init=0xFFFF):
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

	def _json_string(self, text, key, default=""):
		mark = '"' + key + '"'
		i = text.find(mark)
		if i < 0:
			return default
		i = text.find(":", i)
		if i < 0:
			return default
		i += 1
		while i < len(text) and text[i] in " \t":
			i += 1
		if i >= len(text) or text[i] != '"':
			return default
		i += 1
		j = text.find('"', i)
		if j < 0:
			return default
		return text[i:j]

	def _json_number(self, text, key, default=0.0):
		mark = '"' + key + '"'
		i = text.find(mark)
		if i < 0:
			return default
		i = text.find(":", i)
		if i < 0:
			return default
		i += 1
		while i < len(text) and text[i] in " \t":
			i += 1
		j = i
		while j < len(text) and text[j] not in ",}":
			j += 1
		try:
			return float(text[i:j])
		except Exception:
			return default

	def _json_bool(self, text, key, default=False):
		mark = '"' + key + '"'
		i = text.find(mark)
		if i < 0:
			return default
		i = text.find(":", i)
		if i < 0:
			return default
		value = text[i + 1:i + 8].strip().lower()
		if value.startswith("true"):
			return True
		if value.startswith("false"):
			return False
		return default

	def _parse_json_flat(self, text, raw):
		label = self._json_string(text, "label", "none")
		result = {
			"label": label,
			"confidence": self._json_number(text, "confidence", 0.0),
			"cx": self._json_number(text, "cx", -1),
			"cy": self._json_number(text, "cy", -1),
			"pixels": self._json_number(text, "pixels", 0),
			"held": self._json_bool(text, "held", False),
			"raw": raw,
		}
		return result

	def _parse_sp_crc_json(self, text):
		if not text.startswith("SP|"):
			return None
		try:
			_, body, crc_text = text.split("|", 2)
			expected = int(crc_text.strip(), 16)
		except Exception:
			return None
		data = body.encode("utf-8")
		if self._crc16_ibm(data) != expected:
			return None
		return self._parse_json_flat(body, text)

	def _parse_kv_line(self, text):
		label = None
		conf = 0.0
		result = {"raw": text}
		for part in text.split(","):
			item = part.strip()
			if "=" not in item:
				if label is None:
					label = item
				continue
			key, value = item.split("=", 1)
			key = key.strip().lower()
			value = value.strip()
			if key in ("label", "class", "name"):
				label = value
			elif key in ("score", "conf", "confidence"):
				try:
					conf = float(value)
				except Exception:
					conf = 0.0
			elif key in ("cx", "cy", "x", "y", "pixels", "area", "w", "h"):
				try:
					result[key] = float(value)
				except Exception:
					result[key] = value
		if not label:
			return None
		result["label"] = label
		result["confidence"] = conf
		return result

	def _parse_line(self, line):
		text = line.strip()
		if not text:
			return None
		result = self._parse_sp_crc_json(text)
		if result is not None:
			return result
		if text.startswith("{"):
			return self._parse_json_flat(text, text)
		return self._parse_kv_line(text)

	def update(self):
		if self.uart is None:
			return None
		any_fn = getattr(self.uart, "any", None)
		read_fn = getattr(self.uart, "read", None)
		if not callable(any_fn) or not callable(read_fn):
			return self.last_result
		try:
			n = any_fn()
		except Exception:
			n = 0
		if not n:
			return self.last_result
		try:
			chunk = read_fn(n)
		except Exception:
			chunk = None
		if not chunk:
			return self.last_result
		if isinstance(chunk, str):
			chunk = chunk.encode("utf-8")
		self._buf.extend(chunk)
		if len(self._buf) > 512:
			self._buf = self._buf[-256:]
		while True:
			idx = self._buf.find(b"\n")
			if idx < 0:
				break
			line = bytes(self._buf[:idx]).decode("utf-8", "ignore")
			self._buf = self._buf[idx + 1:]
			result = self._parse_line(line)
			if result is not None:
				self.last_result = result
				self.last_update_ms = ticks_ms()
		return self.last_result

	def get_latest(self, timeout_ms=800):
		self.update()
		if not self.last_result:
			return None
		if ticks_diff(ticks_ms(), self.last_update_ms) > int(timeout_ms):
			return None
		return self.last_result
