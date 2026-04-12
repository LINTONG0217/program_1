"""YDLIDAR T-MINI PLUS 串口接收（简化版）。

输出接口与 VisionReceiver 保持兼容：
- `get_latest_frame(timeout_ms=...)`
- `read_frame()`
- `last_update_ms`

控制层可将其当作“视觉目标源”使用：
- `object.offset_x` 由角度误差换算
- `object.size` 由距离反比换算
"""

import time
import sys

try:
	from machine import Pin
except Exception:
	try:
		from pyb import Pin  # type: ignore
	except Exception:
		Pin = None  # type: ignore

# Some firmwares (e.g. OpenMV/OpenART on mimxrt) expose UART via pyb.UART
# and/or do not support passing tx/rx pins in the constructor.
try:
	import pyb  # type: ignore
except Exception:
	pyb = None

try:
	from machine import UART as _MachineUART
except Exception:
	_MachineUART = None


def _try_fpioa_register_uart(uid, tx_pin, rx_pin):
	"""Best-effort FPIOA mapping（K210/MaixPy 兼容）。"""
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


def _clamp(v, lo, hi):
	return max(lo, min(hi, v))


def _norm_deg(deg):
	while deg > 180.0:
		deg -= 360.0
	while deg <= -180.0:
		deg += 360.0
	return deg


class YDLidarReceiver:
	"""解析 YDLIDAR 标准扫描包（AA 55 ...）并输出最近目标。"""

	def __init__(self, uart_id, baudrate, tx_pin, rx_pin, config):
		self.cfg = config
		self.frame_width = int(getattr(config, "FRAME_WIDTH", 320))
		self.frame_height = int(getattr(config, "FRAME_HEIGHT", 240))

		self.front_min_deg = float(getattr(config, "LIDAR_FRONT_MIN_DEG", -70.0))
		self.front_max_deg = float(getattr(config, "LIDAR_FRONT_MAX_DEG", 70.0))
		self.min_dist_mm = float(getattr(config, "LIDAR_MIN_DISTANCE_MM", 90.0))
		self.max_dist_mm = float(getattr(config, "LIDAR_MAX_DISTANCE_MM", 3500.0))
		self.angle_offset_deg = float(getattr(config, "LIDAR_ANGLE_OFFSET_DEG", 0.0))
		self.angle_sign = float(getattr(config, "LIDAR_ANGLE_SIGN", 1.0))
		self.offset_x_per_deg = float(getattr(config, "LIDAR_OFFSET_X_PER_DEG", 4.0))
		self.size_scale = float(getattr(config, "LIDAR_SIZE_SCALE", 60000.0))
		self.size_max = int(getattr(config, "LIDAR_SIZE_MAX", 420))
		self.hold_ms = int(getattr(config, "LIDAR_HOLD_MS", 120))

		try:
			self.send_start_cmd = bool(getattr(config, "LIDAR_SEND_START_CMD", True))
		except Exception:
			self.send_start_cmd = True
		try:
			self.start_with_stop = bool(getattr(config, "LIDAR_START_WITH_STOP", True))
		except Exception:
			self.start_with_stop = True
		try:
			self.start_delay_ms = int(getattr(config, "LIDAR_START_DELAY_MS", 120))
		except Exception:
			self.start_delay_ms = 120

		self.uart = None
		self.uart_impl = None
		self.uart_id = None
		self.uart_open_note = None
		self.uart = self._create_uart(uart_id, baudrate, tx_pin, rx_pin)
		# Some YDLIDAR models (including T-MINI PLUS) may not stream scan packets
		# until the host sends a SCAN command.
		if self.send_start_cmd:
			self._try_start_scan()
		self._buf = bytearray()
		self.last_frame = None
		self.last_update_ms = time.ticks_ms()

	def _uart_write(self, payload):
		if not self.uart:
			return False
		w = getattr(self.uart, "write", None)
		if not callable(w):
			return False
		try:
			w(payload)
			return True
		except Exception:
			return False

	def _try_start_scan(self):
		# Control-plane protocol:
		# - STOP: A5 65
		# - SCAN: A5 60
		# The scan data stream uses AA 55 ... packets.
		try:
			if self.start_with_stop:
				self._uart_write(b"\xA5\x65")
				try:
					time.sleep_ms(40)
				except Exception:
					time.sleep(0.04)
			self._uart_write(b"\xA5\x60")
			if int(self.start_delay_ms) > 0:
				try:
					time.sleep_ms(int(self.start_delay_ms))
				except Exception:
					time.sleep(int(self.start_delay_ms) / 1000.0)
			# Best-effort note for debugging output
			try:
				base = self.uart_open_note or ""
				self.uart_open_note = (base + " start_scan=A5 60").strip()
			except Exception:
				pass
		except Exception:
			return

	def get_uart_info(self):
		return {
			"platform": getattr(sys, "platform", None),
			"uart_impl": self.uart_impl,
			"uart_id": self.uart_id,
			"note": self.uart_open_note,
		}

	def _get_uart_impls(self):
		impls = []
		# machine.UART (if present)
		if _MachineUART is not None:
			impls.append(("machine", _MachineUART))
		# pyb.UART (OpenMV/OpenART style). Import lazily to avoid module import ordering issues.
		global pyb
		if pyb is None:
			try:
				import pyb as _pyb  # type: ignore
				pyb = _pyb
			except Exception:
				pyb = None
		try:
			if pyb is not None and hasattr(pyb, "UART"):
				u = getattr(pyb, "UART")
				# Avoid duplicates
				if not any(u is it for _, it in impls):
					impls.append(("pyb", u))
		except Exception:
			pass
		return impls

	def _close_uart(self, u):
		if not u:
			return
		deinit_fn = getattr(u, "deinit", None)
		if callable(deinit_fn):
			try:
				deinit_fn()
			except Exception:
				pass

	def _create_uart(self, uart_id, baudrate, tx_pin, rx_pin):
		plat = str(getattr(sys, "platform", "") or "").lower()
		is_mimxrt = plat == "mimxrt"

		# Optional: probe bytes to pick the UART that actually receives lidar stream.
		try:
			probe_ms = int(getattr(self.cfg, "LIDAR_UART_PROBE_MS", 120))
		except Exception:
			probe_ms = 120
		probe_ms = max(0, probe_ms)

		# Allow fallback scan when configured uart_id doesn't exist on this firmware.
		# Keep behavior conservative by default on non-mimxrt firmwares.
		try:
			auto_fallback = bool(getattr(self.cfg, "LIDAR_UART_AUTO_FALLBACK", is_mimxrt))
		except Exception:
			auto_fallback = bool(is_mimxrt)

		# Candidate UART ids: try configured first, then scan.
		candidates = []
		if uart_id is not None:
			candidates.append(int(uart_id))
		if auto_fallback:
			# Some firmwares number UART from 0, others from 1.
			# OpenMV/OpenART commonly has UART(1..4). Other boards may have (0..6).
			for uid in (0, 1, 2, 3, 4, 5, 6):
				if uid not in candidates:
					candidates.append(uid)
		if not candidates:
			candidates = [5, 4, 3, 2, 1, 0]

		impls = self._get_uart_impls()
		if not impls:
			raise ValueError("no UART implementation available")

		best = None  # (score, impl_name, uid, uart_obj, note)
		tried = []
		failures = []  # (impl_name, uid, label, err)

		def _score_uart(u, score_window_ms):
			if score_window_ms <= 0:
				return 0
			any_fn = getattr(u, "any", None)
			read_fn = getattr(u, "read", None)
			if not callable(any_fn) or not callable(read_fn):
				return 0
			start = time.ticks_ms()
			seen = 0
			while time.ticks_diff(time.ticks_ms(), start) < int(score_window_ms):
				try:
					n = any_fn()
				except Exception:
					n = 0
				if n and n > 0:
					try:
						chunk = read_fn(n)
					except Exception:
						chunk = None
					if chunk:
						try:
							seen += len(chunk)
						except Exception:
							seen += 1
				try:
					time.sleep_ms(5)
				except Exception:
					time.sleep(0.005)
			return int(seen)

		for uid in candidates:
			if is_mimxrt:
				# 针对 mimxrt (OpenART 等固件) 直连调用底层，跳过任意闭包以防解释器 BUG
				import machine
				try:
					u = machine.UART(int(uid), int(baudrate))
					
					# 如果启用了探测且不是配置文档中指定的ID，才需要验证是否能读到数据
					if probe_ms > 0 and auto_fallback and uart_id is not None and int(uid) != int(uart_id):
						# 注意：有的雷达不发 START 指令不会有数据。但如果真的物理配置不对，我们宁可错杀也不强开。
						score = _score_uart(u, probe_ms)
						if score > 0:
							self.uart_impl = "machine_direct"
							self.uart_id = uid
							self.uart_open_note = f"machine.UART({uid}, {baudrate}) score={score}"
							return u
						else:
							self._close_uart(u)
					else:
						# 这是 config 指定的直连 ID，或者是关闭了 probe 的情况。直接返回！
						self.uart_impl = "machine_direct"
						self.uart_id = uid
						self.uart_open_note = f"machine.UART({uid}, {baudrate}) direct"
						return u
				except Exception as e:
					failures.append(("machine", uid, "direct", repr(e)))
				continue

			for impl_name, uart_impl in impls:
				key = (impl_name, uid)
				if key in tried:
					continue
				tried.append(key)

				if not is_mimxrt:
					_try_fpioa_register_uart(uid, tx_pin, rx_pin)

				def _mimxrt_posbaud_direct():
					return uart_impl(uid, baudrate)

				def _idonly_init_posbaud():
					# Deprecated on mimxrt: some builds have broken init() and no deinit().
					u = uart_impl(uid)
					return u

				def _ctor0_init(init_call):
					# Some firmwares expose UART as a zero-arg constructor and require
					# configuring via .init(...).
					u = uart_impl()
					init_fn = getattr(u, "init", None)
					if callable(init_fn):
						init_call(init_fn)
					return u

				tries = []
				if is_mimxrt:
					# mimxrt: do NOT call init() and do NOT pass keyword args.
					# Use positional UART(uid, baud) only.
					baud_int = int(baudrate)
					tries += [
						("posbaud", lambda: uart_impl(uid, baud_int)),
						("idonly", lambda: uart_impl(uid)),
					]
				else:
					# Generic: try with pins then fallback pinless.
					tries += [
						("pins+Pin", lambda: uart_impl(uid, baudrate=baudrate, tx=Pin(tx_pin), rx=Pin(rx_pin))),
						("pins+raw", lambda: uart_impl(uid, baudrate=baudrate, tx=tx_pin, rx=rx_pin)),
						("pinless+baudrate", lambda: uart_impl(uid, baudrate=baudrate)),
						("pinless+posbaud", lambda: uart_impl(uid, baudrate)),
						("idonly", lambda: uart_impl(uid)),
					]

					# Fallback for firmwares where UART() takes no args and must be configured via init().
					tries += [
						(
							"ctor0+init+baudrate+pins+Pin",
							lambda: _ctor0_init(lambda init_fn: init_fn(baudrate=baudrate, tx=Pin(tx_pin), rx=Pin(rx_pin))),
						),
						(
							"ctor0+init+posbaud+pins+Pin",
							lambda: _ctor0_init(lambda init_fn: init_fn(baudrate, tx=Pin(tx_pin), rx=Pin(rx_pin))),
						),
						(
							"ctor0+init+baudrate+pins+raw",
							lambda: _ctor0_init(lambda init_fn: init_fn(baudrate=baudrate, tx=tx_pin, rx=rx_pin)),
						),
						(
							"ctor0+init+posbaud+pins+raw",
							lambda: _ctor0_init(lambda init_fn: init_fn(baudrate, tx=tx_pin, rx=rx_pin)),
						),
						(
							"ctor0+init+baudrate+rxPin",
							lambda: _ctor0_init(lambda init_fn: init_fn(baudrate=baudrate, rx=Pin(rx_pin))),
						),
						(
							"ctor0+init+posbaud+rxPin",
							lambda: _ctor0_init(lambda init_fn: init_fn(baudrate, rx=Pin(rx_pin))),
						),
						(
							"ctor0+init+baudrate+rxRaw",
							lambda: _ctor0_init(lambda init_fn: init_fn(baudrate=baudrate, rx=rx_pin)),
						),
						(
							"ctor0+init+posbaud+rxRaw",
							lambda: _ctor0_init(lambda init_fn: init_fn(baudrate, rx=rx_pin)),
						),
						(
							"ctor0+init+baudrate+pinless",
							lambda: _ctor0_init(lambda init_fn: init_fn(baudrate=baudrate)),
						),
						(
							"ctor0+init+posbaud+pinless",
							lambda: _ctor0_init(lambda init_fn: init_fn(baudrate)),
						),
						(
							"ctor0+noinit",
							lambda: uart_impl(),
						),
					]

				last_err = None
				for label, fn in tries:
					u = None
					try:
						u = fn()
					except Exception as e:
						last_err = e
						try:
							failures.append((impl_name, uid, label, repr(e)))
						except Exception:
							pass
						continue

					# If probing is enabled and we're scanning fallbacks, choose the uart
					# that actually receives bytes.
					score = 0
					if probe_ms > 0 and auto_fallback:
						score = _score_uart(u, probe_ms)
						note = "{}:{} probe_bytes={}".format(impl_name, label, score)
					else:
						note = "{}:{}".format(impl_name, label)

					# Prefer configured uid when it opens; otherwise pick the one with bytes.
					prefer = 1 if (uart_id is not None and int(uid) == int(uart_id)) else 0
					total_score = int(score) + prefer * 1000000
					if best is None or total_score > best[0]:
						# Close previous best UART object (if any) to avoid leaks.
						if best is not None:
							self._close_uart(best[3])
						best = (total_score, impl_name, uid, u, note)
					else:
						self._close_uart(u)

					# If this is the configured uart_id and it opened, we can stop early
					# when not probing.
					if (uart_id is not None and int(uid) == int(uart_id)) and (probe_ms <= 0 or not auto_fallback):
						break
				# end tries
				# Continue scanning other impls/uids

		if best is None:
			# Keep the error short but informative.
			msg = "no usable UART for ydlidar; tried={}"
			try:
				if failures:
					# Dedupe by (impl, uid, label, err) while preserving order.
					seen = set()
					uniq = []
					for item in failures:
						try:
							k = (item[0], int(item[1]), item[2], item[3])
						except Exception:
							k = item
						if k in seen:
							continue
						seen.add(k)
						uniq.append(item)

					# Heuristic: if every attempt fails with the same weird ValueError,
					# it's likely UART is unsupported/broken in this firmware/interpreter.
					try:
						err_set = set([it[3] for it in uniq if len(it) >= 4])
						if len(err_set) == 1:
							err0 = list(err_set)[0]
							if "too many values to unpack (expected 0)" in str(err0):
								msg += "; hint=UART API seems unsupported/broken on this runtime (all opens raise 'too many values to unpack (expected 0)'). Ensure you are running on the MAIN controller firmware that provides a working UART implementation (machine.UART or pyb.UART)."
					except Exception:
						pass

					preview = uniq[:12]
					tail = uniq[-6:] if len(uniq) > 12 else []
					if tail:
						msg += "; failures_preview={}; failures_tail={}".format(preview, tail)
					else:
						msg += "; failures_preview={}".format(preview)
			except Exception:
				pass
			raise ValueError(msg.format(tried))

		self.uart_impl = best[1]
		self.uart_id = best[2]
		self.uart_open_note = best[4]
		return best[3]

	def _in_front_window(self, ang_deg):
		mn = self.front_min_deg
		mx = self.front_max_deg
		if mn <= mx:
			return mn <= ang_deg <= mx
		# 支持跨 -180/180 的窗口
		return ang_deg >= mn or ang_deg <= mx

	def _angle_to_offset_x(self, ang_deg):
		off = int(ang_deg * self.offset_x_per_deg)
		lim = self.frame_width // 2 - 1
		return int(_clamp(off, -lim, lim))

	def _distance_to_size(self, dist_mm):
		if dist_mm <= 0:
			return 0
		size = int(self.size_scale / dist_mm)
		return int(_clamp(size, 0, self.size_max))

	def _target_from_nearest(self, nearest):
		if not nearest:
			return None
		d = float(nearest["distance_mm"])
		a = float(nearest["angle_deg"])
		offset_x = self._angle_to_offset_x(a)
		x = int(self.frame_width // 2 + offset_x)
		x = int(_clamp(x, 0, self.frame_width - 1))
		y = self.frame_height // 2
		size = self._distance_to_size(d)
		return {
			"x": x,
			"y": y,
			"w": size,
			"h": size,
			"size": size,
			"offset_x": offset_x,
			"offset_y": 0,
			"held": False,
			"method": "lidar",
			"distance_mm": int(d),
			"angle_deg": a,
		}

	def _parse_available(self):
		"""从串口读取并解析，返回 (has_packet, nearest_dict_or_None)。"""
		any_fn = getattr(self.uart, "any", None)
		read_fn = getattr(self.uart, "read", None)
		if not callable(any_fn) or not callable(read_fn):
			return False, None

		try:
			n = any_fn()
		except Exception:
			return False, None

		if not n or n <= 0:
			return False, None

		try:
			chunk = read_fn(n)
		except Exception:
			chunk = None
		if not chunk:
			return False, None

		if isinstance(chunk, str):
			chunk = chunk.encode("latin1")
		self._buf.extend(chunk)

		# 防止异常情况下缓冲无限增长
		if len(self._buf) > 8192:
			self._buf = self._buf[-4096:]

		has_packet = False
		nearest = None

		while True:
			if len(self._buf) < 10:
				break

			idx = bytes(self._buf).find(b"\xAA\x55")
			if idx < 0:
				# 只保留最后1字节，避免切断包头
				if len(self._buf) > 1:
					self._buf = self._buf[-1:]
				break
			if idx > 0:
				self._buf = self._buf[idx:]
				if len(self._buf) < 10:
					break

			# AA 55 CT LSN FSA_L FSA_H LSA_L LSA_H CS_L CS_H [2*LSN]
			lsn = int(self._buf[3])
			if lsn <= 0 or lsn > 120:
				self._buf = self._buf[1:]
				continue

			packet_len = 10 + lsn * 2
			if len(self._buf) < packet_len:
				break

			pkt = self._buf[:packet_len]
			self._buf = self._buf[packet_len:]
			has_packet = True

			fsa = int(pkt[4]) | (int(pkt[5]) << 8)
			lsa = int(pkt[6]) | (int(pkt[7]) << 8)
			start_deg = ((fsa >> 1) / 64.0)
			end_deg = ((lsa >> 1) / 64.0)

			if lsn > 1:
				diff = end_deg - start_deg
				if diff < 0:
					diff += 360.0
				step = diff / float(lsn - 1)
			else:
				step = 0.0

			for i in range(lsn):
				raw = int(pkt[10 + i * 2]) | (int(pkt[11 + i * 2]) << 8)
				dist_mm = raw / 4.0
				if dist_mm <= 0:
					continue
				if dist_mm < self.min_dist_mm or dist_mm > self.max_dist_mm:
					continue

				ang = (start_deg + step * i) % 360.0
				ang = _norm_deg((ang + self.angle_offset_deg) * self.angle_sign)
				if not self._in_front_window(ang):
					continue

				if nearest is None or dist_mm < nearest["distance_mm"]:
					nearest = {
						"distance_mm": dist_mm,
						"angle_deg": ang,
					}

		return has_packet, nearest

	def read_frame(self):
		has_packet, nearest = self._parse_available()
		if not has_packet:
			return None

		now = time.ticks_ms()
		frame = {
			"frame": {"w": self.frame_width, "h": self.frame_height},
			"object": self._target_from_nearest(nearest),
			"zone": None,
			"field": None,
			"object_out_of_field": False,
			"objects_count": 1 if nearest else 0,
			"source": "lidar",
			"ts": now,
		}

		self.last_frame = frame
		self.last_update_ms = now
		return frame

	def get_latest_frame(self, timeout_ms=500):
		frame = self.read_frame()
		if frame:
			return frame

		if not self.last_frame:
			return None

		age_ms = time.ticks_diff(time.ticks_ms(), self.last_update_ms)
		if age_ms > int(timeout_ms):
			return None

		# 缓存帧保护：短时间复用时标记 held，避免盲冲。
		obj = self.last_frame.get("object") if isinstance(self.last_frame, dict) else None
		if isinstance(obj, dict):
			obj["held"] = bool(age_ms >= self.hold_ms)

		return self.last_frame
