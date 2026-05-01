"""Dual vision wrapper (near + far OpenART over UART).

The controller expects a single `read_frame()` / `get_latest_frame()` interface.
This wrapper reads both receivers and returns a selected frame.

Selection policy (simple + robust):
- If near has an object (and not held), prefer near.
- Else if far has an object (and not held), use far.
- Else prefer the freshest frame.

It also adds `source` field: "near" / "far".
"""

import time


class DualVisionReceiver:
	def __init__(self, near_vision, far_vision, config=None):
		self.near = near_vision
		self.far = far_vision
		self.cfg = config
		self.last_frame = None
		self.last_update_ms = None
		self._dbg_last_ms = time.ticks_ms()
		self._dbg_last_source = None
		self._warn_last_ms = 0

	def _debug_status(self, near_frame, far_frame, chosen_source):
		try:
			enable = bool(getattr(self.cfg, "VISION_DUAL_STATUS_PRINT", True))
		except Exception:
			enable = True
		if not enable:
			return

		try:
			interval_ms = int(getattr(self.cfg, "VISION_DUAL_STATUS_PRINT_MS", 1000))
		except Exception:
			interval_ms = 1000
		interval_ms = max(200, interval_ms)

		now = time.ticks_ms()
		if self._dbg_last_source != chosen_source:
			print("VISION_SOURCE:", chosen_source)
			self._dbg_last_source = chosen_source

		if time.ticks_diff(now, self._dbg_last_ms) < interval_ms:
			return
		self._dbg_last_ms = now

		try:
			n_age = self.near.age_ms() if self.near and hasattr(self.near, "age_ms") else None
		except Exception:
			n_age = None
		try:
			f_age = self.far.age_ms() if self.far and hasattr(self.far, "age_ms") else None
		except Exception:
			f_age = None
		try:
			n_pending = self.near.rx_pending() if self.near and hasattr(self.near, "rx_pending") else None
		except Exception:
			n_pending = None
		try:
			f_pending = self.far.rx_pending() if self.far and hasattr(self.far, "rx_pending") else None
		except Exception:
			f_pending = None
		try:
			n_stat = self.near.rx_stat() if self.near and hasattr(self.near, "rx_stat") else None
		except Exception:
			n_stat = None
		try:
			f_stat = self.far.rx_stat() if self.far and hasattr(self.far, "rx_stat") else None
		except Exception:
			f_stat = None

		near_obj = False
		far_obj = False
		try:
			near_obj = bool(near_frame.get("object")) if isinstance(near_frame, dict) else False
		except Exception:
			pass
		try:
			far_obj = bool(far_frame.get("object")) if isinstance(far_frame, dict) else False
		except Exception:
			pass

		print(
			"VISION_DUAL:",
			"near_age_ms=", n_age,
			"near_pending=", n_pending,
			"near_obj=", near_obj,
			"near_total=", (n_stat or {}).get("total") if isinstance(n_stat, dict) else None,
			"near_ok=", (n_stat or {}).get("ok") if isinstance(n_stat, dict) else None,
			"near_crc=", (n_stat or {}).get("crc_fail") if isinstance(n_stat, dict) else None,
			"near_parse=", (n_stat or {}).get("parse_fail") if isinstance(n_stat, dict) else None,
			"near_bytes=", (n_stat or {}).get("chunk_bytes") if isinstance(n_stat, dict) else None,
			"far_age_ms=", f_age,
			"far_pending=", f_pending,
			"far_obj=", far_obj,
			"far_total=", (f_stat or {}).get("total") if isinstance(f_stat, dict) else None,
			"far_ok=", (f_stat or {}).get("ok") if isinstance(f_stat, dict) else None,
			"far_crc=", (f_stat or {}).get("crc_fail") if isinstance(f_stat, dict) else None,
			"far_parse=", (f_stat or {}).get("parse_fail") if isinstance(f_stat, dict) else None,
			"far_bytes=", (f_stat or {}).get("chunk_bytes") if isinstance(f_stat, dict) else None,
			"chosen=", chosen_source,
		)

		# Extra hint: one side appears offline.
		try:
			warn_gap_ms = 3000
			warn_period_ms = 2000
			if time.ticks_diff(now, self._warn_last_ms) >= warn_period_ms:
				if (self.near is not None) and (n_age is not None) and (n_age >= warn_gap_ms) and ((f_age is None) or (f_age < warn_gap_ms)):
					print("VISION_WARN: near offline? age_ms=", n_age)
					self._warn_last_ms = now
				elif (self.far is not None) and (f_age is not None) and (f_age >= warn_gap_ms) and ((n_age is None) or (n_age < warn_gap_ms)):
					print("VISION_WARN: far offline? age_ms=", f_age)
					self._warn_last_ms = now
		except Exception:
			pass

	def _frame_ts(self, frame):
		if not isinstance(frame, dict):
			return None
		return frame.get("ts")

	def _has_object(self, frame):
		if not isinstance(frame, dict):
			return False
		obj = frame.get("object")
		if not obj or not isinstance(obj, dict):
			return False
		if bool(obj.get("held", False)):
			return False
		return True

	def _read_one(self, vision):
		if not vision:
			return None
		read_fn = getattr(vision, "read_frame", None)
		if callable(read_fn):
			return read_fn()
		return None

	def _latest_one(self, vision, timeout_ms):
		if not vision:
			return None
		get_fn = getattr(vision, "get_latest_frame", None)
		if callable(get_fn):
			return get_fn(timeout_ms=timeout_ms)
		return self._read_one(vision)

	def read_frame(self):
		# Poll both to keep their internal buffers moving.
		near_new = self._read_one(self.near)
		far_new = self._read_one(self.far)

		near_frame = near_new or getattr(self.near, "last_frame", None)
		far_frame = far_new or getattr(self.far, "last_frame", None)

		chosen = None
		source = None
		if self._has_object(near_frame):
			chosen = near_frame
			source = "near"
		elif self._has_object(far_frame):
			chosen = far_frame
			source = "far"
		else:
			# No object: pick freshest.
			nts = self._frame_ts(near_frame)
			fts = self._frame_ts(far_frame)
			if nts is None and fts is None:
				return None
			if fts is None or (nts is not None and int(nts) >= int(fts)):
				chosen = near_frame
				source = "near"
			else:
				chosen = far_frame
				source = "far"

		if not chosen:
			return None

		# Shallow copy to avoid mutating underlying receiver cache.
		out = {}
		for k in chosen:
			out[k] = chosen[k]
		out["source"] = source
		self._debug_status(near_frame, far_frame, source)
		self.last_frame = out
		self.last_update_ms = out.get("ts", time.ticks_ms())
		return out

	def get_latest_frame(self, timeout_ms=500):
		frame = self.read_frame()
		if frame:
			return frame
		near_frame = self._latest_one(self.near, timeout_ms)
		far_frame = self._latest_one(self.far, timeout_ms)

		if self._has_object(near_frame):
			near_frame = dict(near_frame)
			near_frame["source"] = "near"
			self._debug_status(near_frame, far_frame, "near")
			return near_frame
		if self._has_object(far_frame):
			far_frame = dict(far_frame)
			far_frame["source"] = "far"
			self._debug_status(near_frame, far_frame, "far")
			return far_frame

		if near_frame and (not far_frame):
			n = dict(near_frame)
			n["source"] = "near"
			self._debug_status(n, far_frame, "near")
			return n
		if far_frame:
			f = dict(far_frame)
			f["source"] = "far"
			self._debug_status(near_frame, f, "far")
			return f
		self._debug_status(near_frame, far_frame, None)
		return None
