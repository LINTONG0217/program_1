"""Two-anchor UWB localization helpers.

This module is designed for the current project constraint:
- yaw comes from IMU / odometry continuity
- UWB provides distance measurements to two fixed anchors
- the solver uses the predicted pose to choose between the mirrored
  circle intersections
"""

import math
import time

try:
	import ujson as json
except ImportError:
	import json

from machine import Pin, UART


def _pin_value(pin):
	if pin is None:
		return None
	try:
		return int(pin)
	except Exception:
		return pin


def _decode_line(line):
	if isinstance(line, bytes):
		try:
			line = line.decode("utf-8")
		except Exception:
			line = line.decode("latin1")
	return (line or "").strip()


def _try_float(value):
	text = str(value or "").strip()
	if not text:
		return None
	if text.endswith("m") or text.endswith("M"):
		text = text[:-1].strip()
	try:
		return float(text)
	except Exception:
		return None


def _try_int(value):
	number = _try_float(value)
	if number is None:
		return None
	try:
		return int(number)
	except Exception:
		return None


def _split_tokens(text):
	for ch in (",", ";", "|", "\t", "\r", "\n"):
		text = text.replace(ch, " ")
	return [token for token in text.split(" ") if token]


def _rotate_body_to_world(yaw_deg, forward_m, lateral_m):
	rad = math.radians(float(yaw_deg))
	cos_a = math.cos(rad)
	sin_a = math.sin(rad)
	return (
		forward_m * cos_a - lateral_m * sin_a,
		forward_m * sin_a + lateral_m * cos_a,
	)


class UWBRangeReceiver:
	def __init__(self, cfg):
		self.cfg = cfg
		tx_pin = getattr(cfg, "UWB_TX_PIN", None)
		rx_pin = getattr(cfg, "UWB_RX_PIN", None)
		uart_id = getattr(cfg, "UWB_UART_ID", 1)
		baudrate = getattr(cfg, "UWB_BAUDRATE", 115200)
		self.uart = self._build_uart(uart_id, baudrate, tx_pin, rx_pin)

		self.anchor_ids = [
			int(getattr(cfg, "UWB_ANCHOR_0_ID", 0)),
			int(getattr(cfg, "UWB_ANCHOR_1_ID", 1)),
		]
		self.ranges = {}
		self.last_line = None
		self.last_rx_ms = None
		self.poll_enable = bool(getattr(cfg, "UWB_RANGE_POLL_ENABLE", False))
		self.poll_ms = int(getattr(cfg, "UWB_RANGE_POLL_MS", 120))
		self.poll_cmd_template = str(getattr(cfg, "UWB_RANGE_CMD_TEMPLATE", "") or "")
		self.last_poll_ms = 0
		self.last_poll_cmd = None
		self._poll_index = 0
		self._awaiting_anchor_id = None

	def _build_uart(self, uart_id, baudrate, tx_pin=None, rx_pin=None):
		if tx_pin is None and rx_pin is None:
			return UART(uart_id, baudrate=baudrate)

		tx_obj = Pin(_pin_value(tx_pin))
		rx_obj = Pin(_pin_value(rx_pin))

		# Compatibility fallback for MicroPython ports that do not accept
		# tx=/rx= keyword arguments during UART construction.
		try:
			return UART(uart_id, baudrate=baudrate, tx=tx_obj, rx=rx_obj)
		except TypeError:
			try:
				return UART(uart_id, baudrate)
			except Exception:
				return UART(uart_id, baudrate=baudrate)
		except Exception:
			try:
				return UART(uart_id, baudrate)
			except Exception:
				return UART(uart_id, baudrate=baudrate)

	def _parse_json(self, line):
		try:
			obj = json.loads(line)
		except Exception:
			return None
		if not isinstance(obj, dict):
			return None

		anchor_id = None
		for key in ("id", "anchor", "anchor_id", "node", "base", "bs"):
			if key in obj:
				anchor_id = _try_int(obj.get(key))
				if anchor_id is not None:
					break

		distance = None
		for key in ("distance", "dist", "range", "r"):
			if key in obj:
				distance = _try_float(obj.get(key))
				if distance is not None:
					break

		if distance is None:
			return None
		if anchor_id is None:
			anchor_id = self._awaiting_anchor_id
		if anchor_id is None:
			return None
		return {
			"anchor_id": int(anchor_id),
			"distance_m": float(distance),
		}

	def _parse_kv_tokens(self, line):
		pairs = {}
		tokens = _split_tokens(line)
		plain_values = []
		for token in tokens:
			if "=" in token:
				key, value = token.split("=", 1)
				pairs[key.strip().lower()] = value.strip()
			elif ":" in token:
				key, value = token.split(":", 1)
				pairs[key.strip().lower()] = value.strip()
			else:
				plain_values.append(token.strip())

		anchor_id = None
		for key in ("id", "anchor", "anchor_id", "node", "base", "bs"):
			if key in pairs:
				anchor_id = _try_int(pairs.get(key))
				if anchor_id is not None:
					break

		distance = None
		for key in ("distance", "dist", "range", "r"):
			if key in pairs:
				distance = _try_float(pairs.get(key))
				if distance is not None:
					break

		if distance is not None:
			if anchor_id is None:
				anchor_id = self._awaiting_anchor_id
			if anchor_id is None:
				return None
			return {
				"anchor_id": int(anchor_id),
				"distance_m": float(distance),
			}

		if len(plain_values) >= 2:
			candidate_id = _try_int(plain_values[0])
			candidate_dist = _try_float(plain_values[1])
			if candidate_id is not None and candidate_dist is not None:
				return {
					"anchor_id": int(candidate_id),
					"distance_m": float(candidate_dist),
				}

		if len(plain_values) >= 1 and self._awaiting_anchor_id is not None:
			candidate_dist = _try_float(plain_values[0])
			if candidate_dist is not None:
				return {
					"anchor_id": int(self._awaiting_anchor_id),
					"distance_m": float(candidate_dist),
				}

		return None

	def _parse_line(self, line):
		line = _decode_line(line)
		if not line:
			return None
		self.last_line = line

		if line.startswith("{") and line.endswith("}"):
			parsed = self._parse_json(line)
			if parsed:
				return parsed

		return self._parse_kv_tokens(line)

	def _send_poll(self, now_ms):
		if not self.poll_enable:
			return
		if not self.poll_cmd_template:
			return
		if time.ticks_diff(now_ms, self.last_poll_ms) < self.poll_ms:
			return

		anchor_id = self.anchor_ids[self._poll_index % len(self.anchor_ids)]
		self._poll_index += 1
		self.last_poll_ms = now_ms
		self._awaiting_anchor_id = anchor_id

		cmd = self.poll_cmd_template
		if "{id}" in cmd:
			cmd = cmd.format(id=anchor_id)
		self.last_poll_cmd = cmd
		try:
			self.uart.write(cmd.encode("utf-8"))
		except Exception:
			try:
				self.uart.write(cmd)
			except Exception:
				pass

	def update(self):
		now_ms = time.ticks_ms()
		self._send_poll(now_ms)

		for _ in range(8):
			if not getattr(self.uart, "any", lambda: 0)():
				break
			line = self.uart.readline()
			if not line:
				break
			parsed = self._parse_line(line)
			if not parsed:
				continue

			anchor_id = int(parsed["anchor_id"])
			distance_m = float(parsed["distance_m"])
			if distance_m <= 0:
				continue
			self.ranges[anchor_id] = {
				"distance_m": distance_m,
				"ts": now_ms,
				"raw": self.last_line,
			}
			self.last_rx_ms = now_ms

	def get_fresh_ranges(self, max_age_ms=300):
		now_ms = time.ticks_ms()
		out = {}
		for anchor_id, data in self.ranges.items():
			ts = int(data.get("ts", 0) or 0)
			if ts <= 0:
				continue
			if time.ticks_diff(now_ms, ts) > int(max_age_ms):
				continue
			out[int(anchor_id)] = data
		return out


class TwoAnchorPoseSolver:
	def __init__(self, cfg):
		self.cfg = cfg
		self.anchor_a = {
			"id": int(getattr(cfg, "UWB_ANCHOR_0_ID", 0)),
			"x": float(getattr(cfg, "UWB_ANCHOR_0_X_M", 0.0)),
			"y": float(getattr(cfg, "UWB_ANCHOR_0_Y_M", 0.0)),
		}
		self.anchor_b = {
			"id": int(getattr(cfg, "UWB_ANCHOR_1_ID", 1)),
			"x": float(getattr(cfg, "UWB_ANCHOR_1_X_M", 1.0)),
			"y": float(getattr(cfg, "UWB_ANCHOR_1_Y_M", 0.0)),
		}
		self.range_max_age_ms = int(getattr(cfg, "UWB_RANGE_MAX_AGE_MS", 350))
		self.intersection_tol_m = float(getattr(cfg, "UWB_TWO_ANCHOR_INTERSECTION_TOL_M", 0.12))
		self.preferred_side = int(getattr(cfg, "UWB_TWO_ANCHOR_PREFERRED_SIDE", 1))
		self.tag_offset_forward_m = float(getattr(cfg, "UWB_TAG_OFFSET_FORWARD_M", 0.0))
		self.tag_offset_lateral_m = float(getattr(cfg, "UWB_TAG_OFFSET_LATERAL_M", 0.0))
		self.last_fix = None

	def _tag_world_from_pose(self, pose):
		if not isinstance(pose, dict):
			return None
		yaw = float(pose.get("yaw", 0.0))
		dx, dy = _rotate_body_to_world(yaw, self.tag_offset_forward_m, self.tag_offset_lateral_m)
		return {
			"x": float(pose.get("x", 0.0)) + dx,
			"y": float(pose.get("y", 0.0)) + dy,
		}

	def _center_world_from_tag(self, tag_x, tag_y, yaw_deg):
		dx, dy = _rotate_body_to_world(yaw_deg, self.tag_offset_forward_m, self.tag_offset_lateral_m)
		return {
			"x": float(tag_x) - dx,
			"y": float(tag_y) - dy,
			"yaw": float(yaw_deg),
		}

	def _circle_intersections(self, r0, r1):
		x0 = self.anchor_a["x"]
		y0 = self.anchor_a["y"]
		x1 = self.anchor_b["x"]
		y1 = self.anchor_b["y"]
		dx = x1 - x0
		dy = y1 - y0
		d = math.sqrt(dx * dx + dy * dy)
		if d <= 1e-6:
			return None

		a = (r0 * r0 - r1 * r1 + d * d) / (2.0 * d)
		h_sq = r0 * r0 - a * a
		tol_sq = self.intersection_tol_m * self.intersection_tol_m
		if h_sq < -tol_sq:
			return None
		if h_sq < 0.0:
			h_sq = 0.0
		h = math.sqrt(h_sq)

		ux = dx / d
		uy = dy / d
		px = x0 + a * ux
		py = y0 + a * uy
		rx = -uy * h
		ry = ux * h

		return [
			{"x": px + rx, "y": py + ry},
			{"x": px - rx, "y": py - ry},
		]

	def _side_sign(self, point):
		ax = self.anchor_a["x"]
		ay = self.anchor_a["y"]
		bx = self.anchor_b["x"]
		by = self.anchor_b["y"]
		return (bx - ax) * (float(point["y"]) - ay) - (by - ay) * (float(point["x"]) - ax)

	def _pick_candidate(self, candidates, predicted_pose):
		if not candidates:
			return None
		if len(candidates) == 1:
			return candidates[0]

		predicted_tag = self._tag_world_from_pose(predicted_pose)
		if predicted_tag is not None:
			best = None
			best_err = None
			for item in candidates:
				err = (
					(float(item["x"]) - predicted_tag["x"]) ** 2
					+ (float(item["y"]) - predicted_tag["y"]) ** 2
				)
				if best_err is None or err < best_err:
					best = item
					best_err = err
			if best is not None:
				return best

		if self.preferred_side != 0:
			for item in candidates:
				sign = self._side_sign(item)
				if self.preferred_side > 0 and sign >= 0:
					return item
				if self.preferred_side < 0 and sign <= 0:
					return item

		return candidates[0]

	def solve(self, predicted_pose, range_map):
		if not isinstance(range_map, dict):
			return None

		a_data = range_map.get(self.anchor_a["id"])
		b_data = range_map.get(self.anchor_b["id"])
		if not a_data or not b_data:
			return None

		r0 = float(a_data.get("distance_m", 0.0))
		r1 = float(b_data.get("distance_m", 0.0))
		candidates = self._circle_intersections(r0, r1)
		if not candidates:
			return None

		best_tag = self._pick_candidate(candidates, predicted_pose)
		if not best_tag:
			return None

		yaw = float((predicted_pose or {}).get("yaw", 0.0))
		center_pose = self._center_world_from_tag(best_tag["x"], best_tag["y"], yaw)
		now_ms = time.ticks_ms()
		fix = {
			"x": center_pose["x"],
			"y": center_pose["y"],
			"yaw": center_pose["yaw"],
			"ts": now_ms,
			"tag_x": float(best_tag["x"]),
			"tag_y": float(best_tag["y"]),
			"r0": r0,
			"r1": r1,
			"anchor0_id": self.anchor_a["id"],
			"anchor1_id": self.anchor_b["id"],
			"candidate_count": len(candidates),
		}
		self.last_fix = fix
		return fix

	def fresh_ranges(self, receiver):
		return receiver.get_fresh_ranges(self.range_max_age_ms)
