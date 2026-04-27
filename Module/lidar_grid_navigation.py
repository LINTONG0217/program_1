"""Lidar scan to occupancy grid, A* planning and short-range push helpers."""

import math
import time


def clamp(value, low, high):
	return max(low, min(high, value))


def normalize_angle_deg(angle):
	while angle > 180.0:
		angle -= 360.0
	while angle <= -180.0:
		angle += 360.0
	return angle


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


class LocalOccupancyAStar:
	"""Build a local occupancy grid in world coordinates and plan to the nearest object cluster."""

	def __init__(self, config):
		self.cfg = config
		self.width_m = float(getattr(config, "LIDAR_GRID_WIDTH_M", 2.4))
		self.height_m = float(getattr(config, "LIDAR_GRID_HEIGHT_M", 2.4))
		self.resolution_m = float(getattr(config, "LIDAR_GRID_RESOLUTION_M", 0.08))
		self.front_margin_m = float(getattr(config, "LIDAR_GRID_FRONT_MARGIN_M", 1.6))
		self.rear_margin_m = float(getattr(config, "LIDAR_GRID_REAR_MARGIN_M", 0.8))
		self.inflation_m = float(getattr(config, "LIDAR_GRID_INFLATION_M", 0.12))
		self.object_inflation_m = float(getattr(config, "LIDAR_OBJECT_INFLATION_M", 0.05))
		self.goal_standoff_m = float(getattr(config, "LIDAR_GOAL_STANDOFF_M", 0.28))
		self.cluster_angle_gap_deg = float(getattr(config, "LIDAR_CLUSTER_GAP_DEG", 7.0))
		self.cluster_dist_gap_mm = float(getattr(config, "LIDAR_CLUSTER_GAP_MM", 180.0))
		self.cluster_min_points = int(getattr(config, "LIDAR_CLUSTER_MIN_POINTS", 3))
		self.cols = max(8, int(round(self.width_m / self.resolution_m)))
		self.rows = max(8, int(round(self.height_m / self.resolution_m)))
		self.inflation_cells = max(0, int(round(self.inflation_m / self.resolution_m)))
		self.object_inflation_cells = max(0, int(round(self.object_inflation_m / self.resolution_m)))
		self.last_grid = None
		self.last_target = None
		self.last_path = None

	def _make_grid(self):
		return [[0 for _ in range(self.cols)] for _ in range(self.rows)]

	def _world_bounds(self, pose):
		x = float(pose.get("x", 0.0))
		y = float(pose.get("y", 0.0))
		x_min = x - self.rear_margin_m
		x_max = x + self.front_margin_m
		y_min = y - self.width_m * 0.5
		y_max = y + self.width_m * 0.5
		return x_min, x_max, y_min, y_max

	def world_to_cell(self, x_m, y_m, bounds):
		x_min, x_max, y_min, y_max = bounds
		if x_m < x_min or x_m >= x_max or y_m < y_min or y_m >= y_max:
			return None
		col = int((x_m - x_min) / self.resolution_m)
		row = int((y_m - y_min) / self.resolution_m)
		if row < 0 or row >= self.rows or col < 0 or col >= self.cols:
			return None
		return row, col

	def cell_to_world_center(self, row, col, bounds):
		x_min, _, y_min, _ = bounds
		x_m = x_min + (float(col) + 0.5) * self.resolution_m
		y_m = y_min + (float(row) + 0.5) * self.resolution_m
		return x_m, y_m

	def _neighbors(self, row, col):
		for dr in (-1, 0, 1):
			for dc in (-1, 0, 1):
				if dr == 0 and dc == 0:
					continue
				nr = row + dr
				nc = col + dc
				if 0 <= nr < self.rows and 0 <= nc < self.cols:
					yield nr, nc

	def _raytrace_free(self, grid, start, end):
		r0, c0 = start
		r1, c1 = end
		dr = abs(r1 - r0)
		dc = abs(c1 - c0)
		sr = 1 if r0 < r1 else -1
		sc = 1 if c0 < c1 else -1
		err = dc - dr
		r = r0
		c = c0
		while True:
			if (r, c) != (r1, c1):
				grid[r][c] = 0
			if r == r1 and c == c1:
				break
			e2 = err * 2
			if e2 > -dr:
				err -= dr
				c += sc
			if e2 < dc:
				err += dc
				r += sr

	def _inflate(self, grid, cells):
		if cells <= 0:
			return [row[:] for row in grid]
		out = [row[:] for row in grid]
		for row in range(self.rows):
			for col in range(self.cols):
				if grid[row][col] != 1:
					continue
				for rr in range(max(0, row - cells), min(self.rows, row + cells + 1)):
					for cc in range(max(0, col - cells), min(self.cols, col + cells + 1)):
						out[rr][cc] = 1
		return out

	def _cluster_scan_points(self, scan_points):
		if not scan_points:
			return []
		points = sorted(scan_points, key=lambda item: float(item.get("angle_deg", 0.0)))
		clusters = []
		current = [points[0]]
		for point in points[1:]:
			prev = current[-1]
			da = abs(float(point.get("angle_deg", 0.0)) - float(prev.get("angle_deg", 0.0)))
			dd = abs(float(point.get("distance_mm", 0.0)) - float(prev.get("distance_mm", 0.0)))
			if da <= self.cluster_angle_gap_deg and dd <= self.cluster_dist_gap_mm:
				current.append(point)
			else:
				clusters.append(current)
				current = [point]
		if current:
			clusters.append(current)
		return [cluster for cluster in clusters if len(cluster) >= self.cluster_min_points]

	def _cluster_to_world_target(self, cluster, pose):
		if not cluster:
			return None
		pose_x = float(pose.get("x", 0.0))
		pose_y = float(pose.get("y", 0.0))
		pose_yaw = float(pose.get("yaw", 0.0))
		sum_x = 0.0
		sum_y = 0.0
		sum_dist = 0.0
		for point in cluster:
			dist_m = float(point.get("distance_mm", 0.0)) / 1000.0
			angle_deg = pose_yaw + float(point.get("angle_deg", 0.0))
			rad = math.radians(angle_deg)
			wx = pose_x + dist_m * math.cos(rad)
			wy = pose_y + dist_m * math.sin(rad)
			sum_x += wx
			sum_y += wy
			sum_dist += dist_m
		count = float(len(cluster))
		center_x = sum_x / count
		center_y = sum_y / count
		mean_dist = sum_dist / count
		dir_x = center_x - pose_x
		dir_y = center_y - pose_y
		norm = math.sqrt(dir_x * dir_x + dir_y * dir_y)
		if norm <= 1e-6:
			return None
		goal_dist = max(0.08, mean_dist - self.goal_standoff_m)
		scale = goal_dist / mean_dist if mean_dist > 1e-6 else 0.0
		goal_x = pose_x + dir_x * scale
		goal_y = pose_y + dir_y * scale
		return {
			"center_x": center_x,
			"center_y": center_y,
			"goal_x": goal_x,
			"goal_y": goal_y,
			"distance_m": mean_dist,
			"points": len(cluster),
		}

	def build(self, pose, scan):
		bounds = self._world_bounds(pose)
		grid = self._make_grid()
		scan_points = list(scan.get("points", [])) if isinstance(scan, dict) else []
		start_cell = self.world_to_cell(float(pose.get("x", 0.0)), float(pose.get("y", 0.0)), bounds)

		for point in scan_points:
			dist_m = float(point.get("distance_mm", 0.0)) / 1000.0
			angle_deg = float(pose.get("yaw", 0.0)) + float(point.get("angle_deg", 0.0))
			rad = math.radians(angle_deg)
			wx = float(pose.get("x", 0.0)) + dist_m * math.cos(rad)
			wy = float(pose.get("y", 0.0)) + dist_m * math.sin(rad)
			cell = self.world_to_cell(wx, wy, bounds)
			if cell is None:
				continue
			row, col = cell
			if start_cell is not None:
				self._raytrace_free(grid, start_cell, cell)
			grid[row][col] = 1

		inflated = self._inflate(grid, self.inflation_cells)
		if start_cell is not None:
			sr, sc = start_cell
			inflated[sr][sc] = 0

		clusters = self._cluster_scan_points(scan_points)
		targets = []
		for cluster in clusters:
			target = self._cluster_to_world_target(cluster, pose)
			if target is None:
				continue
			target_cell = self.world_to_cell(target["goal_x"], target["goal_y"], bounds)
			object_cell = self.world_to_cell(target["center_x"], target["center_y"], bounds)
			if target_cell is None or object_cell is None:
				continue
			target["goal_cell"] = target_cell
			target["object_cell"] = object_cell
			targets.append(target)

		targets.sort(key=lambda item: (float(item.get("distance_m", 0.0)), -int(item.get("points", 0))))
		result = {
			"bounds": bounds,
			"grid": grid,
			"inflated_grid": inflated,
			"start_cell": start_cell,
			"targets": targets,
			"ts": int(scan.get("ts", ticks_ms())) if isinstance(scan, dict) else ticks_ms(),
		}
		self.last_grid = result
		return result

	def _prepare_goal_cell(self, grid, goal):
		if goal is None:
			return None
		row, col = goal
		if 0 <= row < self.rows and 0 <= col < self.cols and grid[row][col] == 0:
			return goal
		for radius in range(1, 4):
			for rr in range(max(0, row - radius), min(self.rows, row + radius + 1)):
				for cc in range(max(0, col - radius), min(self.cols, col + radius + 1)):
					if grid[rr][cc] == 0:
						return rr, cc
		return None

	def _heuristic(self, a, b):
		return abs(a[0] - b[0]) + abs(a[1] - b[1])

	def plan(self, pose, scan):
		grid_data = self.build(pose, scan)
		start = grid_data.get("start_cell")
		if start is None:
			self.last_target = None
			self.last_path = None
			return None
		grid = grid_data.get("inflated_grid")
		if not grid_data.get("targets"):
			self.last_target = None
			self.last_path = None
			return None

		for target in grid_data["targets"]:
			goal = target.get("goal_cell")
			if goal is None:
				continue
			path = self._astar(grid, start, goal)
			if path:
				world_path = [self.cell_to_world_center(row, col, grid_data["bounds"]) for row, col in path]
				result = {
					"target": target,
					"path_cells": path,
					"path_world": world_path,
					"grid": grid_data,
				}
				self.last_target = target
				self.last_path = result
				return result

		self.last_target = None
		self.last_path = None
		return None

	def _astar(self, grid, start, goal):
		goal = self._prepare_goal_cell(grid, goal)
		if goal is None:
			return None
		open_set = [start]
		came_from = {}
		g_score = {start: 0.0}
		f_score = {start: self._heuristic(start, goal)}
		closed = set()

		while open_set:
			current = min(open_set, key=lambda node: f_score.get(node, 1e9))
			if current == goal:
				path = [current]
				while current in came_from:
					current = came_from[current]
					path.append(current)
				path.reverse()
				return path

			open_set.remove(current)
			closed.add(current)

			for neighbor in self._neighbors(current[0], current[1]):
				if neighbor in closed:
					continue
				if grid[neighbor[0]][neighbor[1]] != 0:
					continue
				step_cost = 1.414 if neighbor[0] != current[0] and neighbor[1] != current[1] else 1.0
				tentative = g_score[current] + step_cost
				if tentative >= g_score.get(neighbor, 1e9):
					continue
				came_from[neighbor] = current
				g_score[neighbor] = tentative
				f_score[neighbor] = tentative + self._heuristic(neighbor, goal)
				if neighbor not in open_set:
					open_set.append(neighbor)
		return None

	def plan_to_world_goal(self, pose, scan, goal_x, goal_y, object_x=None, object_y=None):
		grid_data = self.build(pose, scan)
		start = grid_data.get("start_cell")
		if start is None:
			return None
		goal = self.world_to_cell(float(goal_x), float(goal_y), grid_data["bounds"])
		if goal is None:
			return None
		path = self._astar(grid_data["inflated_grid"], start, goal)
		if not path:
			return None
		world_path = [self.cell_to_world_center(row, col, grid_data["bounds"]) for row, col in path]
		target = {
			"goal_x": float(goal_x),
			"goal_y": float(goal_y),
			"center_x": float(object_x if object_x is not None else goal_x),
			"center_y": float(object_y if object_y is not None else goal_y),
			"goal_cell": goal,
			"object_cell": goal,
			"distance_m": math.sqrt((float(goal_x) - float(pose.get("x", 0.0))) ** 2 + (float(goal_y) - float(pose.get("y", 0.0))) ** 2),
			"points": 0,
		}
		return {
			"target": target,
			"path_cells": path,
			"path_world": world_path,
			"grid": grid_data,
		}


class PathFollower:
	def __init__(self, config):
		self.cfg = config
		self.max_speed = float(getattr(config, "GRID_FOLLOW_MAX_SPEED", 28.0))
		self.min_speed = float(getattr(config, "GRID_FOLLOW_MIN_SPEED", 10.0))
		self.max_strafe = float(getattr(config, "GRID_FOLLOW_MAX_STRAFE", 24.0))
		self.max_yaw = float(getattr(config, "GRID_FOLLOW_MAX_YAW", 18.0))
		self.lookahead = int(getattr(config, "GRID_FOLLOW_LOOKAHEAD_POINTS", 3))
		self.arrive_xy_m = float(getattr(config, "GRID_FOLLOW_ARRIVE_XY_M", 0.10))
		self.arrive_object_m = float(getattr(config, "GRID_APPROACH_OBJECT_M", 0.38))

	def _world_to_body(self, dx, dy, yaw_deg):
		rad = math.radians(yaw_deg)
		cos_a = math.cos(rad)
		sin_a = math.sin(rad)
		vx = dx * cos_a + dy * sin_a
		vy = -dx * sin_a + dy * cos_a
		return vx, vy

	def compute(self, pose, plan_result):
		if not plan_result:
			return {"vx": 0.0, "vy": 0.0, "vw": 0.0, "arrived": False}

		path_world = plan_result.get("path_world") or []
		target = plan_result.get("target") or {}
		if not path_world:
			return {"vx": 0.0, "vy": 0.0, "vw": 0.0, "arrived": False}

		goal_x = float(target.get("goal_x", path_world[-1][0]))
		goal_y = float(target.get("goal_y", path_world[-1][1]))
		object_x = float(target.get("center_x", goal_x))
		object_y = float(target.get("center_y", goal_y))
		pose_x = float(pose.get("x", 0.0))
		pose_y = float(pose.get("y", 0.0))
		pose_yaw = float(pose.get("yaw", 0.0))

		dist_goal = math.sqrt((goal_x - pose_x) * (goal_x - pose_x) + (goal_y - pose_y) * (goal_y - pose_y))
		dist_object = math.sqrt((object_x - pose_x) * (object_x - pose_x) + (object_y - pose_y) * (object_y - pose_y))
		if dist_goal <= self.arrive_xy_m or dist_object <= self.arrive_object_m:
			return {"vx": 0.0, "vy": 0.0, "vw": 0.0, "arrived": True}

		index = min(len(path_world) - 1, self.lookahead)
		ref_x, ref_y = path_world[index]
		dx = ref_x - pose_x
		dy = ref_y - pose_y
		vx_body, vy_body = self._world_to_body(dx, dy, pose_yaw)
		norm = math.sqrt(vx_body * vx_body + vy_body * vy_body)
		if norm > 1e-6:
			scale = self.max_speed / max(self.max_speed, norm / max(1e-6, self.arrive_xy_m))
			vx_body *= scale * 18.0
			vy_body *= scale * 18.0

		vx_cmd = clamp(vx_body, -self.max_speed, self.max_speed)
		vy_cmd = clamp(vy_body, -self.max_strafe, self.max_strafe)
		if abs(vx_cmd) > 0 and abs(vx_cmd) < self.min_speed:
			vx_cmd = self.min_speed if vx_cmd > 0 else -self.min_speed

		target_heading = math.degrees(math.atan2(object_y - pose_y, object_x - pose_x))
		yaw_error = normalize_angle_deg(target_heading - pose_yaw)
		vw_cmd = clamp(yaw_error * float(getattr(self.cfg, "GRID_FOLLOW_YAW_P", 0.22)), -self.max_yaw, self.max_yaw)
		return {"vx": vx_cmd, "vy": vy_cmd, "vw": vw_cmd, "arrived": False}


class OpenArtMiniReceiver:
	"""Best-effort UART line receiver for OpenArt mini classification output."""

	def __init__(self, config):
		self.cfg = config
		self.uart = None
		self._buf = bytearray()
		self.last_result = None
		self.last_update_ms = 0
		if not bool(getattr(config, "OPENART_MINI_ENABLE", False)):
			return
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

	def _parse_line(self, line):
		text = line.strip()
		if not text:
			return None
		label = None
		conf = 0.0
		extra = {}
		parts = text.split(",")
		for part in parts:
			item = part.strip()
			if "=" in item:
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
						extra[key] = float(value)
					except Exception:
						extra[key] = value
			elif label is None:
				label = item
		if not label:
			return None
		result = {
			"label": label,
			"confidence": conf,
			"raw": text,
		}
		result.update(extra)
		return result

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


def edge_target_from_label(config, label):
	label = str(label or "").strip().lower()
	mapping = getattr(config, "OPENART_TARGET_EDGE_MAP", None)
	if isinstance(mapping, dict):
		for key in mapping:
			if str(key).strip().lower() == label:
				return mapping[key]
	default_edge = getattr(config, "OPENART_DEFAULT_EDGE", "right")
	return default_edge


def exit_goal_for_edge(config, edge_name, object_x=None, object_y=None):
	field_w = float(getattr(config, "FIELD_WIDTH_M", 3.2))
	field_h = float(getattr(config, "FIELD_HEIGHT_M", 2.8))
	margin = float(getattr(config, "FIELD_EXIT_MARGIN_M", 0.08))
	edge = str(edge_name or "").strip().lower()
	x = field_w * 0.5 if object_x is None else float(object_x)
	y = field_h * 0.5 if object_y is None else float(object_y)
	if edge == "left":
		return -margin, clamp(y, 0.15, field_h - 0.15)
	if edge == "right":
		return field_w + margin, clamp(y, 0.15, field_h - 0.15)
	if edge == "top":
		return clamp(x, 0.15, field_w - 0.15), field_h + margin
	if edge == "bottom":
		return clamp(x, 0.15, field_w - 0.15), -margin
	return field_w + margin, clamp(y, 0.15, field_h - 0.15)


import collections

class KalmanFilter2D:
	def __init__(self, x=0.0, y=0.0, vx=0.0, vy=0.0, dt=0.1, process_var=1e-2, measure_var=1e-1):
		# 状态: [x, y, vx, vy]
		self.x = x
		self.y = y
		self.vx = vx
		self.vy = vy
		self.dt = dt
		self.P = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
		self.Q = [[process_var,0,0,0],[0,process_var,0,0],[0,0,process_var,0],[0,0,0,process_var]]
		self.R = [[measure_var,0],[0,measure_var]]

	def predict(self):
		# 预测下一步
		self.x += self.vx * self.dt
		self.y += self.vy * self.dt
		# 状态转移矩阵F
		F = [
			[1,0,self.dt,0],
			[0,1,0,self.dt],
			[0,0,1,0],
			[0,0,0,1]
		]
		# P = FPF^T + Q
		P = self.P
		# 只做对角线近似，简化运算
		for i in range(4):
			for j in range(4):
				P[i][j] = F[i][i]*P[i][j]*F[j][j] + self.Q[i][j]
		self.P = P

	def update(self, mx, my):
		# 观测更新
		z = [mx, my]
		x_prior = [self.x, self.y, self.vx, self.vy]
		# 观测矩阵H
		H = [
			[1,0,0,0],
			[0,1,0,0]
		]
		# 计算卡尔曼增益K
		S = [[self.P[0][0]+self.R[0][0], self.P[0][1]], [self.P[1][0], self.P[1][1]+self.R[1][1]]]
		K = [
			[self.P[0][0]/S[0][0], self.P[0][1]/S[1][1]],
			[self.P[1][0]/S[0][0], self.P[1][1]/S[1][1]],
			[self.P[2][0]/S[0][0], self.P[2][1]/S[1][1]],
			[self.P[3][0]/S[0][0], self.P[3][1]/S[1][1]]
		]
		# y = z - Hx
		y = [z[0] - self.x, z[1] - self.y]
		# 更新状态
		self.x += K[0][0]*y[0] + K[0][1]*y[1]
		self.y += K[1][0]*y[0] + K[1][1]*y[1]
		self.vx += K[2][0]*y[0] + K[2][1]*y[1]
		self.vy += K[3][0]*y[0] + K[3][1]*y[1]
		# 更新协方差P（简化）
		for i in range(4):
			for j in range(4):
				self.P[i][j] *= 0.8

	def get_state(self):
		return self.x, self.y, self.vx, self.vy


class MatchObjectTracker:
	def __init__(self, config):
		self.cfg = config
		self.tracks = []
		self.next_id = 1
		self.last_seen_ms = 0
		self.delivered_count = 0

	def _match_distance_m(self):
		return float(getattr(self.cfg, "MATCH_TRACK_MATCH_DIST_M", 0.32))

	def _ttl_ms(self):
		return int(getattr(self.cfg, "MATCH_TRACK_TTL_MS", 2200))

	def _inside_field(self, x, y):
		field_w = float(getattr(self.cfg, "FIELD_WIDTH_M", 3.2))
		field_h = float(getattr(self.cfg, "FIELD_HEIGHT_M", 2.8))
		margin = float(getattr(self.cfg, "FIELD_TRACK_MARGIN_M", 0.05))
		return (-margin) <= float(x) <= (field_w + margin) and (-margin) <= float(y) <= (field_h + margin)

	def update_from_targets(self, targets, now_ms=None):
		now_ms = ticks_ms() if now_ms is None else int(now_ms)
		for track in self.tracks:
			track["seen"] = False
			# 卡尔曼预测
			if "kf" in track:
				track["kf"].predict()
		for target in targets or []:
			tx = float(target.get("center_x", 0.0))
			ty = float(target.get("center_y", 0.0))
			best = None
			best_d = None
			for track in self.tracks:
				d = math.sqrt((tx - track["x"]) ** 2 + (ty - track["y"]) ** 2)
				if best_d is None or d < best_d:
					best = track
					best_d = d
			if best is not None and best_d is not None and best_d <= self._match_distance_m():
				# 卡尔曼更新
				if "kf" not in best:
					best["kf"] = KalmanFilter2D(best["x"], best["y"])
				best["kf"].update(tx, ty)
				kx, ky, kvx, kvy = best["kf"].get_state()
				best["x"] = kx
				best["y"] = ky
				best["vx"] = kvx
				best["vy"] = kvy
				best["last_seen_ms"] = now_ms
				best["seen"] = True
				best["active"] = True
				best["target"] = dict(target)
			else:
				# 新目标，初始化卡尔曼
				kf = KalmanFilter2D(tx, ty)
				self.tracks.append({
					"id": self.next_id,
					"x": tx,
					"y": ty,
					"vx": 0.0,
					"vy": 0.0,
					"last_seen_ms": now_ms,
					"seen": True,
					"active": True,
					"target": dict(target),
					"kf": kf,
				})
				self.next_id += 1
		self.last_seen_ms = now_ms if targets else self.last_seen_ms
		fresh = []
		for track in self.tracks:
			age = ticks_diff(now_ms, track["last_seen_ms"])
			if age > self._ttl_ms():
				continue
			if not self._inside_field(track["x"], track["y"]):
				continue
			fresh.append(track)
		self.tracks = fresh
		return self.get_active_tracks()

	def mark_delivered(self, track_id=None):
		if track_id is not None:
			self.tracks = [track for track in self.tracks if int(track.get("id", -1)) != int(track_id)]
		self.delivered_count += 1

	def get_active_tracks(self):
		return [dict(track) for track in self.tracks if bool(track.get("active", True))]

	def nearest_track(self, x, y):
		best = None
		best_d = None
		for track in self.tracks:
			d = math.sqrt((float(x) - track["x"]) ** 2 + (float(y) - track["y"]) ** 2)
			if best_d is None or d < best_d:
				best = track
				best_d = d
		return dict(best) if best is not None else None

	def empty_for_too_long(self, now_ms=None):
		now_ms = ticks_ms() if now_ms is None else int(now_ms)
		if self.tracks:
			return False
		if not self.last_seen_ms:
			return False
		confirm_ms = int(getattr(self.cfg, "MISSION_EMPTY_CONFIRM_MS", 5000))
		return ticks_diff(now_ms, self.last_seen_ms) >= confirm_ms


def edge_push_vector(config, edge_name):
	edge = str(edge_name or "").strip().lower()
	push_speed = float(getattr(config, "OPENART_EDGE_PUSH_SPEED", 22.0))
	forward_speed = float(getattr(config, "OPENART_EDGE_FORWARD_SPEED", 30.0))
	if edge == "left":
		return {"vx": forward_speed, "vy": push_speed, "vw": 0.0}
	if edge == "right":
		return {"vx": forward_speed, "vy": -push_speed, "vw": 0.0}
	if edge == "top":
		return {"vx": forward_speed, "vy": 0.0, "vw": 10.0}
	if edge == "bottom":
		return {"vx": forward_speed, "vy": 0.0, "vw": -10.0}
	return {"vx": forward_speed, "vy": 0.0, "vw": 0.0}
