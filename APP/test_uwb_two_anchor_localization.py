import math
import time
from machine import Pin

from BSP.board_runtime import MainControl
from BSP.motor_actuator import Motor
from BSP.omni_chassis_driver import OmniChassis
from Module import config
from Module.dual_car_coop import build_robot_status
from Module.lidar_grid_navigation import (
	LocalOccupancyAStar,
	MatchObjectTracker,
	OpenArtMiniReceiver,
	PathFollower,
	edge_target_from_label,
	exit_goal_for_edge,
)
from Module.pose_estimation import PoseEstimator
from Module.uart_car_link import UartCarLink
from Module.uwb_two_anchor_localizer import TwoAnchorPoseSolver, UWBRangeReceiver
from Module.ydlidar_receiver import YDLidarReceiver


class _DisplayChassisProxy:
	def __init__(self):
		self.last_vx = 0.0
		self.last_vy = 0.0


def wait_c14_start():
	if not bool(getattr(config, "TEST_WAIT_C14_ENABLE", True)):
		return
	pin_name = getattr(config, "NAV_KEY_PIN", "C14")

	pull_up = getattr(Pin, "PULL_UP_47K", None)
	if pull_up is not None:
		try:
			key = Pin(pin_name, Pin.IN, pull_up)
		except Exception:
			key = Pin(pin_name, Pin.IN)
	else:
		key = Pin(pin_name, Pin.IN)

	print("press {} to start grid navigation + push test".format(pin_name))
	while key.value() != 0:
		time.sleep_ms(20)
	time.sleep_ms(60)
	while key.value() == 0:
		time.sleep_ms(20)


def build_chassis():
	motor_fl = Motor(*config.MOTOR_FL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fl", False))
	motor_fr = Motor(*config.MOTOR_FR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("fr", True))
	motor_bl = Motor(*config.MOTOR_BL_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("bl", False))
	motor_br = Motor(*config.MOTOR_BR_PINS, freq=config.PWM_FREQ, pwm_max=config.PWM_MAX, reverse=config.MOTOR_REVERSE.get("br", True))
	return OmniChassis(motor_fl, motor_fr, motor_bl, motor_br, config=config)


def _format_range(anchor_id, range_map):
	data = range_map.get(anchor_id) if isinstance(range_map, dict) else None
	if not data:
		return "id{}=--".format(anchor_id)
	return "id{}={:.3f}m".format(anchor_id, float(data.get("distance_m", 0.0)))


def _distance_m(a_x, a_y, b_x, b_y):
	dx = float(a_x) - float(b_x)
	dy = float(a_y) - float(b_y)
	return math.sqrt(dx * dx + dy * dy)


def _short_path_text(plan_result):
	if not plan_result:
		return "path=none"
	target = plan_result.get("target") or {}
	path_world = plan_result.get("path_world") or []
	return "path={} target=({:.2f},{:.2f}) obj=({:.2f},{:.2f})".format(
		len(path_world),
		float(target.get("goal_x", 0.0)),
		float(target.get("goal_y", 0.0)),
		float(target.get("center_x", 0.0)),
		float(target.get("center_y", 0.0)),
	)


def _edge_name(edge_name):
	edge = str(edge_name or "").strip().lower()
	if edge in ("left", "right", "top", "bottom"):
		return edge
	return "right"


def _point_near_target_edge(config, edge_name, x_m, y_m, band_m=None):
	field_w = float(getattr(config, "FIELD_WIDTH_M", 3.2))
	field_h = float(getattr(config, "FIELD_HEIGHT_M", 2.8))
	band = float(getattr(config, "COOP_DELIVER_EDGE_NEAR_M", 0.22)) if band_m is None else float(band_m)
	cross_margin = float(getattr(config, "COOP_DELIVER_CROSS_MARGIN_M", 0.25))
	x = float(x_m)
	y = float(y_m)
	edge = _edge_name(edge_name)
	if edge == "left":
		return x <= band and (-cross_margin) <= y <= (field_h + cross_margin)
	if edge == "right":
		return x >= (field_w - band) and (-cross_margin) <= y <= (field_h + cross_margin)
	if edge == "top":
		return y >= (field_h - band) and (-cross_margin) <= x <= (field_w + cross_margin)
	return y <= band and (-cross_margin) <= x <= (field_w + cross_margin)


def _point_out_target_edge(config, edge_name, x_m, y_m, margin_m=None):
	field_w = float(getattr(config, "FIELD_WIDTH_M", 3.2))
	field_h = float(getattr(config, "FIELD_HEIGHT_M", 2.8))
	margin = float(getattr(config, "FIELD_EXIT_MARGIN_M", 0.08)) if margin_m is None else float(margin_m)
	cross_margin = float(getattr(config, "COOP_DELIVER_CROSS_MARGIN_M", 0.25))
	x = float(x_m)
	y = float(y_m)
	edge = _edge_name(edge_name)
	if edge == "left":
		return x < -margin and (-cross_margin) <= y <= (field_h + cross_margin)
	if edge == "right":
		return x > (field_w + margin) and (-cross_margin) <= y <= (field_h + cross_margin)
	if edge == "top":
		return y > (field_h + margin) and (-cross_margin) <= x <= (field_w + cross_margin)
	return y < -margin and (-cross_margin) <= x <= (field_w + cross_margin)


def _transport_elapsed_ms(now_ms, start_ms):
	if start_ms is None:
		return 0
	return time.ticks_diff(now_ms, start_ms)


def _angle_delta_deg(current_deg, previous_deg):
	delta = float(current_deg) - float(previous_deg)
	while delta > 180.0:
		delta -= 360.0
	while delta <= -180.0:
		delta += 360.0
	return delta


def _track_xy(track):
	if not isinstance(track, dict):
		return None, None
	target = track.get("target") if isinstance(track.get("target"), dict) else {}
	x = track.get("x", target.get("center_x"))
	y = track.get("y", target.get("center_y"))
	if x is None or y is None:
		return None, None
	return float(x), float(y)


def _track_in_mission_field(config, track):
	x, y = _track_xy(track)
	if x is None or y is None:
		return False
	field_w = float(getattr(config, "FIELD_WIDTH_M", 3.2))
	field_h = float(getattr(config, "FIELD_HEIGHT_M", 2.8))
	margin = float(getattr(config, "MISSION_FIELD_EMPTY_MARGIN_M", 0.03))
	return (-margin) <= x <= (field_w + margin) and (-margin) <= y <= (field_h + margin)


def _filter_field_tracks(config, tracks):
	return [track for track in (tracks or []) if _track_in_mission_field(config, track)]


def _nearest_track_from_list(tracks, x_m, y_m):
	best = None
	best_d = None
	for track in tracks or []:
		tx, ty = _track_xy(track)
		if tx is None or ty is None:
			continue
		d = _distance_m(x_m, y_m, tx, ty)
		if best_d is None or d < best_d:
			best = track
			best_d = d
	return best


def _plan_to_track(planner, pose, scan, track):
	if not track or not scan:
		return None
	target = track.get("target") if isinstance(track.get("target"), dict) else {}
	object_x, object_y = _track_xy(track)
	if object_x is None or object_y is None:
		return None
	goal_x = target.get("goal_x")
	goal_y = target.get("goal_y")
	if goal_x is None or goal_y is None:
		pose_x = float(pose.get("x", 0.0))
		pose_y = float(pose.get("y", 0.0))
		dx = object_x - pose_x
		dy = object_y - pose_y
		dist = math.sqrt(dx * dx + dy * dy)
		standoff = float(getattr(config, "LIDAR_GOAL_STANDOFF_M", 0.28))
		if dist <= 1e-6:
			goal_x = object_x
			goal_y = object_y
		else:
			goal_x = object_x - dx / dist * standoff
			goal_y = object_y - dy / dist * standoff
	return planner.plan_to_world_goal(pose, scan, goal_x, goal_y, object_x=object_x, object_y=object_y)


def build_car_link():
	if not bool(getattr(config, "COMPETITION_DUAL_ENABLE", True)):
		return None
	try:
		return UartCarLink(
			getattr(config, "CAR_LINK_UART_ID", 2),
			getattr(config, "CAR_LINK_BAUDRATE", 115200),
			tx_pin=getattr(config, "CAR_LINK_TX_PIN", None),
			rx_pin=getattr(config, "CAR_LINK_RX_PIN", None),
		)
	except Exception as exc:
		print("car link disabled:", exc)
		return None


def _print_uwb_startup_diagnostics():
	uwb_uart_id = getattr(config, "UWB_UART_ID", None)
	lidar_uart_id = getattr(config, "LIDAR_UART_ID", None)
	lidar_enable = bool(getattr(config, "UWB_TWO_ANCHOR_LIDAR_ENABLE", False))
	if lidar_enable and uwb_uart_id is not None and lidar_uart_id is not None:
		try:
			if int(uwb_uart_id) == int(lidar_uart_id):
				print(
					"[WARN] UWB and lidar both use UART{}; lidar init may remap or occupy the same UART and make UWB unreadable.".format(
						int(uwb_uart_id)
					)
				)
				print(
					"[WARN] UWB pins=({}, {}), lidar pins=({}, {})".format(
						getattr(config, "UWB_TX_PIN", None),
						getattr(config, "UWB_RX_PIN", None),
						getattr(config, "LIDAR_TX_PIN", None),
						getattr(config, "LIDAR_RX_PIN", None),
					)
				)
		except Exception:
			pass

	poll_enable = bool(getattr(config, "UWB_RANGE_POLL_ENABLE", False))
	cmd = str(getattr(config, "UWB_RANGE_CMD_TEMPLATE", "") or "")
	if poll_enable and cmd and "{id}" not in cmd:
		print(
			"[WARN] UWB_RANGE_CMD_TEMPLATE has no {id}; replies without anchor_id will be guessed as alternating anchor 0/1."
		)
		print(
			"[WARN] For two-anchor localization, prefer a command/output that carries anchor id explicitly."
		)


def main():
	print("test: two-anchor uwb + lidar grid + astar + openart push")
	print(
		"anchors: id{}=({:.2f},{:.2f}) id{}=({:.2f},{:.2f})".format(
			int(getattr(config, "UWB_ANCHOR_0_ID", 0)),
			float(getattr(config, "UWB_ANCHOR_0_X_M", 0.0)),
			float(getattr(config, "UWB_ANCHOR_0_Y_M", 0.0)),
			int(getattr(config, "UWB_ANCHOR_1_ID", 1)),
			float(getattr(config, "UWB_ANCHOR_1_X_M", 0.0)),
			float(getattr(config, "UWB_ANCHOR_1_Y_M", 0.0)),
		)
	)
	print(
		"lidar sector: [{:.1f}, {:.1f}] deg grid_res={:.2f}m".format(
			float(getattr(config, "LIDAR_FRONT_MIN_DEG", -45.0)),
			float(getattr(config, "LIDAR_FRONT_MAX_DEG", 45.0)),
			float(getattr(config, "LIDAR_GRID_RESOLUTION_M", 0.08)),
		)
	)
	print("lidar enabled: {}".format(bool(getattr(config, "UWB_TWO_ANCHOR_LIDAR_ENABLE", False))))
	print("openart mini enabled: {}".format(bool(getattr(config, "OPENART_MINI_ENABLE", False))))
	print("imu yaw fusion enabled: {}".format(bool(getattr(config, "UWB_TWO_ANCHOR_USE_IMU", False))))
	_print_uwb_startup_diagnostics()

	control_board = MainControl(config)
	control_board.init()
	chassis = build_chassis()
	display_proxy = _DisplayChassisProxy()
	wait_c14_start()

	estimator = PoseEstimator(config)
	if bool(getattr(config, "UWB_TWO_ANCHOR_INIT_ENABLE", True)):
		estimator.request_pose_init(
			getattr(config, "UWB_TWO_ANCHOR_INIT_X_M", 0.25),
			getattr(config, "UWB_TWO_ANCHOR_INIT_Y_M", 0.25),
			getattr(config, "UWB_TWO_ANCHOR_INIT_YAW_DEG", 0.0),
		)

	uwb_receiver = UWBRangeReceiver(config)
	uwb_solver = TwoAnchorPoseSolver(config)
	lidar_enable = bool(getattr(config, "UWB_TWO_ANCHOR_LIDAR_ENABLE", False))
	if lidar_enable:
		lidar = YDLidarReceiver(
			uart_id=getattr(config, "LIDAR_UART_ID", 5),
			baudrate=getattr(config, "LIDAR_BAUDRATE", 230400),
			tx_pin=getattr(config, "LIDAR_TX_PIN", 4),
			rx_pin=getattr(config, "LIDAR_RX_PIN", 5),
			config=config,
		)
		planner = LocalOccupancyAStar(config)
		tracker = MatchObjectTracker(config)
		follower = PathFollower(config)
	else:
		lidar = None
		planner = None
		tracker = None
		follower = None
	openart = OpenArtMiniReceiver(config)
	car_link = build_car_link()

	state = "search_object"
	current_plan = None
	current_track = None
	current_edge = None
	last_plan_ms = 0
	last_print_ms = 0
	last_sync_ms = 0
	last_recognition = None
	recognize_start_ms = None
	last_object_x = None
	last_object_y = None
	last_object_seen_ms = 0
	delivered_since_ms = None
	lost_since_ms = None
	final_sweep_start_ms = None
	final_sweep_last_yaw = None
	final_sweep_yaw_accum = 0.0
	final_sweep_done = False
	final_empty_since_ms = None

	plan_interval_ms = int(getattr(config, "LIDAR_PLAN_INTERVAL_MS", 260))
	print_interval_ms = int(getattr(config, "LIDAR_PLAN_PRINT_MS", 220))
	use_imu = bool(getattr(config, "UWB_TWO_ANCHOR_USE_IMU", False))
	scan_timeout_ms = int(getattr(config, "LIDAR_SCAN_TIMEOUT_MS", 260))
	rec_timeout_ms = int(getattr(config, "OPENART_MINI_TIMEOUT_MS", 900))
	car_link_send_ms = int(getattr(config, "CAR_LINK_SEND_MS", 100))
	field_w = float(getattr(config, "FIELD_WIDTH_M", 3.2))
	field_h = float(getattr(config, "FIELD_HEIGHT_M", 2.8))
	exit_margin = float(getattr(config, "FIELD_EXIT_MARGIN_M", 0.08))
	exit_confirm_ms = int(getattr(config, "MATCH_EXIT_CONFIRM_MS", 1200))
	empty_confirm_ms = int(getattr(config, "MISSION_EMPTY_CONFIRM_MS", 5000))
	transport_min_ms = int(getattr(config, "COOP_TRANSPORT_MIN_MS", 900))
	deliver_confirm_ms = int(getattr(config, "COOP_DELIVER_CONFIRM_MS", 350))
	lost_confirm_ms = int(getattr(config, "COOP_DELIVER_LOST_CONFIRM_MS", 650))
	blind_push_ms = int(getattr(config, "COOP_BLIND_PUSH_MAX_MS", 900))
	final_sweep_rot_speed = float(getattr(config, "MISSION_FINAL_SWEEP_ROT_SPEED", 10.0))
	final_sweep_min_deg = float(getattr(config, "MISSION_FINAL_SWEEP_MIN_DEG", 330.0))
	final_sweep_max_ms = int(getattr(config, "MISSION_FINAL_SWEEP_MAX_MS", 9000))
	final_empty_confirm_ms = int(getattr(config, "MISSION_FINAL_EMPTY_CONFIRM_MS", 5000))
	match_done_since_ms = None
	transport_start_ms = None

	try:
		while True:
			if not control_board.ticker.ready():
				time.sleep_ms(1)
				continue

			if use_imu:
				imu_data = control_board.read_imu()
			else:
				imu_data = {"gyro": None, "mag": None}
			encoder_data = control_board.read_encoder_speeds()
			pose = estimator.update(encoder_data, imu_data)

			uwb_receiver.update()
			fresh_ranges = uwb_solver.fresh_ranges(uwb_receiver)
			fix = uwb_solver.solve(pose, fresh_ranges)
			if fix:
				current_yaw = float(pose.get("yaw", fix.get("yaw", 0.0)))
				estimator.set_pose(fix["x"], fix["y"], fix["yaw"], current_absolute_yaw=current_yaw)
				pose = estimator.get_pose()
				pose["ts"] = int(fix.get("ts", time.ticks_ms()))

			lidar.read_frame()
			scan = lidar.get_latest_scan(timeout_ms=scan_timeout_ms)
			now_ms = time.ticks_ms()
			if scan:
				grid_data = planner.build(pose, scan)
				visible_tracks = tracker.update_from_targets(grid_data.get("targets"), now_ms)
			else:
				grid_data = None
				# 丢帧时用track的卡尔曼预测值维持
				for track in tracker.tracks:
					if "kf" in track:
						track["kf"].predict()
						kx, ky, kvx, kvy = track["kf"].get_state()
						track["x"] = kx
						track["y"] = ky
						track["vx"] = kvx
						track["vy"] = kvy
				visible_tracks = tracker.get_active_tracks()
			field_tracks = _filter_field_tracks(config, visible_tracks)

			if car_link:
				if car_link.uart and car_link.uart.any():
					car_link.receive()
				if time.ticks_diff(now_ms, last_sync_ms) >= car_link_send_ms:
					last_sync_ms = now_ms
					car_link.send(
						build_robot_status(
							"master",
							pose=pose,
							chassis=chassis,
							state=state,
							extra={
								"coop_push_active": bool(state == "transport_to_edge"),
								"target_visible": bool(field_tracks),
								"target_edge": current_edge,
								"match_done": bool(state == "match_done"),
								"active_target": current_track,
							},
						)
					)

			if state == "search_object":
				chassis.stop()
				if not field_tracks:
					current_track = None
					current_plan = None
					if match_done_since_ms is None:
						match_done_since_ms = now_ms
					elif time.ticks_diff(now_ms, match_done_since_ms) >= empty_confirm_ms:
						final_sweep_start_ms = now_ms
						final_sweep_last_yaw = float(pose.get("yaw", 0.0))
						final_sweep_yaw_accum = 0.0
						final_sweep_done = False
						final_empty_since_ms = None
						match_done_since_ms = None
						state = "final_sweep"
				else:
					match_done_since_ms = None
					# 优先用卡尔曼预测track
					current_track = _nearest_track_from_list(field_tracks, pose.get("x", 0.0), pose.get("y", 0.0))
					if current_track and scan and time.ticks_diff(now_ms, last_plan_ms) >= plan_interval_ms:
						last_plan_ms = now_ms
						current_plan = _plan_to_track(planner, pose, scan, current_track)
						if current_plan:
							state = "approach_object"

			elif state == "approach_object":
				if not field_tracks:
					current_plan = None
					current_track = None
					state = "search_object"
				elif not current_plan or (scan and time.ticks_diff(now_ms, last_plan_ms) >= plan_interval_ms):
					last_plan_ms = now_ms
					if current_track:
						current_track = _nearest_track_from_list(field_tracks, current_track.get("x", pose.get("x", 0.0)), current_track.get("y", pose.get("y", 0.0)))
					else:
						current_track = _nearest_track_from_list(field_tracks, pose.get("x", 0.0), pose.get("y", 0.0))
					current_plan = _plan_to_track(planner, pose, scan, current_track)
				if not current_plan:
					chassis.stop()
					state = "search_object"
				else:
					target = current_plan.get("target") or {}
					# 结合颜色辅助（如有color字段）
					# 可扩展：如target.get("color") == track.get("color")
					current_track = _nearest_track_from_list(field_tracks, target.get("center_x", pose.get("x", 0.0)), target.get("center_y", pose.get("y", 0.0)))
					cmd = follower.compute(pose, current_plan)
					display_proxy.last_vx = cmd["vx"]
					display_proxy.last_vy = cmd["vy"]
					if cmd.get("arrived"):
						chassis.stop()
						recognize_start_ms = now_ms
						state = "recognize"
					else:
						chassis.move(cmd["vx"], cmd["vy"], cmd["vw"])

			elif state == "recognize":
				chassis.stop()
				target = current_plan.get("target") if current_plan else None
				if target is None:
					state = "search_object"
				else:
					dist_object = _distance_m(
						pose.get("x", 0.0),
						pose.get("y", 0.0),
						target.get("center_x", 0.0),
						target.get("center_y", 0.0),
					)
					if dist_object > float(getattr(config, "OPENART_RECOGNIZE_DISTANCE_M", 0.42)):
						state = "approach_object"
					else:
						last_recognition = openart.get_latest(timeout_ms=rec_timeout_ms)
						if last_recognition is not None:
							current_edge = edge_target_from_label(config, last_recognition.get("label"))
							transport_start_ms = now_ms
							delivered_since_ms = None
							lost_since_ms = None
							state = "transport_to_edge"
						elif not bool(getattr(config, "OPENART_MINI_ENABLE", False)):
							current_edge = str(getattr(config, "OPENART_DEFAULT_EDGE", "right"))
							transport_start_ms = now_ms
							delivered_since_ms = None
							lost_since_ms = None
							state = "transport_to_edge"
						elif recognize_start_ms is not None and time.ticks_diff(now_ms, recognize_start_ms) >= rec_timeout_ms:
							current_edge = str(getattr(config, "OPENART_DEFAULT_EDGE", "right"))
							transport_start_ms = now_ms
							delivered_since_ms = None
							lost_since_ms = None
							state = "transport_to_edge"

			elif state == "transport_to_edge":
				if current_edge is None:
					state = "search_object"
				else:
					if current_track is not None:
						current_track = tracker.nearest_track(current_track.get("x", pose.get("x", 0.0)), current_track.get("y", pose.get("y", 0.0)))
					target_info = current_plan.get("target") if current_plan else {}
					object_x = target_info.get("center_x") if isinstance(target_info, dict) else None
					object_y = target_info.get("center_y") if isinstance(target_info, dict) else None
					if current_track:
						object_x = current_track.get("x", object_x)
						object_y = current_track.get("y", object_y)
						last_object_x = object_x
						last_object_y = object_y
						last_object_seen_ms = now_ms
						lost_since_ms = None
					elif last_object_x is not None and last_object_y is not None:
						object_x = last_object_x
						object_y = last_object_y
					goal_x, goal_y = exit_goal_for_edge(config, current_edge, object_x, object_y)
					if scan and time.ticks_diff(now_ms, last_plan_ms) >= plan_interval_ms:
						last_plan_ms = now_ms
						current_plan = planner.plan_to_world_goal(pose, scan, goal_x, goal_y, object_x=object_x, object_y=object_y)
					if current_track is None and lost_since_ms is None:
						lost_since_ms = now_ms
					lost_age_ms = time.ticks_diff(now_ms, lost_since_ms) if lost_since_ms is not None else 0
					allow_blind_push = current_track is not None or lost_age_ms <= blind_push_ms
					if current_plan and allow_blind_push:
						cmd = follower.compute(pose, current_plan)
						cmd["vx"] = max(float(getattr(config, "COOP_TRANSPORT_MIN_VX", 12.0)), cmd["vx"])
						chassis.move(cmd["vx"], cmd["vy"], cmd["vw"])
						display_proxy.last_vx = cmd["vx"]
						display_proxy.last_vy = cmd["vy"]
					elif allow_blind_push:
						chassis.move(float(getattr(config, "COOP_TRANSPORT_FALLBACK_VX", 16.0)), 0.0, 0.0)
						display_proxy.last_vx = float(getattr(config, "COOP_TRANSPORT_FALLBACK_VX", 16.0))
						display_proxy.last_vy = 0.0
					else:
						chassis.stop()

					delivered = False
					transport_elapsed = _transport_elapsed_ms(now_ms, transport_start_ms)
					object_known = object_x is not None and object_y is not None
					object_near_edge = object_known and _point_near_target_edge(config, current_edge, object_x, object_y)
					object_out_edge = object_known and _point_out_target_edge(config, current_edge, object_x, object_y, exit_margin)
					if object_out_edge and transport_elapsed >= transport_min_ms:
						if delivered_since_ms is None:
							delivered_since_ms = now_ms
						elif time.ticks_diff(now_ms, delivered_since_ms) >= deliver_confirm_ms:
							delivered = True
					else:
						delivered_since_ms = None
					if (
						not delivered
						and current_track is None
						and object_near_edge
						and transport_elapsed >= transport_min_ms
						and lost_since_ms is not None
						and time.ticks_diff(now_ms, lost_since_ms) >= lost_confirm_ms
					):
						delivered = True
					if not delivered and not allow_blind_push and not object_near_edge:
						current_track = None
						current_plan = None
						delivered_since_ms = None
						lost_since_ms = None
						state = "search_object"
					elif not delivered and transport_start_ms is not None and transport_elapsed >= int(getattr(config, "COOP_TRANSPORT_TIMEOUT_MS", 5000)) and object_near_edge:
						delivered = True

					if delivered:
						chassis.stop()
						if current_track is not None:
							tracker.mark_delivered(current_track.get("id"))
						current_track = None
						current_plan = None
						current_edge = None
						recognize_start_ms = None
						transport_start_ms = None
						delivered_since_ms = None
						lost_since_ms = None
						last_object_x = None
						last_object_y = None
						last_object_seen_ms = 0
						state = "search_object"

			elif state == "final_sweep":
				if field_tracks:
					chassis.stop()
					current_track = _nearest_track_from_list(field_tracks, pose.get("x", 0.0), pose.get("y", 0.0))
					current_plan = None
					final_sweep_start_ms = None
					final_sweep_last_yaw = None
					final_sweep_yaw_accum = 0.0
					final_sweep_done = False
					final_empty_since_ms = None
					state = "search_object"
				else:
					current_yaw = float(pose.get("yaw", 0.0))
					if final_sweep_last_yaw is None:
						final_sweep_last_yaw = current_yaw
					else:
						final_sweep_yaw_accum += abs(_angle_delta_deg(current_yaw, final_sweep_last_yaw))
						final_sweep_last_yaw = current_yaw

					sweep_elapsed = time.ticks_diff(now_ms, final_sweep_start_ms) if final_sweep_start_ms is not None else 0
					sweep_done = final_sweep_yaw_accum >= final_sweep_min_deg or sweep_elapsed >= final_sweep_max_ms
					if sweep_done and not final_sweep_done:
						final_sweep_done = True
						final_empty_since_ms = now_ms

					chassis.move(0.0, 0.0, final_sweep_rot_speed)
					display_proxy.last_vx = 0.0
					display_proxy.last_vy = 0.0
					if (
						final_sweep_done
						and final_empty_since_ms is not None
						and time.ticks_diff(now_ms, final_empty_since_ms) >= final_empty_confirm_ms
					):
						chassis.stop()
						state = "match_done"

			elif state == "match_done":
				chassis.stop()

			else:
				chassis.stop()
				state = "search_object"

			try:
				control_board.update_display(display_proxy if state == "follow_path" else chassis, pose)
			except Exception:
				pass

			if time.ticks_diff(now_ms, last_print_ms) >= print_interval_ms:
				last_print_ms = now_ms
				range_text = "{} {}".format(
					_format_range(int(getattr(config, "UWB_ANCHOR_0_ID", 0)), fresh_ranges),
					_format_range(int(getattr(config, "UWB_ANCHOR_1_ID", 1)), fresh_ranges),
				)
				last_rx_ms = getattr(uwb_receiver, "last_rx_ms", None)
				if last_rx_ms is None:
					uwb_diag = "uwb_rx=never"
				else:
					uwb_diag = "uwb_age={}ms".format(time.ticks_diff(now_ms, last_rx_ms))
				if not fresh_ranges:
					last_poll_cmd = getattr(uwb_receiver, "last_poll_cmd", None)
					last_line = getattr(uwb_receiver, "last_line", None)
					if last_poll_cmd:
						uwb_diag += ' poll="{}"'.format(str(last_poll_cmd).strip())
					if last_line:
						uwb_diag += ' raw="{}"'.format(str(last_line)[:48])
				if last_recognition:
					rec_text = "rec={} conf={:.2f}".format(
						str(last_recognition.get("label")),
						float(last_recognition.get("confidence", 0.0)),
					)
				else:
					rec_text = "rec=--"
				scan_count = int(scan.get("count", 0)) if isinstance(scan, dict) else 0
				print(
					"[GRID] state={} x={:.3f} y={:.3f} yaw={:.1f} scan_pts={} tracks={} field_tracks={} delivered={} edge={} {} {} {}".format(
						state,
						float(pose.get("x", 0.0)),
						float(pose.get("y", 0.0)),
						float(pose.get("yaw", 0.0)),
						scan_count,
						len(visible_tracks),
						len(field_tracks),
						int(getattr(tracker, "delivered_count", 0)),
						str(current_edge),
						range_text,
						uwb_diag,
						_short_path_text(current_plan),
						rec_text,
					)
				)
	finally:
		chassis.stop()


if __name__ == "__main__":
	main()
