"""开机全局位姿初始化。"""

import math


CORNER_WORLD_MAP = {
	"BL": lambda cfg: (0.0, 0.0),
	"BR": lambda cfg: (float(getattr(cfg, "FIELD_SIZE_X_M", 3.2)), 0.0),
	"TL": lambda cfg: (0.0, float(getattr(cfg, "FIELD_SIZE_Y_M", 2.8))),
	"TR": lambda cfg: (
		float(getattr(cfg, "FIELD_SIZE_X_M", 3.2)),
		float(getattr(cfg, "FIELD_SIZE_Y_M", 2.8)),
	),
}


def _world_corner_xy(cfg, corner_id):
	cid = str(corner_id or "BL").upper()
	getter = CORNER_WORLD_MAP.get(cid, CORNER_WORLD_MAP["BL"])
	return getter(cfg)


def estimate_pose_from_openart_corner(frame, cfg, yaw_deg):
	if not frame:
		return None

	corner = frame.get("field_corner")
	info = frame.get("frame") or {}
	if not corner or not info:
		return None

	fw = float(info.get("w", 0) or 0)
	fh = float(info.get("h", 0) or 0)
	if fw <= 0 or fh <= 0:
		return None

	px = float(corner.get("x", fw / 2.0))
	py = float(corner.get("y", fh / 2.0))
	cx = fw / 2.0
	cy = fh / 2.0

	mpp_x = float(getattr(cfg, "OPENART_PIXEL_TO_M_X", 0.01))
	mpp_y = float(getattr(cfg, "OPENART_PIXEL_TO_M_Y", 0.01))
	sign_x = float(getattr(cfg, "OPENART_PIXEL_TO_ROBOT_X_SIGN", 1.0))
	sign_y = float(getattr(cfg, "OPENART_PIXEL_TO_ROBOT_Y_SIGN", 1.0))

	corner_from_cam_x = (px - cx) * mpp_x * sign_x
	corner_from_cam_y = (py - cy) * mpp_y * sign_y
	cam_to_robot_x = float(getattr(cfg, "OPENART_CAM_TO_ROBOT_X_M", 0.0))
	cam_to_robot_y = float(getattr(cfg, "OPENART_CAM_TO_ROBOT_Y_M", 0.0))
	corner_from_robot_x = corner_from_cam_x - cam_to_robot_x
	corner_from_robot_y = corner_from_cam_y - cam_to_robot_y

	world_corner_id = getattr(cfg, "OPENART_VISIBLE_WORLD_CORNER", "BL")
	corner_wx, corner_wy = _world_corner_xy(cfg, world_corner_id)

	yaw = float(yaw_deg)
	rad = math.radians(yaw)
	cos_a = math.cos(rad)
	sin_a = math.sin(rad)
	dx_world = corner_from_robot_x * cos_a - corner_from_robot_y * sin_a
	dy_world = corner_from_robot_x * sin_a + corner_from_robot_y * cos_a
	robot_x = corner_wx - dx_world
	robot_y = corner_wy - dy_world

	if bool(getattr(cfg, "OPENART_GLOBAL_CLAMP_TO_FIELD", True)):
		field_x = float(getattr(cfg, "FIELD_SIZE_X_M", 3.2))
		field_y = float(getattr(cfg, "FIELD_SIZE_Y_M", 2.8))
		robot_x = max(0.0, min(field_x, robot_x))
		robot_y = max(0.0, min(field_y, robot_y))

	return {
		"x": robot_x,
		"y": robot_y,
		"yaw": yaw,
		"corner_px": (px, py),
		"corner_world": (corner_wx, corner_wy),
		"world_corner_id": str(world_corner_id),
	}
