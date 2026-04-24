import math
import time

from Module.task_controller_basic import clamp


def normalize_angle_deg(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def body_to_world(yaw_deg, forward_m, lateral_m):
    rad = math.radians(yaw_deg)
    cos_a = math.cos(rad)
    sin_a = math.sin(rad)
    x = forward_m * cos_a - lateral_m * sin_a
    y = forward_m * sin_a + lateral_m * cos_a
    return x, y


def world_to_body(yaw_deg, dx_world, dy_world):
    rad = math.radians(yaw_deg)
    cos_a = math.cos(rad)
    sin_a = math.sin(rad)
    x_body = dx_world * cos_a + dy_world * sin_a
    y_body = -dx_world * sin_a + dy_world * cos_a
    return x_body, y_body


def _copy_pose(pose):
    if not isinstance(pose, dict):
        return None
    return {
        "x": float(pose.get("x", 0.0)),
        "y": float(pose.get("y", 0.0)),
        "yaw": float(pose.get("yaw", 0.0)),
        "ts": int(pose.get("ts", time.ticks_ms())),
    }


def build_robot_status(role, pose=None, chassis=None, state=None, extra=None):
    payload = {
        "role": str(role or ""),
        "ts": time.ticks_ms(),
    }
    if state is not None:
        payload["state"] = state
    copied_pose = _copy_pose(pose)
    if copied_pose:
        payload["pose"] = copied_pose
    if chassis is not None:
        payload["cmd"] = {
            "vx": float(getattr(chassis, "last_vx", 0.0)),
            "vy": float(getattr(chassis, "last_vy", 0.0)),
            "vw": float(getattr(chassis, "last_vw", 0.0)),
        }
    if isinstance(extra, dict):
        payload.update(extra)
    return payload


class SlaveSideFollower:
    STATE_WAIT_MASTER = "wait_master"
    STATE_TRACK_SIDE = "track_side"

    def __init__(self, chassis, link, config, pose_provider=None):
        self.chassis = chassis
        self.link = link
        self.cfg = config
        self.pose_provider = pose_provider
        self.state = self.STATE_WAIT_MASTER
        self.last_master = None
        self.last_target_pose = None
        self.last_command = {"vx": 0.0, "vy": 0.0, "vw": 0.0}

    def _set_state(self, new_state):
        if self.state != new_state:
            print("slave state ->", new_state)
            self.state = new_state

    def _poll_link(self):
        if not self.link:
            return None
        latest = None
        for _ in range(4):
            data = self.link.receive()
            if not data:
                break
            latest = data
        merged = self.link.get_remote_data() if self.link else None
        if isinstance(merged, dict) and merged:
            self.last_master = merged
        return latest

    def _get_pose(self):
        if callable(self.pose_provider):
            try:
                return self.pose_provider()
            except Exception:
                return None
        return None

    def _master_pose(self):
        if not isinstance(self.last_master, dict):
            return None
        pose = self.last_master.get("pose")
        if not isinstance(pose, dict):
            return None
        max_age = int(getattr(self.cfg, "SLAVE_MAX_MASTER_AGE_MS", 400))
        ts = int(self.last_master.get("ts", pose.get("ts", 0)) or 0)
        if ts > 0 and time.ticks_diff(time.ticks_ms(), ts) > max_age:
            return None
        return pose

    def _target_pose(self, master_pose):
        payload = self.last_master if isinstance(self.last_master, dict) else {}
        coop_push = bool(payload.get("coop_push_active", False))
        if coop_push:
            lateral = float(getattr(self.cfg, "COOP_PUSH_LATERAL_OFFSET_M", getattr(self.cfg, "COOP_FORMATION_LATERAL_OFFSET_M", 0.35)))
            forward = float(getattr(self.cfg, "COOP_PUSH_FORWARD_OFFSET_M", getattr(self.cfg, "COOP_FORMATION_FORWARD_OFFSET_M", 0.0)))
        else:
            lateral = float(getattr(self.cfg, "COOP_FORMATION_LATERAL_OFFSET_M", 0.35))
            forward = float(getattr(self.cfg, "COOP_FORMATION_FORWARD_OFFSET_M", 0.0))
        dx, dy = body_to_world(float(master_pose.get("yaw", 0.0)), forward, lateral)
        target = {
            "x": float(master_pose.get("x", 0.0)) + dx,
            "y": float(master_pose.get("y", 0.0)) + dy,
            "yaw": float(master_pose.get("yaw", 0.0)),
            "ts": time.ticks_ms(),
        }
        self.last_target_pose = target
        return target

    def _current_master_side(self, master_pose, local_pose):
        dx = float(local_pose.get("x", 0.0)) - float(master_pose.get("x", 0.0))
        dy = float(local_pose.get("y", 0.0)) - float(master_pose.get("y", 0.0))
        _, lateral = world_to_body(float(master_pose.get("yaw", 0.0)), dx, dy)
        return lateral

    def _compose_command(self, target_pose, local_pose, master_payload):
        ex_world = float(target_pose.get("x", 0.0)) - float(local_pose.get("x", 0.0))
        ey_world = float(target_pose.get("y", 0.0)) - float(local_pose.get("y", 0.0))
        local_yaw = float(local_pose.get("yaw", 0.0))
        ex_body, ey_body = world_to_body(local_yaw, ex_world, ey_world)
        yaw_err = normalize_angle_deg(float(target_pose.get("yaw", 0.0)) - local_yaw)

        pos_deadband = float(getattr(self.cfg, "SLAVE_TARGET_DEADBAND_M", 0.03))
        yaw_deadband = float(getattr(self.cfg, "SLAVE_YAW_DEADBAND_DEG", 4.0))
        if abs(ex_body) < pos_deadband:
            ex_body = 0.0
        if abs(ey_body) < pos_deadband:
            ey_body = 0.0
        if abs(yaw_err) < yaw_deadband:
            yaw_err = 0.0

        kp_vx = float(getattr(self.cfg, "SLAVE_POSITION_KP_VX", 90.0))
        kp_vy = float(getattr(self.cfg, "COOP_FORMATION_KP_VY", 80.0))
        kp_vw = float(getattr(self.cfg, "SLAVE_YAW_KP", 1.3))

        cmd = master_payload.get("cmd") if isinstance(master_payload, dict) else {}
        ff_scale = float(getattr(self.cfg, "SLAVE_FEEDFORWARD_SCALE", 0.85))
        vx = ex_body * kp_vx + float(cmd.get("vx", 0.0)) * ff_scale
        vy = ey_body * kp_vy + float(cmd.get("vy", 0.0)) * ff_scale
        vw = yaw_err * kp_vw + float(cmd.get("vw", 0.0)) * float(getattr(self.cfg, "SLAVE_ROTATE_SCALE", 1.0)) * 0.35

        max_vx = float(getattr(self.cfg, "SLAVE_FOLLOW_MAX_VX", 35.0))
        max_vx_forward = max_vx
        max_vy = float(getattr(self.cfg, "SLAVE_FOLLOW_MAX_VY", 35.0))
        max_vw = float(getattr(self.cfg, "SLAVE_FOLLOW_MAX_VW", 28.0))

        coop_push = bool(master_payload.get("coop_push_active", False))
        if coop_push and bool(getattr(self.cfg, "COOP_PUSH_ENABLE", True)):
            max_vx_forward = float(getattr(self.cfg, "COOP_SLAVE_PUSH_ALLOW_VX_MAX", max_vx_forward))

        vx = clamp(vx, -max_vx, max_vx_forward)
        vy = clamp(vy, -max_vy, max_vy)
        vw = clamp(vw, -max_vw, max_vw)

        if coop_push and bool(getattr(self.cfg, "SLAVE_PUSH_REQUIRE_BIAS_SIDE", True)):
            desired_side = float(getattr(self.cfg, "COOP_FORMATION_LATERAL_OFFSET_M", 0.35))
            current_side = self._current_master_side(master_payload["pose"], local_pose)
            if desired_side > 0 and current_side < 0:
                vx = min(vx, 0)
                vy = abs(vy)
            elif desired_side < 0 and current_side > 0:
                vx = min(vx, 0)
                vy = -abs(vy)

        self.last_command = {"vx": vx, "vy": vy, "vw": vw}
        return self.last_command

    def update(self, local_pose=None):
        self._poll_link()
        if local_pose is None:
            local_pose = self._get_pose()
        master_pose = self._master_pose()

        if not master_pose or not local_pose:
            self._set_state(self.STATE_WAIT_MASTER)
            self.chassis.stop()
            return None

        target_pose = self._target_pose(master_pose)
        cmd = self._compose_command(target_pose, local_pose, self.last_master or {})
        self.chassis.move(cmd["vx"], cmd["vy"], cmd["vw"])
        self._set_state(self.STATE_TRACK_SIDE)
        return {
            "state": self.state,
            "target_pose": target_pose,
            "master_pose": master_pose,
            "command": dict(cmd),
        }
