"""主从联动底盘。"""

import time
import math


def clamp(value, low, high):
    return max(low, min(high, value))


class LinkedChassis:
    def __init__(self, local_chassis, link, config, pose_provider=None):
        self.local = local_chassis
        self.link = link
        self.cfg = config
        self.pose_provider = pose_provider
        self.seq = 0
        self.last_send_ms = None
        self.last_status = None
        self.last_status_ms = None
        self.started_ms = time.ticks_ms()
        self.last_ack_seq = -1

    def _get_local_pose(self):
        provider = self.pose_provider
        if callable(provider):
            try:
                return provider()
            except Exception:
                return None
        return None

    def _formation_correction(self, state):
        if state != "push":
            return 0.0
        if not bool(getattr(self.cfg, "COOP_FORMATION_ENABLE", False)):
            return 0.0

        local_pose = self._get_local_pose()
        remote_pose = None
        if self.last_status:
            remote_pose = self.last_status.get("status", {}).get("pose")
        if not local_pose or not remote_pose:
            return 0.0

        try:
            mx = float(local_pose.get("x", 0.0))
            my = float(local_pose.get("y", 0.0))
            myaw = float(local_pose.get("yaw", 0.0))
            sx = float(remote_pose.get("x", 0.0))
            sy = float(remote_pose.get("y", 0.0))
        except Exception:
            return 0.0

        dx = sx - mx
        dy = sy - my
        yaw_rad = myaw * math.pi / 180.0
        # Convert global delta into master body frame.
        lateral = (-dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad))

        desired = float(getattr(self.cfg, "COOP_FORMATION_LATERAL_OFFSET_M", 0.0))
        err = lateral - desired
        kp = float(getattr(self.cfg, "COOP_FORMATION_KP_VY", 0.0))
        corr = -kp * err
        max_vy = float(getattr(self.cfg, "COOP_FORMATION_MAX_VY", getattr(self.cfg, "ASSIST_SAFE_LATERAL_LIMIT", 15)))
        corr = clamp(corr, -max_vy, max_vy)
        return corr

    def update_link(self):
        while True:
            status = self.link.read_status()
            if status:
                self.last_status = status
                self.last_status_ms = time.ticks_ms()
                self.last_ack_seq = status.get("ack", self.last_ack_seq)
                continue

            heartbeat = self.link.read_heartbeat()
            if heartbeat:
                self.last_status_ms = time.ticks_ms()
                self.last_ack_seq = heartbeat.get("ack", self.last_ack_seq)
                continue
            break

    def is_remote_ready(self):
        self.update_link()

        if not getattr(self.cfg, "DUAL_REQUIRE_SLAVE_READY", False):
            return True

        if self.last_status_ms is None:
            return time.ticks_diff(time.ticks_ms(), self.started_ms) < self.cfg.CAR_LINK_STARTUP_GRACE_MS

        return time.ticks_diff(time.ticks_ms(), self.last_status_ms) <= self.cfg.CAR_LINK_STATUS_TIMEOUT_MS

    def _send_remote(self, vx, vy, vw, state="run", force=False):
        self.update_link()
        now = time.ticks_ms()
        should_send = force or self.last_send_ms is None
        if not should_send:
            should_send = time.ticks_diff(now, self.last_send_ms) >= self.cfg.CAR_LINK_SEND_MS

        if not should_send:
            return

        remote_vx = vx * self.cfg.SLAVE_SPEED_SCALE
        remote_vy = vy * self.cfg.SLAVE_SPEED_SCALE
        remote_vw = vw * self.cfg.SLAVE_ROTATE_SCALE

        if state == "push" and remote_vx > 0:
            remote_vy += self.cfg.SLAVE_LATERAL_BIAS
            remote_vy += self._formation_correction(state)

        self.link.send_command(remote_vx, remote_vy, remote_vw, state=state, seq=self.seq)
        self.seq += 1
        self.last_send_ms = now

    def send_heartbeat(self, state="run"):
        self.update_link()
        self.link.send_heartbeat(seq=self.seq, state=state)

    def send_remote_only(self, vx, vy, vw, state="run", force=False):
        self._send_remote(vx, vy, vw, state=state, force=force)

    def move(self, vx, vy, vw):
        if not self.is_remote_ready():
            self.local.stop()
            return

        self.local.move(vx, vy, vw)
        state = "push" if vx >= self.cfg.PUSH_SPEED * 0.8 else "run"
        self._send_remote(vx, vy, vw, state=state)

    def forward(self, speed):
        self.move(speed, 0, 0)

    def strafe(self, speed):
        self.move(0, speed, 0)

    def rotate(self, speed):
        self.move(0, 0, speed)

    def stop(self):
        self.local.stop()
        self._send_remote(0, 0, 0, state="stop", force=True)
