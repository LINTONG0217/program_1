"""Runtime orchestration."""

import time

from Module.pose_estimation import PoseEstimator


class DriveLayer:
	def __init__(self, chassis, control_board=None):
		self.chassis = chassis
		self.control_board = control_board
		if control_board and hasattr(chassis, "attach_feedback"):
			chassis.attach_feedback(control_board.encoders, control_board.imu)

	def safety_tick(self):
		if hasattr(self.chassis, "check_watchdog"):
			self.chassis.check_watchdog()


class EstimationLayer:
	def __init__(self, config, control_board=None):
		self.control_board = control_board
		self.estimator = PoseEstimator(config)
		if getattr(config, "POSE_INIT_ENABLE", False):
			self.estimator.request_pose_init(
				getattr(config, "POSE_INIT_X", 0.0),
				getattr(config, "POSE_INIT_Y", 0.0),
				getattr(config, "POSE_INIT_YAW_DEG", 0.0),
			)
		self.last_pose = self.estimator.get_pose()

	def update(self, chassis):
		imu_data = self.control_board.read_imu() if self.control_board else {"gyro": None, "mag": None}
		encoder_data = self.control_board.read_encoder_speeds() if self.control_board else {}
		if not encoder_data and hasattr(chassis, "get_wheel_feedback"):
			encoder_data = chassis.get_wheel_feedback()
		self.last_pose = self.estimator.update(encoder_data, imu_data)
		return self.last_pose


class PerceptionLayer:
	def __init__(self, vision=None, obstacle_sensor=None, ultrasonic=None):
		self.vision = vision
		self.obstacle_sensor = obstacle_sensor
		self.ultrasonic = ultrasonic


class TaskLayer:
	def __init__(self, controller):
		self.controller = controller

	def step(self):
		self.controller.update()


class DisplayLayer:
	def __init__(self, control_board=None):
		self.control_board = control_board

	def update(self, pose, chassis):
		if self.control_board:
			self.control_board.update_display(chassis, pose)


class CommunicationLayer:
	def __init__(self, linked_chassis=None, link=None, status_provider=None):
		self.linked_chassis = linked_chassis
		self.link = link
		self.status_provider = status_provider
		self.last_heartbeat_ms = time.ticks_ms()
		self.last_status_ms = None
		self.status_seq = 0

	def master_ready(self):
		if not self.linked_chassis:
			return True
		return self.linked_chassis.is_remote_ready()

	def tick(self, state="run"):
		if not self.linked_chassis:
			return

		now = time.ticks_ms()
		interval = getattr(self.linked_chassis.cfg, "CAR_LINK_HEARTBEAT_MS", 100)
		if time.ticks_diff(now, self.last_heartbeat_ms) >= interval:
			self.linked_chassis.send_heartbeat(state=state)
			self.last_heartbeat_ms = now

		provider = self.status_provider
		if callable(provider) and self.link and hasattr(self.link, "send_status"):
			status_interval = int(getattr(self.linked_chassis.cfg, "CAR_LINK_MASTER_STATUS_MS", 0) or 0)
			if status_interval > 0:
				should_send = self.last_status_ms is None or time.ticks_diff(now, self.last_status_ms) >= status_interval
				if should_send:
					try:
						status = provider()
						if status:
							self.link.send_status(state=state, seq=self.status_seq, status=status, online=True)
							self.status_seq = (self.status_seq + 1) & 0xFFFF
							self.last_status_ms = now
					except Exception:
							pass


class RobotSystem:
	def __init__(self, control_board, drive_layer, estimation_layer, task_layer, display_layer=None, communication_layer=None):
		self.control_board = control_board
		self.drive = drive_layer
		self.estimation = estimation_layer
		self.task = task_layer
		self.display = display_layer
		self.communication = communication_layer

	def step(self):
		if self.communication:
			self.communication.tick()
		if self.communication and not self.communication.master_ready():
			self.drive.chassis.stop()
			return None

		self.task.step()
		pose = self.estimation.update(self.drive.chassis)
		self.drive.safety_tick()
		if self.display:
			self.display.update(pose, self.drive.chassis)
		return pose

	def run_forever(self):
		while True:
			if self.control_board and self.control_board.ticker.ready():
				self.step()
			elif hasattr(time, "sleep_ms"):
				time.sleep_ms(1)
