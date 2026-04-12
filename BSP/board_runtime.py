"""板级控制运行时。"""

import time

try:
	import lcd  # type: ignore
except ImportError:
	lcd = None
from BSP.device_adapters import EncoderAdapter, IMUAdapter
from BSP.grid_display import GridMap
from machine import Pin
from Module import config


try:
	from seekfree import IMU963RX  # type: ignore
except ImportError:
	try:
		from imu import IMU963RX  # type: ignore
	except ImportError:
		IMU963RX = None

try:
	from smartcar import encoder as HardwareEncoder  # type: ignore
except ImportError:
	try:
		from encoder import Encoder as HardwareEncoder  # type: ignore
	except ImportError:
		HardwareEncoder = None

try:
	from smartcar import ticker as HardwareTicker  # type: ignore
except ImportError:
	try:
		from ticker import ticker as HardwareTicker  # type: ignore
	except ImportError:
		HardwareTicker = None

try:
	from display import LCD_Drv, LCD  # type: ignore
except ImportError:
	LCD_Drv = None
	LCD = None


def clamp(value, low, high):
	return max(low, min(high, value))


class NavigationKey:
	def __init__(self, pin_name):
		self.pin = Pin(pin_name, Pin.IN)
		pull_up = getattr(Pin, "PULL_UP_47K", None)
		if pull_up is not None:
			try:
				self.pin = Pin(pin_name, Pin.IN, pull_up)
			except Exception:
				self.pin = Pin(pin_name, Pin.IN)

	def is_pressed(self):
		return bool(self.pin.value() == 0)


class IMUDevice:
	def __init__(self, cfg):
		self.cfg = cfg
		self.imu = IMU963RX() if IMU963RX else None
		self.adapter = IMUAdapter(self.imu)
		self.gyro_offset = 0.0

	def calibrate(self):
		if self.imu is None:
			return

		warmup = getattr(self.cfg, "GYRO_CALIBRATION_WARMUP", 20)
		sample_count = getattr(self.cfg, "GYRO_CALIBRATION_SAMPLES", 100)
		total = 0.0

		for _ in range(warmup):
			self._read_raw()
		for _ in range(sample_count):
			raw = self._read_raw()
			total += raw[5] if raw and len(raw) > 5 else 0.0
			time.sleep_ms(10) if hasattr(time, "sleep_ms") else time.sleep(0.01)

		self.gyro_offset = total / max(1, sample_count)

	def _read_raw(self):
		if self.imu is None:
			return None

		reader = getattr(self.imu, "read", None)
		if callable(reader):
			try:
				reader()
			except Exception:
				pass

		getter = getattr(self.imu, "get", None)
		if callable(getter):
			try:
				return getter()
			except Exception:
				return None
		return None

	def read(self):
		raw = self._read_raw()
		if raw and len(raw) > 7:
			gz = (raw[5] - self.gyro_offset) * getattr(self.cfg, "GYRO_RAW_TO_DPS", 0.07)
			hx = raw[6] * getattr(self.cfg, "MAG_RAW_TO_UNIT", 0.0003333333)
			hy = raw[7] * getattr(self.cfg, "MAG_RAW_TO_UNIT", 0.0003333333)
			return {"gyro": (0.0, 0.0, gz), "mag": (hx, hy, 0.0), "ok": True, "raw": raw}

		data = self.adapter.read(
			gyro_scale=getattr(self.cfg, "IMU_GYRO_SCALE", 1.0),
			mag_scale=getattr(self.cfg, "IMU_MAG_SCALE", 1.0),
		)
		return {"gyro": data.get("gyro"), "mag": data.get("mag"), "ok": data.get("ok", False), "raw": raw}


class Encoder:
	def __init__(self, pin_a, pin_b, reverse=False):
		self.pin_a = pin_a
		self.pin_b = pin_b
		self.reverse = reverse
		self.count = 0
		self.last_count = 0
		self.last_speed = 0.0
		self.last_ms = time.ticks_ms()
		self.hw = None
		if HardwareEncoder:
			try:
				self.hw = HardwareEncoder(pin_a, pin_b)
			except TypeError:
				try:
					self.hw = HardwareEncoder((pin_a, pin_b))
				except Exception:
					self.hw = None
			except Exception:
				self.hw = None
		self.adapter = EncoderAdapter(self.hw, reverse=reverse)
		self.available = self.hw is not None

	def _raw_count(self):
		if self.hw is None:
			return self.count
		value = self.adapter.read_count()
		return self.count if value is None else value

	def update(self):
		now = time.ticks_ms()
		count = self._raw_count()
		dt_ms = max(1, time.ticks_diff(now, self.last_ms))
		self.last_speed = (count - self.last_count) * 1000.0 / dt_ms
		self.count = count
		self.last_count = count
		self.last_ms = now
		return count

	def read(self):
		return self.update()

	def read_speed(self):
		self.update()
		return self.last_speed

	def snapshot(self):
		self.update()
		return {
			"count": self.count,
			"speed": self.last_speed,
			"available": self.available,
		}


class ControlTicker:
	def __init__(self, ticker_id, period_ms, sources=None):
		self.ticker_id = ticker_id
		self.period_ms = period_ms
		self.last_ms = time.ticks_ms()
		self.hw = None
		self.flag = False
		self.sources = sources or []
		if HardwareTicker:
			try:
				self.hw = HardwareTicker(ticker_id)
			except TypeError:
				try:
					self.hw = HardwareTicker()
				except Exception:
					self.hw = None
			except Exception:
				self.hw = None
		if self.hw is not None:
			capture_list = getattr(self.hw, "capture_list", None)
			callback = getattr(self.hw, "callback", None)
			start = getattr(self.hw, "start", None)
			if callable(capture_list) and self.sources:
				try:
					capture_list(*self.sources)
				except Exception:
					pass
			if callable(callback):
				try:
					callback(self._on_tick)
				except Exception:
					pass
			if callable(start):
				try:
					start(period_ms)
				except Exception:
					self.hw = None

	def _on_tick(self, *args, **kwargs):
		self.flag = True

	def ready(self):
		if self.hw is not None and self.flag:
			self.flag = False
			return True

		now = time.ticks_ms()
		if time.ticks_diff(now, self.last_ms) >= self.period_ms:
			self.last_ms = now
			return True
		return False


class LCDGridDisplay:
	def __init__(self, cfg):
		self.cfg = cfg
		self.width = cfg.LCD_WIDTH
		self.height = cfg.LCD_HEIGHT
		self.margin = cfg.LCD_GRID_MARGIN
		self.cols = cfg.LCD_GRID_COLS
		self.rows = cfg.LCD_GRID_ROWS
		self.cell_w = max(1, (self.width - self.margin * 2) // self.cols)
		self.cell_h = max(1, (self.height - self.margin * 2) // self.rows)
		self.grid_left = self.margin
		self.grid_top = self.margin
		self.grid_right = self.grid_left + self.cell_w * self.cols
		self.grid_bottom = self.grid_top + self.cell_h * self.rows
		self.car_col = self.cols // 2
		self.car_row = self.rows // 2
		self.last_ms = time.ticks_ms()
		self.lcd_dev = None
		self.map = None

	def _init_seekfree_lcd(self):
		if not LCD_Drv or not LCD:
			return False
		try:
			cs_lcd = Pin(self.cfg.LCD_CS_PIN, Pin.OUT)
			cs_lcd.value(1)
			cs_lcd.value(0)
			rst_lcd = Pin(self.cfg.LCD_RST_PIN, Pin.OUT)
			dc_lcd = Pin(self.cfg.LCD_DC_PIN, Pin.OUT)
			drv = LCD_Drv(
				SPI_INDEX=self.cfg.LCD_SPI_INDEX,
				BAUDRATE=60000000,
				DC_PIN=dc_lcd,
				RST_PIN=rst_lcd,
				LCD_TYPE=getattr(LCD_Drv, "LCD200_TYPE", 0),
			)
			self.lcd_dev = LCD(drv)
			mode = getattr(self.lcd_dev, "mode", None)
			if callable(mode):
				mode(2)
			return True
		except Exception:
			self.lcd_dev = None
			return False

	def init(self):
		if self._init_seekfree_lcd():
			self.map = GridMap(self.lcd_dev)
			self.map.draw_bg()
			return
		if hasattr(lcd, "init"):
			try:
				lcd.init()
			except TypeError:
				lcd.init()
		if hasattr(lcd, "clear") and hasattr(lcd, "color") and hasattr(lcd, "line"):
			self.map = GridMap(lcd)
			self.map.draw_bg()
		else:
			self.clear()
			self.draw_grid()
			self.draw_car(self.car_col, self.car_row)

	def clear(self):
		if self.lcd_dev is not None and hasattr(self.lcd_dev, "clear"):
			self.lcd_dev.clear(self.cfg.LCD_BG_COLOR)
			return
		if hasattr(lcd, "clear"):
			lcd.clear(self.cfg.LCD_BG_COLOR)

	def _set_color(self, color):
		if self.lcd_dev is not None and hasattr(self.lcd_dev, "color"):
			self.lcd_dev.color(color)
			return
		if hasattr(lcd, "color"):
			lcd.color(color)

	def _line(self, x1, y1, x2, y2, color):
		self._set_color(color)
		if self.lcd_dev is not None and hasattr(self.lcd_dev, "line"):
			self.lcd_dev.line(int(x1), int(y1), int(x2), int(y2))
			return
		if hasattr(lcd, "line"):
			lcd.line(int(x1), int(y1), int(x2), int(y2))

	def draw_grid(self):
		for col in range(self.cols + 1):
			x = self.grid_left + col * self.cell_w
			self._line(x, self.grid_top, x, self.grid_bottom, self.cfg.LCD_GRID_COLOR)
		for row in range(self.rows + 1):
			y = self.grid_top + row * self.cell_h
			self._line(self.grid_left, y, self.grid_right, y, self.cfg.LCD_GRID_COLOR)

	def draw_car(self, col, row):
		center_x = self.grid_left + col * self.cell_w + self.cell_w // 2
		center_y = self.grid_top + row * self.cell_h + self.cell_h // 2
		radius_x = max(2, self.cell_w // 3)
		radius_y = max(2, self.cell_h // 3)

		self._line(center_x - radius_x, center_y, center_x + radius_x, center_y, self.cfg.LCD_CAR_COLOR)
		self._line(center_x, center_y - radius_y, center_x, center_y + radius_y, self.cfg.LCD_CAR_COLOR)

	def update(self, chassis, pose=None):
		if self.map is not None:
			if pose:
				self.map.update_car(pose.get("x", 0.0), pose.get("y", 0.0))
			else:
				now = time.ticks_ms()
				dt_ms = max(1, time.ticks_diff(now, self.last_ms))
				self.last_ms = now
				est_x = chassis.last_vx * dt_ms * self.cfg.LCD_SPEED_TO_CELL
				est_y = chassis.last_vy * dt_ms * self.cfg.LCD_SPEED_TO_CELL
				self.map.update_car(est_x, est_y)
			return

		now = time.ticks_ms()
		dt_ms = max(1, time.ticks_diff(now, self.last_ms))
		self.last_ms = now

		if pose:
			dx = pose.get("y", 0.0) / max(self.cfg.ODOM_LINEAR_SCALE, 1e-6)
			dy = pose.get("x", 0.0) / max(self.cfg.ODOM_LINEAR_SCALE, 1e-6)
		else:
			dx = chassis.last_vy * dt_ms * self.cfg.LCD_SPEED_TO_CELL
			dy = chassis.last_vx * dt_ms * self.cfg.LCD_SPEED_TO_CELL

		self.car_col = int(clamp(round(self.cols // 2 + dx), 0, self.cols - 1))
		self.car_row = int(clamp(round(self.rows // 2 - dy), 0, self.rows - 1))

		self.clear()
		self.draw_grid()
		self.draw_car(self.car_col, self.car_row)


class MainControl:
	def __init__(self, cfg=config):
		self.cfg = cfg
		self.nav_key = NavigationKey(cfg.NAV_KEY_PIN)
		self.imu = IMUDevice(cfg)
		self.encoders = {
			"fl": Encoder(*cfg.ENCODER_FL_PINS, reverse=cfg.ENCODER_REVERSE.get("fl", False)),
			"fr": Encoder(*cfg.ENCODER_FR_PINS, reverse=cfg.ENCODER_REVERSE.get("fr", False)),
			"bl": Encoder(*cfg.ENCODER_BL_PINS, reverse=cfg.ENCODER_REVERSE.get("bl", False)),
			"br": Encoder(*cfg.ENCODER_BR_PINS, reverse=cfg.ENCODER_REVERSE.get("br", False)),
		}
		capture_sources = [self.imu.imu]
		for name in ("fl", "fr", "bl", "br"):
			capture_sources.append(self.encoders[name].hw)
		self.ticker = ControlTicker(cfg.CONTROL_TICKER_ID, cfg.CONTROL_PERIOD_MS, sources=[item for item in capture_sources if item is not None])
		self.lcd = LCDGridDisplay(cfg)

	def init(self):
		# NOTE: Board init can take a while (IMU calibration + LCD init).
		# Add explicit progress prints so it doesn't look like a hang.
		if bool(getattr(self.cfg, "IMU_CALIBRATION_ENABLE", True)):
			print("board: imu calibrate...")
			self.imu.calibrate()
			print("board: imu calibrate done")
		else:
			print("board: imu calibrate skipped")

		if bool(getattr(self.cfg, "LCD_ENABLE", True)):
			print("board: lcd init...")
			self.lcd.init()
			print("board: lcd init done")
		else:
			print("board: lcd init skipped")

	def read_imu(self):
		return self.imu.read()

	def read_encoder_snapshot(self):
		return {name: encoder.snapshot() for name, encoder in self.encoders.items()}

	def read_encoder_speeds(self):
		speeds = {}
		for name, encoder in self.encoders.items():
			data = encoder.snapshot()
			if data["available"]:
				speeds[name] = data["speed"]
		return speeds

	def update_display(self, chassis, pose=None):
		self.lcd.update(chassis, pose)
