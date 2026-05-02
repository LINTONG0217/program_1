"""硬件适配器。"""


def _try_call(obj, names, *args, **kwargs):
	for name in names:
		member = getattr(obj, name, None)
		if callable(member):
			try:
				return member(*args, **kwargs)
			except TypeError:
				continue
			except Exception:
				return None
	return None


def _try_attr(obj, names):
	for name in names:
		if hasattr(obj, name):
			return getattr(obj, name)
	return None


class IMUAdapter:
	def __init__(self, imu):
		self.imu = imu

	def _normalize_vec(self, value, scale=1.0):
		if value is None:
			return None
		if isinstance(value, dict):
			for keys in (("x", "y", "z"), ("gx", "gy", "gz"), ("mx", "my", "mz")):
				if all(key in value for key in keys):
					return tuple(float(value[key]) * scale for key in keys)
			return None
		if isinstance(value, (tuple, list)) and len(value) >= 3:
			return (float(value[0]) * scale, float(value[1]) * scale, float(value[2]) * scale)
		return None

	def read(self, gyro_scale=1.0, mag_scale=1.0):
		if self.imu is None:
			return {"gyro": None, "mag": None, "ok": False}

		raw = _try_call(self.imu, ("get",))
                if isinstance(raw, (tuple, list)) and len(raw) >= 6:
                        # Assumes format: (ax, ay, az, gx, gy, gz, mx, my, mz) or similar
                        ax, ay, az = float(raw[0]), float(raw[1]), float(raw[2])
                        gx, gy, gz = float(raw[3]), float(raw[4]), float(raw[5])
                        
                        mag_data = (0.0, 0.0, 0.0)
                        if len(raw) >= 8:
                            mag_data = (float(raw[6]) * mag_scale, float(raw[7]) * mag_scale, 0.0)
                            
                        return {
                                "accel": (ax, ay, az),
                                "gyro": (gx * gyro_scale, gy * gyro_scale, gz * gyro_scale), 
                                "mag": mag_data,
			}

		gyro = _try_call(self.imu, ("gyro", "get_gyro", "gyro_read", "read_gyro", "readGyro"))
		if gyro is None:
			gyro = _try_attr(self.imu, ("gyro_data", "gyro", "gyr", "gyr_data"))

		mag = _try_call(self.imu, ("mag", "get_mag", "read_mag", "readMag", "magnetic"))
		if mag is None:
			mag = _try_attr(self.imu, ("mag_data", "mag", "magnetic_data", "magn"))

		return {
			"gyro": self._normalize_vec(gyro, gyro_scale),
			"mag": self._normalize_vec(mag, mag_scale),
			"ok": True,
		}


class EncoderAdapter:
	def __init__(self, encoder, reverse=False):
		self.encoder = encoder
		self.reverse = reverse

	def read_count(self):
		if self.encoder is None:
			return None

		value = _try_call(self.encoder, ("value", "read", "get", "position", "get_count", "counter"))
		if value is None:
			value = _try_attr(self.encoder, ("value", "count", "position", "counter"))
		if value is None:
			return None
		try:
			value = int(value)
		except Exception:
			return None
		return -value if self.reverse else value


class TickerAdapter:
	def __init__(self, ticker):
		self.ticker = ticker

	def attach(self, callback, period_ms):
		if self.ticker is None:
			return False
		result = _try_call(self.ticker, ("capture_list", "attach", "start", "init"), callback, period_ms)
		return result is not None
