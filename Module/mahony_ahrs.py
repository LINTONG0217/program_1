"""Mahony AHRS helpers for lightweight heading fusion."""

import math


class MahonyYawAHRS:
    """Mahony PI feedback fusion for yaw from gyro Z and mag X/Y."""

    def __init__(self, kp=1.4, ki=0.02):
        self.kp = float(kp)
        self.ki = float(ki)
        self.yaw = 0.0
        self.integral = 0.0
        self.initialized = False

    def _wrap_180(self, value):
        while value > 180.0:
            value -= 360.0
        while value <= -180.0:
            value += 360.0
        return value

    def _wrap_360(self, value):
        while value >= 360.0:
            value -= 360.0
        while value < 0.0:
            value += 360.0
        return value

    def _mag_yaw(self, mag):
        if not isinstance(mag, (tuple, list)) or len(mag) < 2:
            return None
        hx = float(mag[0])
        hy = float(mag[1])
        if abs(hx) <= 1e-9 and abs(hy) <= 1e-9:
            return None
        return self._wrap_360(math.degrees(math.atan2(hy, hx)))

    def update(self, gyro_z_dps, mag=None, dt=0.02):
        dt = max(0.001, float(dt))
        gyro_z = 0.0 if gyro_z_dps is None else float(gyro_z_dps)
        mag_yaw = self._mag_yaw(mag)

        if not self.initialized:
            if mag_yaw is not None:
                self.yaw = mag_yaw
            self.initialized = True

        error = 0.0
        if mag_yaw is not None:
            error = self._wrap_180(mag_yaw - self.yaw)
            self.integral += error * dt

        corrected_rate = gyro_z + self.kp * error + self.ki * self.integral
        self.yaw = self._wrap_360(self.yaw + corrected_rate * dt)
        return self.yaw
