import math
import time
try:
    from array import array
except Exception:
    array = None

def cross3d(a, b):
    return (a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0])

def norm3d(v):
    mag = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
    if mag == 0:
        return (0.0, 0.0, 0.0)
    return (v[0]/mag, v[1]/mag, v[2]/mag)

class IMUFusion:
    """Quaternion-based IMU Attitude Fusion referencing ins_task.c."""
    def __init__(self, q_angle=0.001, q_gyro=0.003, r_angle=0.03):
        self.q = [1.0, 0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.angle = 0.0 
        self.bias = 0.0   

        self.Kp = 2.0  
        self.Ki = 0.0  
        self.exInt = 0.0
        self.eyInt = 0.0
        self.ezInt = 0.0
        
        self.mag_table = None
        self.is_calibrated = False
        self._init_done = False

    def reset_quaternion(self, acc):
        """Initializes the quaternion using accelerometer data to estimate initial pitch and roll."""
        if not acc or (acc[0] == 0 and acc[1] == 0 and acc[2] == 0):
            self.q = [1.0, 0.0, 0.0, 0.0]
            return
        
        ax, ay, az = norm3d(acc)
        dot = az
        if dot > 1.0: dot = 1.0
        elif dot < -1.0: dot = -1.0
        angle = math.acos(dot)
        
        axis = cross3d((ax, ay, az), (0, 0, 1))
        nm = math.sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)
        if nm > 0.0:
            axis = (axis[0]/nm, axis[1]/nm, axis[2]/nm)
        else:
            axis = (1.0, 0.0, 0.0)
            
        half_angle = angle / 2.0
        s_half = math.sin(half_angle)
        self.q[0] = math.cos(half_angle)
        self.q[1] = axis[0] * s_half
        self.q[2] = axis[1] * s_half
        self.q[3] = axis[2] * s_half

    def update_quaternion(self, gx, gy, gz, dt):
        """Core standard Quaternion update from ins_task.c (gyro integration)."""
        gx *= 0.5 * dt
        gy *= 0.5 * dt
        gz *= 0.5 * dt
        
        qa, qb, qc, qd = self.q[0], self.q[1], self.q[2], self.q[3]
        self.q[0] += (-qb * gx - qc * gy - qd * gz)
        self.q[1] += (qa * gx + qc * gz - qd * gy)
        self.q[2] += (qa * gy - qb * gz + qd * gx)
        self.q[3] += (qa * gz + qb * gy - qc * gx)
        
        norm = math.sqrt(self.q[0]**2 + self.q[1]**2 + self.q[2]**2 + self.q[3]**2)
        if norm > 0.0:
            self.q[0] /= norm
            self.q[1] /= norm
            self.q[2] /= norm
            self.q[3] /= norm

    def update_mahony(self, gx, gy, gz, ax, ay, az, dt):
        """Mahony filter for fusing accel with gyro."""
        norm_a = math.sqrt(ax**2 + ay**2 + az**2)
        if norm_a == 0.0:
            self.update_quaternion(gx, gy, gz, dt)
            return

        ax /= norm_a
        ay /= norm_a
        az /= norm_a

        q0, q1, q2, q3 = self.q[0], self.q[1], self.q[2], self.q[3]

        vx = 2.0 * (q1 * q3 - q0 * q2)
        vy = 2.0 * (q0 * q1 + q2 * q3)
        vz = q0**2 - q1**2 - q2**2 + q3**2

        ex = (ay * vz - az * vy)
        ey = (az * vx - ax * vz)
        ez = (ax * vy - ay * vx)

        if self.Ki > 0.0:
            self.exInt += ex * dt
            self.eyInt += ey * dt
            self.ezInt += ez * dt
        else:
            self.exInt = 0.0
            self.eyInt = 0.0
            self.ezInt = 0.0

        gx += self.Kp * ex + self.Ki * self.exInt
        gy += self.Kp * ey + self.Ki * self.eyInt
        gz += self.Kp * ez + self.Ki * self.ezInt

        self.update_quaternion(gx, gy, gz, dt)

    def extract_euler(self):
        """Convert quaternion to Euler angles: Yaw, Pitch, Roll (degree), matching ins_task.c."""
        q = self.q
        self.yaw = math.atan2(2.0 * (q[0]*q[3] + q[1]*q[2]), 2.0 * (q[0]*q[0] + q[1]*q[1]) - 1.0) * 57.295779513
        self.pitch = math.atan2(2.0 * (q[0]*q[1] + q[2]*q[3]), 2.0 * (q[0]*q[0] + q[3]*q[3]) - 1.0) * 57.295779513
        
        val = 2.0 * (q[0]*q[2] - q[1]*q[3])
        if val > 1.0: val = 1.0
        elif val < -1.0: val = -1.0
        self.roll = math.asin(val) * 57.295779513
        self.angle = self.yaw

    def update_3d(self, gyro, accel, mag, dt):
        """Main interface for 3D update. Expecting gyro in degree/s."""
        gx, gy, gz = gyro[0], gyro[1], gyro[2]
        
        gx = math.radians(gx)
        gy = math.radians(gy)
        gz = math.radians(gz)

        if not self._init_done and accel is not None:
            self.reset_quaternion(accel)
            self._init_done = True

        if accel is not None:
            self.update_mahony(gx, gy, gz, accel[0], accel[1], accel[2], dt)
        else:
            self.update_quaternion(gx, gy, gz, dt)
        
        self.extract_euler()
        return self.yaw

    def update(self, gyro_rate, mag_angle, dt):
        """Legacy 1D update method"""
        return self.update_3d((0, 0, gyro_rate), None, None, dt)

    def get_mag_angle(self, hx, hy):
        raw_angle = math.degrees(math.atan2(hy, hx))
        if raw_angle < 0:
            raw_angle += 360
        if self.mag_table:
            return float(self.mag_table[int(raw_angle) % 360])
        return raw_angle

    def calibrate_mag(self, imu, motors, duration=5.0):
        print(">>> Ignore mag calib when using Mahony filter for now...")
        self.is_calibrated = True
