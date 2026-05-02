import sys

with open('Module/pose_estimation.py', 'r', encoding='utf-8') as f:
    text = f.read()

text = text.replace(
'''                gyro = None
                mag = None
                if imu_data:
                        gyro = imu_data.get("gyro")
                        mag = imu_data.get("mag")
                gyro_z = self._safe_axis(gyro, 2)
                mag_angle = self._safe_mag_angle(mag)''',
'''                gyro = None
                mag = None
                accel = None
                if imu_data:
                        gyro = imu_data.get("gyro")
                        mag = imu_data.get("mag")
                        accel = imu_data.get("accel")
                gyro_z = self._safe_axis(gyro, 2)
                mag_angle = self._safe_mag_angle(mag)'''
)

text = text.replace(
'''                if gyro_z is None and mag_angle is None:
                        self.vw = odom_vw
                        self.yaw = (self.yaw + self.vw * dt) % 360.0
                elif gyro_z is None:
                        self.yaw = mag_angle
                        self.vw = odom_vw
                else:
                        gyro_rate = float(gyro_z)
                        self.vw = gyro_rate
                        if mag_angle is not None:
                                self.yaw = self.fusion.update(gyro_rate, mag_angle, dt) % 360.0
                        else:
                                self.yaw = (self.yaw + gyro_rate * dt) % 360.0  ''',
'''                if gyro is not None:
                        self.vw = float(gyro_z) if gyro_z else 0.0
                        self.yaw = self.fusion.update_3d(gyro, accel, mag, dt) % 360.0
                elif gyro_z is None and mag_angle is None:
                        self.vw = odom_vw
                        self.yaw = (self.yaw + self.vw * dt) % 360.0
                elif gyro_z is None:
                        self.yaw = mag_angle
                        self.vw = odom_vw
                else:
                        pass
'''
)

with open('Module/pose_estimation.py', 'w', encoding='utf-8') as f:
    f.write(text)
