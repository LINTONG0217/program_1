import sys
import re

with open('Module/pose_estimation.py', 'r', encoding='utf-8') as f:
    text = f.read()

text = re.sub(r'gyro = None\s+mag = None\s+if imu_data:\s+gyro = imu_data\.get\("gyro"\)\s+mag = imu_data\.get\("mag"\)',
              r'gyro = None\n\t\tmag = None\n\t\taccel = None\n\t\tif imu_data:\n\t\t\tgyro = imu_data.get("gyro")\n\t\t\tmag = imu_data.get("mag")\n\t\t\taccel = imu_data.get("accel")', text)

text = re.sub(r'if gyro_z is None and mag_angle is None:(.+?)elif gyro_z is None:(.+?)else:(.+?)self\.yaw = \(self\.yaw \+ gyro_rate \* dt\) % 360\.0\s*',
              r'if gyro is not None:\n\t\t\tself.vw = float(gyro_z) if gyro_z else 0.0\n\t\t\tself.yaw = self.fusion.update_3d(gyro, accel, mag, dt) % 360.0\n\t\telif gyro_z is None and mag_angle is None:\g<1>elif gyro_z is None:\g<2>else:\n\t\t\tpass\n\t\t', text, flags=re.DOTALL)

with open('Module/pose_estimation.py', 'w', encoding='utf-8') as f:
    f.write(text)
