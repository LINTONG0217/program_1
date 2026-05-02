"""综合系统健康检查脚本
在Thonny中运行此脚本，将自动为您验证主板的以下基础硬件状态：
1. IMU 芯片通讯及读数 (验证刚升级的IMU融合算法数据流入)
2. 电机控制器 PWM极性尝试
3. 编码器 读数获取
"""

import time
from machine import Pin
try:
    from seekfree import IMU963RA, MOTOR_CONTROLLER, ENCODER
except ImportError:
    pass

print("="*40)
print("=== 小车综合硬件健康检查测试 ===")
print("="*40)

def test_imu():
    print("\n[1] 正在检查 IMU963RA (姿态传感器)...")
    try:
        imu = IMU963RA("C10", "C12", "C13")
        imu.init()
        data = imu.get()
        if data and len(data) >= 8:
            print("  [✓] IMU 通讯正常！")
            ax, ay, az = data[0:3]
            gx, gy, gz = data[3:6]
            print("  加速度计原始值: ax={:6}, ay={:6}, az={:6}".format(ax, ay, az))
            print("  陀螺仪原始值 : gx={:6}, gy={:6}, gz={:6}".format(gx, gy, gz))
            return True
        else:
            print("  [✗] IMU 获取数据格式异常: ", data)
    except Exception as e:
        print("  [✗] IMU 报错：", e)
    return False

def test_motors():
    print("\n[2] 正在检查 四轮电机 PWM驱动...")
    try:
        motors = [
            MOTOR_CONTROLLER("D0", "D1"),
            MOTOR_CONTROLLER("D2", "D3"),
            MOTOR_CONTROLLER("D4", "D5"),
            MOTOR_CONTROLLER("D6", "D7")
        ]
        print("  电机初始化成功，尝试进行微弱抖动测试(3秒)...")
        print("  !!! 注意：小车轮子可能会轻微抖动，请悬空放置或确保安全 !!!")
        time.sleep(2)
        for i, motor in enumerate(motors):
            motor.duty(1500) # 微弱正转
            time.sleep(0.5)
            motor.duty(0)
            print(f"  - 电机[{i}] 单独测试完成")
        print("  [✓] 电机控制器命令下发正常！")
    except Exception as e:
        print("  [✗] 电机 报错：", e)

def test_encoders():
    print("\n[3] 正在检查 编码器 (手动推送测试)...")
    try:
        encoders = [
            ENCODER("A0", "A1"),
            ENCODER("A2", "A3"),
            ENCODER("A4", "A5"),
            ENCODER("A6", "A7")
        ]
        list_e = []
        for enc in encoders:
            list_e.append(enc.init())
        print("  系统准备读取编码器脉冲数。")
        print("  << 请用手拨动小车的4个轮子 >>")
        for _ in range(20): # 循环监测2秒
            counts = [enc.get() for enc in encoders]
            print(f"  [监测中] 轮1:{counts[0]:4} | 轮2:{counts[1]:4} | 轮3:{counts[2]:4} | 轮4:{counts[3]:4} \r", end="")
            time.sleep(0.2)
        print("\n  [✓] 编码器读取完毕。如果在拨动轮子时数值发生变化，则编码器正常。")
    except Exception as e:
        print("  [✗] 编码器 报错：", e)

def main():
    test_imu()
    time.sleep(1)
    test_motors()
    time.sleep(1)
    test_encoders()
    print("\n" + "="*40)
    print("硬件健康检查结束！请根据上述输出判断问题点。")
    print("="*40)

if __name__ == "__main__":
    main()
