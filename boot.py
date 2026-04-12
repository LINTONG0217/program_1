
# 本示例程序演示如何通过 boot.py 文件进行 soft-boot 控制
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的拨码开关控制

# 示例程序运行效果为复位后执行本文件 通过 D8 电平状态决定是否跳转执行 user_main.py

# 从 machine 库包含所有内容
from machine import *

# 包含 gc 与 time 类
import gc
import os
import time


try:
	execfile  # type: ignore
except NameError:
	def execfile(path):
		with open(path, "r") as f:
			code = f.read()
		exec(compile(code, path, "exec"), globals(), globals())

# 上电启动时间延时：给 USB/串口枚举留出更稳定的窗口
time.sleep_ms(250)
# 选择学习板上的一号拨码开关作为启动选择开关
boot_select = Pin('D8', Pin.IN, pull=Pin.PULL_UP_47K)

# 安全烧录模式：按住 C14（导航键）上电/复位时，不自动运行 main.py，方便 Thonny/工具稳定连接。
try:
	nav_key = Pin('C14', Pin.IN, pull=Pin.PULL_UP_47K)
	safe_upload = (nav_key.value() == 0)
except Exception:
	safe_upload = False

# 如果拨码开关打开 对应引脚拉低 就启动用户文件（安全烧录模式下跳过）
if boot_select.value() == 0 and (not safe_upload):
	try:
		os.chdir("/flash")
		execfile("main.py")
	except:
		print("File not found.")

if safe_upload:
	try:
		print("SAFE_UPLOAD: C14 held, skip main.py")
	except Exception:
		pass
