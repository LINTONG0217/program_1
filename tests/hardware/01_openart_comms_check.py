"""视觉模块基础通讯检测"""

import time
from machine import Pin
try:
    from seekfree import IMU963RA, MOTOR_CONTROLLER, ENCODER
except ImportError:
    pass

from Module import config
from Module.uart_vision_receiver import VisionReceiver

print("="*40)
print("=== OpenART 视觉模块通讯体检 ===")
print("="*40)

def main():
    try:
        vision = VisionReceiver(
            config.VISION_UART_ID, 
            config.VISION_BAUDRATE, 
            config.VISION_TX_PIN, 
            config.VISION_RX_PIN, 
            config.FRAME_WIDTH, 
            config.FRAME_HEIGHT
        )
        print("[✓] 串口初始化成功")
    except Exception as e:
        print("[✗] 串口初始化失败:", e)
        return

    print("\n正在等待接收画面帧... (如果在 Thonny 中，随时可以点击红色的 Stop 终止)")
    
    last_print = time.ticks_ms()
    frame_count = 0
    
    while True:
        frame = vision.get_latest_frame(timeout_ms=500)
        
        target = frame.get("object") if frame else None
        
        if time.ticks_diff(time.ticks_ms(), last_print) > 500:
            last_print = time.ticks_ms()
            if not target:
                print(f"[{frame_count}] 接收通讯正常，但视野内没有没识别到目标...")
            else:
                print(f"[{frame_count}] 🎯 发现目标！ ->", target)
            frame_count += 1
            
        time.sleep(0.02)

if __name__ == "__main__":
    main()
