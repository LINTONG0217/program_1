import os
import glob

def patch_file(filepath):
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()

    new_content = content.replace(
"""    duties = [fl, fr, bl, br]
    for i in range(4):
        motors[i].duty(int(clamp(duties[i], -10000, 10000)))""", 
"""    duties = [fl, fr, bl, br]
    reverses = [
        getattr(config, "MOTOR_REVERSE", {}).get("fl", False),
        getattr(config, "MOTOR_REVERSE", {}).get("fr", True),
        getattr(config, "MOTOR_REVERSE", {}).get("bl", False),
        getattr(config, "MOTOR_REVERSE", {}).get("br", True),
    ]
    for i in range(4):
        duty = duties[i]
        if reverses[i]:
            duty = -duty
        motors[i].duty(int(clamp(duty, -10000, 10000)))""")

    if new_content != content:
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(new_content)
        print("Patched:", filepath)

for f in glob.glob("tests/hardware/manual_*.py"):
    patch_file(f)

