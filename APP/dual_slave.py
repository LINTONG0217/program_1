import time

from APP.competition import build_car_link, build_chassis, build_uwb_pose, maybe_print_pose
from BSP.board_runtime import MainControl
from Module import config
from Module.dual_car_coop import SlaveSideFollower, build_robot_status
from Module.robot_runtime import DisplayLayer, DriveLayer, EstimationLayer


def main():
    print("dual_slave: build=2026-04-16 side-follow")

    chassis = build_chassis()
    control_board = MainControl(config)
    control_board.init()

    uwb_pose_source = build_uwb_pose(config)
    estimation = EstimationLayer(config, control_board, pose_source=uwb_pose_source)
    display = DisplayLayer(control_board)
    drive = DriveLayer(chassis, control_board)
    car_link = build_car_link(config)
    follower = SlaveSideFollower(chassis, car_link, config, pose_provider=lambda: estimation.last_pose)

    if bool(getattr(config, "BOOT_WAIT_C14_ENABLE", True)):
        print("press C14 to start")
        while not control_board.nav_key.is_pressed():
            time.sleep_ms(20)
        time.sleep_ms(60)
        while control_board.nav_key.is_pressed():
            time.sleep_ms(20)
        print("C14 start")

    allow_toggle = bool(getattr(config, "RUNTIME_C14_TOGGLE_ENABLE", True))
    running = True
    key_was_pressed = False
    last_sync = 0
    pose_print_ms = {"value": 0}
    print("dual slave mode start")

    try:
        while True:
            if allow_toggle:
                key_pressed = control_board.nav_key.is_pressed()
                if key_pressed and not key_was_pressed:
                    running = not running
                    if running:
                        print("C14: resume")
                    else:
                        print("C14: pause")
                        chassis.stop()
                    time.sleep_ms(60)
                key_was_pressed = key_pressed

            if control_board.ticker.ready():
                pose = estimation.update(chassis)

                if running:
                    follower.update(pose)
                    drive.safety_tick()
                else:
                    chassis.stop()

                display.update(pose, chassis)
                maybe_print_pose("SLAVE", pose, pose_print_ms, config)

                if car_link:
                    now = time.ticks_ms()
                    if time.ticks_diff(now, last_sync) >= getattr(config, "CAR_LINK_SEND_MS", 100):
                        last_sync = now
                        car_link.send(
                            build_robot_status(
                                "slave",
                                pose=pose,
                                chassis=chassis,
                                state=follower.state,
                                extra={
                                    "master_online": bool(follower._master_pose()),
                                    "target_pose": follower.last_target_pose,
                                },
                            )
                        )
            else:
                time.sleep_ms(1)
    finally:
        chassis.stop()


if __name__ == "__main__":
    main()
