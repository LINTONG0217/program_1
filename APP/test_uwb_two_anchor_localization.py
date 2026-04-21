import time
from machine import Pin

from BSP.board_runtime import MainControl
from Module import config
from Module.pose_estimation import PoseEstimator
from Module.uwb_two_anchor_localizer import TwoAnchorPoseSolver, UWBRangeReceiver


class _DisplayChassisProxy:
    def __init__(self):
        self.last_vx = 0.0
        self.last_vy = 0.0


def wait_c14_start():
    if not bool(getattr(config, "TEST_WAIT_C14_ENABLE", True)):
        return
    pin_name = getattr(config, "NAV_KEY_PIN", "C14")

    pull_up = getattr(Pin, "PULL_UP_47K", None)
    if pull_up is not None:
        try:
            key = Pin(pin_name, Pin.IN, pull_up)
        except Exception:
            key = Pin(pin_name, Pin.IN)
    else:
        key = Pin(pin_name, Pin.IN)

    print("press {} to start two-anchor UWB localization test".format(pin_name))
    while key.value() != 0:
        time.sleep_ms(20)
    time.sleep_ms(60)
    while key.value() == 0:
        time.sleep_ms(20)


def _format_range(anchor_id, range_map):
    data = range_map.get(anchor_id) if isinstance(range_map, dict) else None
    if not data:
        return "id{}=--".format(anchor_id)
    return "id{}={:.3f}m".format(anchor_id, float(data.get("distance_m", 0.0)))


def main():
    print("test: two-anchor uwb localization")
    print(
        "anchors: id{}=({:.2f},{:.2f}) id{}=({:.2f},{:.2f})".format(
            int(getattr(config, "UWB_ANCHOR_0_ID", 0)),
            float(getattr(config, "UWB_ANCHOR_0_X_M", 0.0)),
            float(getattr(config, "UWB_ANCHOR_0_Y_M", 0.0)),
            int(getattr(config, "UWB_ANCHOR_1_ID", 1)),
            float(getattr(config, "UWB_ANCHOR_1_X_M", 0.0)),
            float(getattr(config, "UWB_ANCHOR_1_Y_M", 0.0)),
        )
    )
    print(
        "poll: enable={} cmd={!r}".format(
            bool(getattr(config, "UWB_RANGE_POLL_ENABLE", False)),
            str(getattr(config, "UWB_RANGE_CMD_TEMPLATE", "") or ""),
        )
    )
    print("imu for two-anchor test: {}".format(bool(getattr(config, "UWB_TWO_ANCHOR_USE_IMU", False))))
    print(
        "start hint: x={:.2f} y={:.2f} yaw={:.1f} side={}".format(
            float(getattr(config, "UWB_TWO_ANCHOR_INIT_X_M", 0.25)),
            float(getattr(config, "UWB_TWO_ANCHOR_INIT_Y_M", 0.25)),
            float(getattr(config, "UWB_TWO_ANCHOR_INIT_YAW_DEG", 0.0)),
            int(getattr(config, "UWB_TWO_ANCHOR_PREFERRED_SIDE", 1)),
        )
    )

    control_board = MainControl(config)
    control_board.init()
    wait_c14_start()

    estimator = PoseEstimator(config)
    if bool(getattr(config, "UWB_TWO_ANCHOR_INIT_ENABLE", True)):
        estimator.request_pose_init(
            getattr(config, "UWB_TWO_ANCHOR_INIT_X_M", 0.25),
            getattr(config, "UWB_TWO_ANCHOR_INIT_Y_M", 0.25),
            getattr(config, "UWB_TWO_ANCHOR_INIT_YAW_DEG", 0.0),
        )
    elif bool(getattr(config, "POSE_INIT_ENABLE", False)):
        estimator.request_pose_init(
            getattr(config, "POSE_INIT_X", 0.0),
            getattr(config, "POSE_INIT_Y", 0.0),
            getattr(config, "POSE_INIT_YAW_DEG", 0.0),
        )

    receiver = UWBRangeReceiver(config)
    solver = TwoAnchorPoseSolver(config)
    display_proxy = _DisplayChassisProxy()
    print_interval = int(getattr(config, "UWB_TWO_ANCHOR_PRINT_MS", 200))
    last_print_ms = 0
    use_imu = bool(getattr(config, "UWB_TWO_ANCHOR_USE_IMU", False))
    display_error_printed = False

    while True:
        if not control_board.ticker.ready():
            time.sleep_ms(1)
            continue

        if use_imu:
            imu_data = control_board.read_imu()
        else:
            imu_data = {"gyro": None, "mag": None}
        encoder_data = control_board.read_encoder_speeds()
        pose = estimator.update(encoder_data, imu_data)

        receiver.update()
        fresh_ranges = solver.fresh_ranges(receiver)
        fix = solver.solve(pose, fresh_ranges)
        if fix:
            current_yaw = float(pose.get("yaw", fix.get("yaw", 0.0)))
            estimator.set_pose(fix["x"], fix["y"], fix["yaw"], current_absolute_yaw=current_yaw)
            pose = estimator.get_pose()
            pose["ts"] = int(fix.get("ts", time.ticks_ms()))

        try:
            control_board.update_display(display_proxy, pose)
        except Exception as e:
            if not display_error_printed:
                print("two-anchor test: lcd update disabled:", e)
                display_error_printed = True

        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_print_ms) < print_interval:
            continue
        last_print_ms = now_ms

        range_text = "{} {}".format(
            _format_range(int(getattr(config, "UWB_ANCHOR_0_ID", 0)), fresh_ranges),
            _format_range(int(getattr(config, "UWB_ANCHOR_1_ID", 1)), fresh_ranges),
        )
        if fix:
            print(
                "[2A FIX] x={:.3f} y={:.3f} yaw={:.1f} {} raw={!r}".format(
                    float(pose.get("x", 0.0)),
                    float(pose.get("y", 0.0)),
                    float(pose.get("yaw", 0.0)),
                    range_text,
                    receiver.last_line,
                )
            )
        else:
            print(
                "[2A WAIT] pred_x={:.3f} pred_y={:.3f} yaw={:.1f} {} raw={!r}".format(
                    float(pose.get("x", 0.0)),
                    float(pose.get("y", 0.0)),
                    float(pose.get("yaw", 0.0)),
                    range_text,
                    receiver.last_line,
                )
            )


if __name__ == "__main__":
    main()
