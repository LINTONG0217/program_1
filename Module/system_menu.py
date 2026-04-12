"""系统菜单状态机。"""

import time


class SystemMenu:
    STATE_IDLE = 0
    STATE_NAVIGATION = 1

    def __init__(self, btn_nav, global_pos, target_x=2.0, target_y=1.0, debounce_ms=20):
        self.btn_nav = btn_nav
        self.global_pos = global_pos
        self.state = self.STATE_IDLE
        self.debounce_ms = debounce_ms
        self.target_x = target_x
        self.target_y = target_y

    def configure(self, target_x=None, target_y=None, debounce_ms=None):
        if target_x is not None:
            self.target_x = target_x
        if target_y is not None:
            self.target_y = target_y
        if debounce_ms is not None:
            self.debounce_ms = debounce_ms

    def set_target(self, target_x, target_y):
        self.target_x = target_x
        self.target_y = target_y

    def start_navigation(self, curr_yaw):
        self.state = self.STATE_NAVIGATION
        self.global_pos.reset(curr_yaw)
        print("\n>>> 🚀【启动全向导航】-> 目标: X:{}m, Y:{}m".format(self.target_x, self.target_y))

    def stop_navigation(self):
        self.state = self.STATE_IDLE
        print("\n>>> ⏹️【紧急停车】！")

    def check_inputs(self, curr_yaw):
        if self.btn_nav.value() == 0:
            time.sleep_ms(self.debounce_ms)
            if self.btn_nav.value() == 0:
                if self.state != self.STATE_NAVIGATION:
                    self.start_navigation(curr_yaw)
                else:
                    self.stop_navigation()
                while self.btn_nav.value() == 0:
                    pass
