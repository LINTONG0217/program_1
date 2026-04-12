"""APP 调度。"""


def _hint_missing_package(err):
    msg = str(err or "")
    if "no module named" not in msg.lower():
        return False

    if "'bsp'" not in msg.lower() and " bsp" not in msg.lower():
        return False

    try:
        import os

        try:
            flash_ls = os.listdir("/flash")
        except Exception:
            flash_ls = os.listdir()
    except Exception:
        flash_ls = None

    print("\n[IMPORT ERROR] 缺少 BSP 包：", msg)
    print("这通常是因为板子里没有完整上传 BSP/Module/APP 目录，或你在错误的设备/解释器里运行。")
    print("建议：在电脑上运行 deploy.bat（或 deploy.ps1）重新部署，部署时确保 Thonny/串口助手已关闭。")
    if flash_ls is not None:
        print("/flash 目录内容:", flash_ls)
    print("")
    return True


def run_profile(profile):
    p = str(profile or "").strip().lower()

    if p in ("single",):
        from APP.single import main

        return main()

    if p in ("test_chassis", "chassis_test"):
        from APP.test_chassis import main

        return main()

    if p in ("test_straight", "straight_test"):
        from APP.test_straight import main

        return main()

    if p in ("competition", "comp"):
        try:
            from APP.competition import main

            return main()
        except ImportError as e:
            if _hint_missing_package(e):
                raise
            raise

    if p in ("dual_slave", "slave"):
        from APP.dual_slave import main

        return main()

    raise ValueError("unsupported APP_PROFILE: {} (supported: single/test_chassis/test_straight/competition/dual_slave)".format(p))
