"""APP profile dispatcher.

Current slim build keeps only the competition runtime in APP.
Hardware/manual test scripts live under tests/hardware and are not part of the
main firmware entry chain.
"""


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

    print("\n[IMPORT ERROR] missing BSP package:", msg)
    print("Upload boot.py plus APP/, BSP/ and Module/ runtime files again.")
    if flash_ls is not None:
        print("/flash files:", flash_ls)
    print("")
    return True


def run_profile(profile):
    p = str(profile or "").strip().lower()

    if p in ("competition", "comp"):
        try:
            from APP.competition import main

            return main()
        except ImportError as e:
            if _hint_missing_package(e):
                raise
            raise

    raise ValueError("unsupported APP_PROFILE: {} (supported: competition)".format(p))
