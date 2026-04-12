"""扫描主控可用串口并显示是否收到视觉字节。"""

import time
import sys
import machine
from machine import Pin

# Some firmwares (e.g. OpenMV/OpenART) expose UART via pyb.UART.
try:
    import pyb  # type: ignore
except Exception:
    pyb = None
from Module import config


# Strict mode avoids misleading results caused by pinless UART fallbacks.
# When True, scanner will only open UART with explicit rx (and optionally tx).
STRICT_PINS = True

# Print summarized UART open errors when nothing is received.
PRINT_OPEN_ERRORS = True

# Quick sanity check to see which UART IDs can be opened at all (pinless).
PREFLIGHT_PINLESS = True

# Print one full exception traceback (once) to diagnose firmware API mismatch.
VERBOSE_TRACEBACK_ONCE = True
_printed_traceback = False

# This script is intended to run on the MAIN controller (receiver).
# Some OpenART/OpenMV firmwares also report sys.platform == 'mimxrt'.
# Do NOT abort automatically — user may intentionally run it there.
ABORT_ON_MIMXRT = False

# Print detailed UART open attempts for the first failing (impl, uid) per pin pair.
PRINT_OPEN_ATTEMPTS_ONCE_PER_PAIR = True


def _try_fpioa_register_uart(uid, tx, rx):
    try:
        from fpioa_manager import fm  # type: ignore
    except Exception:
        return {"ok": False, "tx": False, "rx": False}
    try:
        fpioa = getattr(fm, "fpioa", None)
        tx_func = getattr(fpioa, "UART{}_TX".format(uid), None)
        rx_func = getattr(fpioa, "UART{}_RX".format(uid), None)

        ok_tx = False
        ok_rx = False
        if tx_func is not None:
            try:
                fm.register(tx, tx_func, force=True)
                ok_tx = True
            except Exception:
                pass
        if rx_func is not None:
            try:
                fm.register(rx, rx_func, force=True)
                ok_rx = True
            except Exception:
                pass
        return {"ok": bool(ok_tx or ok_rx), "tx": bool(ok_tx), "rx": bool(ok_rx)}
    except Exception:
        return {"ok": False, "tx": False, "rx": False}


def _err_sig(err):
    if err is None:
        return "None"
    try:
        msg = str(err)
    except Exception:
        msg = ""
    try:
        msg = msg.replace("\r", " ").replace("\n", " ")
    except Exception:
        pass
    if len(msg) > 80:
        msg = msg[:80] + "..."
    return "{}:{}".format(type(err).__name__, msg)


def _print_exception_once(err):
    global _printed_traceback
    if not VERBOSE_TRACEBACK_ONCE or _printed_traceback:
        return
    _printed_traceback = True
    try:
        print("--- first UART open exception traceback ---")
        try:
            sys.print_exception(err)
        except Exception:
            pass
        print("--- end traceback ---")
    except Exception:
        pass


def _get_uart_impls():
    impls = []
    try:
        impls.append(("machine", getattr(machine, "UART")))
    except Exception:
        pass
    try:
        if pyb is not None and hasattr(pyb, "UART"):
            u = getattr(pyb, "UART")
            # Avoid duplicates
            if not any(u is it for _, it in impls):
                impls.append(("pyb", u))
    except Exception:
        pass
    return impls


def open_uart(uart_impl, uid, tx, rx, baud):
    map_stat = _try_fpioa_register_uart(uid, tx, rx)

    tries = []
    # Full config first
    tries += [
        ("baudrate+Pin+timeout", lambda: uart_impl(uid, baudrate=baud, tx=Pin(tx), rx=Pin(rx), timeout=80, read_buf_len=1024)),
        ("posbaud+Pin+timeout", lambda: uart_impl(uid, baud, tx=Pin(tx), rx=Pin(rx), timeout=80, read_buf_len=1024)),
        ("baudrate+raw+timeout", lambda: uart_impl(uid, baudrate=baud, tx=tx, rx=rx, timeout=80, read_buf_len=1024)),
        ("posbaud+raw+timeout", lambda: uart_impl(uid, baud, tx=tx, rx=rx, timeout=80, read_buf_len=1024)),
        ("baudrate+Pin", lambda: uart_impl(uid, baudrate=baud, tx=Pin(tx), rx=Pin(rx))),
        ("posbaud+Pin", lambda: uart_impl(uid, baud, tx=Pin(tx), rx=Pin(rx))),
        ("baudrate+raw", lambda: uart_impl(uid, baudrate=baud, tx=tx, rx=rx)),
        ("posbaud+raw", lambda: uart_impl(uid, baud, tx=tx, rx=rx)),
    ]
    # RX-only listen (often more compatible)
    tries += [
        ("baudrate+rxPin+timeout", lambda: uart_impl(uid, baudrate=baud, rx=Pin(rx), timeout=80, read_buf_len=1024)),
        ("posbaud+rxPin+timeout", lambda: uart_impl(uid, baud, rx=Pin(rx), timeout=80, read_buf_len=1024)),
        ("baudrate+rxRaw+timeout", lambda: uart_impl(uid, baudrate=baud, rx=rx, timeout=80, read_buf_len=1024)),
        ("posbaud+rxRaw+timeout", lambda: uart_impl(uid, baud, rx=rx, timeout=80, read_buf_len=1024)),
        ("baudrate+rxPin", lambda: uart_impl(uid, baudrate=baud, rx=Pin(rx))),
        ("posbaud+rxPin", lambda: uart_impl(uid, baud, rx=Pin(rx))),
        ("baudrate+rxRaw", lambda: uart_impl(uid, baudrate=baud, rx=rx)),
        ("posbaud+rxRaw", lambda: uart_impl(uid, baud, rx=rx)),
    ]

    # Optional pinless fallbacks (may read from default pins and confuse diagnosis)
    if not STRICT_PINS:
        tries += [
            ("baudrate", lambda: uart_impl(uid, baudrate=baud)),
            ("posbaud", lambda: uart_impl(uid, baud)),
        ]

    # If we successfully mapped pins via FPIOA, a pinless UART open can still be
    # the correct way on some firmwares (pins are selected by the mapping).
    if map_stat.get("ok"):
        tries += [
            ("mapped+baudrate+timeout", lambda: uart_impl(uid, baudrate=baud, timeout=80, read_buf_len=1024)),
            ("mapped+posbaud+timeout", lambda: uart_impl(uid, baud, timeout=80, read_buf_len=1024)),
            ("mapped+baudrate", lambda: uart_impl(uid, baudrate=baud)),
            ("mapped+posbaud", lambda: uart_impl(uid, baud)),
        ]

    # On some firmwares (e.g. OpenMV), UART pins are fixed per UART id and
    # passing tx/rx is unsupported; pinless open is the correct method.
    tries += [
        ("pinless+baudrate", lambda: uart_impl(uid, baudrate=baud)),
        ("pinless+posbaud", lambda: uart_impl(uid, baud)),
        ("pinless+idonly", lambda: uart_impl(uid)),
    ]

    last_err = None
    attempt_errs = []
    for label, f in tries:
        try:
            return f(), None
        except Exception as e:
            last_err = e
            attempt_errs.append((label, e))

    if attempt_errs and PRINT_OPEN_ATTEMPTS_ONCE_PER_PAIR:
        # Let caller decide whether to print; we attach details on the exception object.
        try:
            setattr(last_err, "_scan_attempt_errs", attempt_errs)  # type: ignore
        except Exception:
            pass

    _print_exception_once(last_err)
    return None, last_err


def close_uart(u):
    if not u:
        return
    deinit_fn = getattr(u, "deinit", None)
    if callable(deinit_fn):
        try:
            deinit_fn()
        except Exception:
            pass
    try:
        del u
    except Exception:
        pass
    try:
        import gc
        gc.collect()
    except Exception:
        pass


def scan_one(uart_impl, uid, tx, rx, baud, window_ms=1500):
    u, err = open_uart(uart_impl, uid, tx, rx, baud)
    if u is None:
        return None, err

    byte_total = 0
    preview = None
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < int(window_ms):
        data = None
        any_fn = getattr(u, "any", None)
        read_fn = getattr(u, "read", None)
        readline_fn = getattr(u, "readline", None)

        # Path 1: any/read
        if callable(any_fn) and callable(read_fn):
            try:
                n = any_fn()
            except Exception:
                n = 0
            if n and n > 0:
                try:
                    data = read_fn(n)
                except Exception:
                    data = None

        # Path 2: readline (some firmwares don't implement any())
        if not data and callable(readline_fn):
            try:
                data = readline_fn()
            except Exception:
                data = None

        # Path 3: read a small fixed chunk (last resort)
        if not data and callable(read_fn):
            try:
                data = read_fn(64)
            except Exception:
                data = None

        if data:
            ln = len(data) if data else 0
            byte_total += ln
            if preview is None:
                preview = data[:80]
                try:
                    preview = preview.decode("utf-8", "ignore")
                except Exception:
                    preview = str(preview)
        time.sleep_ms(20)

    close_uart(u)
    return {"bytes": byte_total, "preview": preview}, None


def main():
    # Environment banner (helps distinguish which firmware/device is executing this script)
    platform = None
    try:
        platform = getattr(sys, "platform", None)
        print("env: sys.platform=", platform)
    except Exception:
        platform = None
    try:
        uname = getattr(sys, "uname", None)
        if callable(uname):
            print("env: uname=", uname())
    except Exception:
        pass
    try:
        print("env: Pin=", Pin, " type=", type(Pin))
    except Exception:
        pass

    impls = _get_uart_impls()
    try:
        print("env: uart_impls=", [name for name, _ in impls])
    except Exception:
        pass

    if platform == "mimxrt":
        print("WARN: sys.platform=mimxrt; OpenART/OpenMV firmware is common on this platform.")
        if ABORT_ON_MIMXRT:
            print("STOP: ABORT_ON_MIMXRT=True")
            return

    near_tx = getattr(config, "VISION_TX_PIN", 4)
    near_rx = getattr(config, "VISION_RX_PIN", 5)
    near_baud = getattr(config, "VISION_BAUDRATE", 115200)

    f_dual = bool(getattr(config, "VISION_DUAL_ENABLE", False))
    f_tx = getattr(config, "VISION2_TX_PIN", None)
    f_rx = getattr(config, "VISION2_RX_PIN", None)
    f_baud = getattr(config, "VISION2_BAUDRATE", near_baud)

    # Prefer to detect available UART IDs first, then scan only those.
    detected = []
    if PREFLIGHT_PINLESS:
        print("uart preflight (pinless open):")
        for uid in range(0, 10):
            ok_any = False
            for impl_name, uart_impl in impls:
                u = None
                try:
                    for f in (
                        lambda: uart_impl(uid, baudrate=near_baud),
                        lambda: uart_impl(uid, near_baud),
                        lambda: uart_impl(uid),
                    ):
                        try:
                            u = f()
                            break
                        except Exception:
                            u = None
                    if u is not None:
                        close_uart(u)
                        ok_any = True
                except Exception:
                    pass
            if ok_any:
                detected.append(uid)
        if detected:
            print("uart ids available:", detected)
        else:
            print("uart ids available: (none detected with pinless open)")

    uids = detected if detected else [1, 2, 3, 4, 5, 6, 7]

    def _pin_variants(tx, rx):
        pairs = []
        pairs.append((tx, rx, "as_is"))
        pairs.append((rx, tx, "swapped"))
        # If ints like 4/5, also try "D4"/"D5" names.
        try:
            if isinstance(tx, int) and isinstance(rx, int):
                pairs.append(("D{}".format(tx), "D{}".format(rx), "Dxx"))
                pairs.append(("D{}".format(rx), "D{}".format(tx), "Dxx_swapped"))
        except Exception:
            pass
        # If strings like "D20", also try stripping leading "D" as int.
        try:
            if isinstance(tx, str) and tx.upper().startswith("D") and isinstance(rx, str) and rx.upper().startswith("D"):
                itx = int(tx[1:])
                irx = int(rx[1:])
                pairs.append((itx, irx, "int"))
                pairs.append((irx, itx, "int_swapped"))
        except Exception:
            pass
        # Dedup
        out = []
        seen = set()
        for a, b, tag in pairs:
            key = (str(a), str(b))
            if key in seen:
                continue
            seen.add(key)
            out.append((a, b, tag))
        return out

    print("start scan (sequential)... ensure OpenART is sending continuously")
    print("TIP: run with only ONE OpenART connected to isolate wiring")

    near_pairs = _pin_variants(near_tx, near_rx)
    for tx, rx, tag in near_pairs:
        print("near pins [{}] tx={} rx={} baud={}".format(tag, tx, rx, near_baud))
        found_any = False
        open_fail = 0
        err_counts = {}
        printed_attempts = False
        for impl_name, uart_impl in impls:
            for uid in uids:
                res, err = scan_one(uart_impl, uid, tx, rx, near_baud)
                if res is None:
                    open_fail += 1
                    if (not printed_attempts) and PRINT_OPEN_ATTEMPTS_ONCE_PER_PAIR:
                        printed_attempts = True
                        attempt_errs = getattr(err, "_scan_attempt_errs", None)
                        if attempt_errs:
                            print("open attempts ({} UART{}):".format(impl_name, uid))
                            for lbl, ee in attempt_errs[:8]:
                                print(" - {} -> {}".format(lbl, _err_sig(ee)))
                    sig = "{}|{}".format(impl_name, _err_sig(err))
                    err_counts[sig] = err_counts.get(sig, 0) + 1
                    continue
                if res["bytes"] and res["bytes"] > 0:
                    found_any = True
                    print("{} UART{} near bytes={} preview={}".format(impl_name, uid, res["bytes"], res["preview"]))
        if not found_any:
            msg = "(no UART got bytes for this pin pair; open_failed={})".format(open_fail)
            if PRINT_OPEN_ERRORS and open_fail >= len(uids) and err_counts:
                top = sorted(err_counts.items(), key=lambda kv: -kv[1])[:2]
                msg += " top_err=" + " | ".join(["{} x{}".format(k, v) for k, v in top])
            print(msg)

    if f_dual and f_tx is not None and f_rx is not None:
        far_pairs = _pin_variants(f_tx, f_rx)
        for tx, rx, tag in far_pairs:
            print("far pins  [{}] tx={} rx={} baud={}".format(tag, tx, rx, f_baud))
            found_any = False
            open_fail = 0
            err_counts = {}
            printed_attempts = False
            for impl_name, uart_impl in impls:
                for uid in uids:
                    res, err = scan_one(uart_impl, uid, tx, rx, f_baud)
                    if res is None:
                        open_fail += 1
                        if (not printed_attempts) and PRINT_OPEN_ATTEMPTS_ONCE_PER_PAIR:
                            printed_attempts = True
                            attempt_errs = getattr(err, "_scan_attempt_errs", None)
                            if attempt_errs:
                                print("open attempts ({} UART{}):".format(impl_name, uid))
                                for lbl, ee in attempt_errs[:8]:
                                    print(" - {} -> {}".format(lbl, _err_sig(ee)))
                        sig = "{}|{}".format(impl_name, _err_sig(err))
                        err_counts[sig] = err_counts.get(sig, 0) + 1
                        continue
                    if res["bytes"] and res["bytes"] > 0:
                        found_any = True
                        print("{} UART{} far  bytes={} preview={}".format(impl_name, uid, res["bytes"], res["preview"]))
            if not found_any:
                msg = "(no UART got bytes for this pin pair; open_failed={})".format(open_fail)
                if PRINT_OPEN_ERRORS and open_fail >= len(uids) and err_counts:
                    top = sorted(err_counts.items(), key=lambda kv: -kv[1])[:2]
                    msg += " top_err=" + " | ".join(["{} x{}".format(k, v) for k, v in top])
                print(msg)

    print("scan done")


if __name__ == "__main__":
    main()
