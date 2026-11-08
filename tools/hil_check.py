#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
hil_check.py - hardware-in-the-loop gate for auto_dev.py.

Compiles and uploads an example to the test bed, resets the board, captures
its serial output for a while, and exits non-zero unless every --expect
pattern appears and no --fail / crash pattern does. The captured output is
printed so a failing gate hands the author the real hardware log.

  python tools/hil_check.py --example Basic --expect "read versions: Ok"
  auto_dev.py ... --gate "python tools/hil_check.py --example ReadUid --expect \"UID:\""

Requires pyserial (pip install pyserial) and arduino-cli.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CRASH = r"Guru Meditation|Backtrace:|abort\(\) was called|Task watchdog got triggered|rst:0x[0-9a-f]+ \((?:.*PANIC|.*WDT)"


def run(cmd: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=600)


# esptool resets the chip after flashing ("--after hard-reset"), so the sketch boots once
# before the capture below pulses EN again. A sketch that writes cards can then be cut off
# mid-write by that second reset. Where the board's upload arguments allow it, flash with
# "--after no-reset" instead (the chip waits in the ROM loader), so the EN pulse below is
# the only boot.
# The board defines the arguments under the tool key; arduino-cli resolves them to
# upload.pattern_args before applying --upload-property, so the override uses that key.
AFTER_RESET_KEY = "tools.esptool_py.upload.pattern_args"
UPLOAD_ARGS_KEY = "upload.pattern_args"


def no_reset_after_upload(fqbn: str, sketch: Path) -> list[str]:
    props = run(["arduino-cli", "compile", "--fqbn", fqbn, "--show-properties=unexpanded", str(sketch)])
    for line in props.stdout.splitlines():
        key, sep, value = line.partition("=")
        if sep and key == AFTER_RESET_KEY and "--after hard-reset" in value:
            return ["--upload-property",
                    f"{UPLOAD_ARGS_KEY}={value.replace('--after hard-reset', '--after no-reset')}"]
    return []


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--example", required=True, help="examples/<Name>")
    ap.add_argument("--port", default="COM6")
    ap.add_argument("--fqbn", default="esp32:esp32:esp32")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=10.0, help="how long to capture after reset")
    ap.add_argument("--expect", action="append", default=[], help="regex that must appear (repeatable)")
    ap.add_argument("--fail", action="append", default=[], help="regex that must NOT appear (repeatable)")
    ap.add_argument("--until", help="stop capturing early once this regex appears")
    ap.add_argument("--min-count", action="append", default=[], metavar="N:REGEX",
                    help="REGEX must match at least N lines (repeatable)")
    args = ap.parse_args()

    try:
        import serial  # pyserial
    except ImportError:
        print("hil_check: pyserial not installed (pip install pyserial)")
        return 2

    sketch = ROOT / "examples" / args.example
    build = ROOT / "build" / "hil" / args.example
    base = ["--fqbn", args.fqbn, "--library", str(ROOT), "--build-path", str(build)]
    up = run(["arduino-cli", "compile", *base, str(sketch)])
    if up.returncode == 0:
        up = run(["arduino-cli", "upload", "--fqbn", args.fqbn, "--input-dir", str(build), "-p", args.port,
                  *no_reset_after_upload(args.fqbn, sketch), str(sketch)])
    if up.returncode != 0:
        print(f"hil_check: upload of {args.example} to {args.port} failed:")
        print("\n".join((up.stdout + up.stderr).strip().splitlines()[-20:]))
        return 1

    # Pulse EN via RTS (DTR low keeps GPIO0 high -> normal boot) and capture.
    with serial.Serial(args.port, args.baud, timeout=0.2) as port:
        port.dtr = False
        port.rts = True
        time.sleep(0.1)
        port.rts = False
        buf, end = b"", time.time() + args.seconds
        while time.time() < end:
            buf += port.read(4096)
            if args.until and re.search(args.until, buf.decode("utf-8", "replace")):
                time.sleep(0.5)
                buf += port.read(4096)
                break
    out = buf.decode("utf-8", "replace").replace("\r\n", "\n")

    problems = [f"missing expected output /{p}/" for p in args.expect if not re.search(p, out)]
    problems += [f"forbidden output /{p}/ present" for p in args.fail if re.search(p, out)]
    for spec in args.min_count:
        n, pat = spec.split(":", 1)
        got = sum(1 for line in out.splitlines() if re.search(pat, line))  # lines, not matches
        print(f"hil_check: /{pat}/ matched {got} line(s), need >= {n}")
        if got < int(n):
            problems.append(f"/{pat}/ matched {got} times, need >= {n}")
    if re.search(CRASH, out):
        problems.append("board crashed or reset (panic / watchdog)")

    print(f"--- serial output from {args.example} on {args.port} ({len(out)} chars) ---")
    print(out.strip()[-3000:] or "(no output)")
    print("--- end serial output ---")
    for p in problems:
        print(f"hil_check: {p}")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
