#!/usr/bin/env python3
"""Hardware dropout test: mutes the UNO beacon over USB for set durations and times how the
ESP32 tester reacts (COASTING / LOST detection and reacquisition after the beacon returns).

    python3 dropout_test.py --esp /dev/cu.usbserial-0001 --uno /dev/cu.usbmodem11101 --reps 3
Host timestamps are used for both ports (USB adds ~1-10 ms jitter).
"""
import argparse
import re
import statistics
import threading
import time

import serial

ap = argparse.ArgumentParser()
ap.add_argument("--esp", default="/dev/cu.usbserial-0001")
ap.add_argument("--uno", default="/dev/cu.usbmodem11101")
ap.add_argument("--durations", default="1000,3000,10000,30000", help="mute lengths in ms")
ap.add_argument("--reps", type=int, default=3)
ap.add_argument("--settle", type=float, default=20, help="s of normal link between dropouts")
args = ap.parse_args()

esp = serial.Serial(args.esp, 115200, timeout=0.05)
uno = serial.Serial(args.uno, 115200, timeout=0.05)
events = []          # (host_time, line) from the ESP32
lock = threading.Lock()
stop = False


def reader():
    buf = b""
    while not stop:
        buf += esp.read(512)
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode(errors="replace").strip()
            with lock:
                events.append((time.monotonic(), line))
            if line.startswith((">>>", "#      timing", "#      RTT")):
                print(f"  esp: {line}")


threading.Thread(target=reader, daemon=True).start()


def first_after(t0, pattern, timeout):
    rx = re.compile(pattern)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        with lock:
            for t, line in events:
                if t >= t0 and rx.search(line):
                    return t
        time.sleep(0.01)
    return None


print("waiting for link ...")
if first_after(time.monotonic() - 60, r"^DATA,.*,OK,", 120) is None:
    raise SystemExit("no OK packets from ESP32 - is the link up?")
time.sleep(args.settle)

res = {}
for d in [int(x) for x in args.durations.split(",")]:
    for rep in range(args.reps):
        t_mute = time.monotonic()
        uno.write(f"mute {d}\n".encode())
        t_end = t_mute + d / 1000
        coast = first_after(t_mute, r"LINK COASTING", d / 1000 + 1)
        lost = first_after(t_mute, r"LINK LOST", d / 1000 + 1)
        time.sleep(max(0, t_end - time.monotonic()))
        back = first_after(t_end, r"^DATA,.*,OK,", 120)
        r = res.setdefault(d, {"coast": [], "lost": [], "reacq": []})
        if coast: r["coast"].append((coast - t_mute) * 1000)
        if lost: r["lost"].append((lost - t_mute) * 1000)
        if back: r["reacq"].append((back - t_end) * 1000)
        print(f"mute {d:>6} ms #{rep + 1}: coast={'%.0f' % ((coast - t_mute) * 1000) if coast else '-':>6} "
              f"lost={'%.0f' % ((lost - t_mute) * 1000) if lost else '-':>6} "
              f"reacq={'%.0f' % ((back - t_end) * 1000) if back else 'NONE':>6} ms")
        time.sleep(args.settle)

stop = True
print("\n== summary (ms)        coast-detect   lost-detect   reacquisition (median / max)")
for d, r in res.items():
    f = lambda v: f"{statistics.median(v):6.0f} / {max(v):6.0f}" if v else "     - /      -"
    print(f"mute {d:>6}            {f(r['coast'])}  {f(r['lost'])}  {f(r['reacq'])}")
