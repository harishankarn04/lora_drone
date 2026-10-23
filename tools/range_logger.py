#!/usr/bin/env python3
"""Laptop-side logger for the E220 range test.

Shows the ESP32 output live, forwards what you type (commands like `cal 25`,
`mark 300m LOS`, `scan`) to the ESP32, and saves everything to
logs/range_<timestamp>.csv (DATA rows) + .log (raw). Prints a summary on Ctrl-C.

    pip install pyserial
    python3 range_logger.py                 # auto-detect port
    python3 range_logger.py -p /dev/cu.usbserial-0001
"""
import argparse
import csv
import datetime as dt
import os
import sys
import threading

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial missing: pip install pyserial")

HEADER = ["t_ms", "mode", "seq", "status", "rtt_ms", "rssi_down_dbm", "rssi_up_dbm",
          "rssi_avg_dbm", "dist_m", "trend", "attempts", "ok", "lost", "bad", "per_pct", "link", "age_ms"]


def find_port():
    ports = [p.device for p in list_ports.comports()
             if any(k in (p.device + (p.description or "")).lower()
                    for k in ("usbserial", "usbmodem", "slab", "wch", "cp210", "ch34", "ttyusb", "ttyacm"))]
    if not ports:
        sys.exit("No ESP32 serial port found; pass -p PORT")
    return ports[0]


def summarize(rows, marks):
    if not rows:
        print("\nNo data rows.")
        return
    ok = [r for r in rows if r["status"] == "OK"]
    last = rows[-1]
    print("\n===== session summary =====")
    print(f"attempts={last['attempts']} ok={last['ok']} lost={last['lost']} bad={last['bad']} PER={last['per_pct']}%")
    if ok:
        rssi = [float(r["rssi_avg_dbm"]) for r in ok if r["rssi_avg_dbm"]]
        dist = [float(r["dist_m"]) for r in ok if r["dist_m"]]
        print(f"RSSI avg={sum(rssi)/len(rssi):.1f} min={min(rssi):.1f} max={max(rssi):.1f} dBm")
        print(f"max estimated distance with a good packet: {max(dist):.0f} m")
    # loss per marker segment: lets you read "PER at 100 m, 200 m, ..." straight off the log
    if marks:
        print("\nper marker segment:")
        bounds = marks + [(float("inf"), "end")]
        for (t0, label), (t1, _) in zip(bounds, bounds[1:]):
            seg = [r for r in rows if t0 <= int(r["t_ms"]) < t1]
            if not seg:
                continue
            good = [r for r in seg if r["status"] == "OK"]
            lost = sum(1 for r in seg if r["status"] in ("TIMEOUT", "LOST"))
            rs = [float(r["rssi_avg_dbm"]) for r in good if r["rssi_avg_dbm"]]
            avg = f"{sum(rs)/len(rs):.1f} dBm" if rs else "-"
            print(f"  {label:<30} ok={len(good):<4} lost={lost:<4} avg RSSI={avg}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    args = ap.parse_args()

    port = args.port or find_port()
    ser = serial.Serial(port, args.baud, timeout=0.2)
    link = {"ser": ser}
    os.makedirs("logs", exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path, log_path = f"logs/range_{stamp}.csv", f"logs/range_{stamp}.log"
    csv_f, log_f = open(csv_path, "w", newline=""), open(log_path, "w")
    writer = csv.writer(csv_f)
    writer.writerow(["wall_time"] + HEADER)
    print(f"[logger] {port} -> {csv_path}  (type commands + Enter, Ctrl-C to stop)")

    rows, marks = [], []
    stop = threading.Event()

    def reader():
        buf = b""
        while not stop.is_set():
            try:
                buf += link["ser"].read(256)
            except (serial.SerialException, OSError):
                print(f"[logger] lost {port} (unplugged?) - waiting for it to come back ...")
                log_f.write(f"{dt.datetime.now().isoformat(timespec='milliseconds')} [logger] port lost\n")
                try:
                    link["ser"].close()
                except Exception:
                    pass
                while not stop.is_set():
                    try:
                        link["ser"] = serial.Serial(port, args.baud, timeout=0.2)
                        print(f"[logger] reconnected to {port}")
                        break
                    except (serial.SerialException, OSError):
                        stop.wait(1)
                buf = b""
                continue
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if not line:
                    continue
                now = dt.datetime.now().isoformat(timespec="milliseconds")
                log_f.write(f"{now} {line}\n")
                log_f.flush()
                if line.startswith("DATA,"):
                    vals = line.split(",")[1:]
                    if len(vals) == len(HEADER):
                        writer.writerow([now] + vals)
                        csv_f.flush()
                        r = dict(zip(HEADER, vals))
                        rows.append(r)
                        if r["status"] == "OK":
                            line = (f"seq={r['seq']:>5} OK  down={r['rssi_down_dbm']:>4} up={r['rssi_up_dbm'] or '-':>4} dBm"
                                    f"  rtt={r['rtt_ms']:>4}ms  dist~{r['dist_m']:>7}m  {r['trend']:<7}  PER={r['per_pct']}%  {r['link']}")
                        else:
                            line = f"seq={r['seq']:>5} {r['status']:<8}  {r['link']} (no packet for {r['age_ms']} ms)   PER={r['per_pct']}%"
                elif line.startswith("MARK,"):
                    _, t, label = line.split(",", 2)
                    marks.append((int(t), label))
                print(line)

    th = threading.Thread(target=reader, daemon=True)
    th.start()
    try:
        for cmd in sys.stdin:
            try:
                link["ser"].write((cmd.rstrip("\n") + "\n").encode())
            except (serial.SerialException, OSError):
                print("[logger] ESP32 not connected, command not sent")
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        th.join(timeout=1)
        try:
            link["ser"].close()
        except Exception:
            pass
        csv_f.close()
        log_f.close()
        summarize(rows, marks)
        print(f"\nsaved {csv_path} and {log_path}")


if __name__ == "__main__":
    main()
