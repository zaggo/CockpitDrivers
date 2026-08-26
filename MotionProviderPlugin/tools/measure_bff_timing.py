#!/usr/bin/env python3
"""Timestamp every BFF frame arriving on a serial device and report TX-cadence
statistics. The measuring end for SerialLink tests (see bff_sender_test.cpp):

    socat -d -d pty,raw,echo=0 pty,raw,echo=0     # prints two /dev/ttysNNN
    python3 measure_bff_timing.py /dev/ttysAAA --duration 35
    ./bff_sender_test /dev/ttysBBB 60 30

Reports, separately for distinct-payload frames (real updates) and
duplicate-payload frames (keepalives):
  interval min/mean/p99/max/stddev, bursts (<3 ms), gaps (>25 ms between
  distinct updates).

Pass criteria for the event-driven SerialLink (stage 3):
  distinct-update p99 < 1.5x median, no back-to-back duplicates outside the
  deliberate pause window, no <3 ms bursts.

Requires pyserial (PlatformIO's python has it: ~/.platformio/penv/bin/python3).
"""

import argparse
import statistics
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial missing - use ~/.platformio/penv/bin/python3 or pip install pyserial")
    sys.exit(2)

FRAME_SIZE = 16


def percentile(values, pct):
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = min(int(len(ordered) * pct), len(ordered) - 1)
    return ordered[idx]


def report(name, intervals_ms):
    if len(intervals_ms) < 2:
        print(f"{name}: not enough frames ({len(intervals_ms)} intervals)")
        return None
    mean = statistics.mean(intervals_ms)
    med = statistics.median(intervals_ms)
    p99 = percentile(intervals_ms, 0.99)
    std = statistics.pstdev(intervals_ms)
    print(f"{name}: n={len(intervals_ms)} min={min(intervals_ms):.1f} "
          f"mean={mean:.1f} median={med:.1f} p99={p99:.1f} max={max(intervals_ms):.1f} "
          f"std={std:.2f} ms")
    return med, p99


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", help="serial device / pty to listen on")
    ap.add_argument("--duration", type=float, default=35.0, help="seconds to capture")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    # timeout=0 + in_waiting poll at ~1 ms: a blocking read would batch several
    # frames into one chunk and stamp them with a single timestamp, faking 0 ms
    # bursts and 50 ms gaps that the sender never produced.
    port = serial.Serial(args.port, args.baud, timeout=0)
    print(f"listening on {args.port} for {args.duration:.0f} s ...")

    buf = bytearray()
    frames = []  # (t_monotonic, payload_bytes)
    start = time.monotonic()
    while time.monotonic() - start < args.duration:
        waiting = port.in_waiting
        if not waiting:
            time.sleep(0.001)
            continue
        chunk = port.read(waiting)
        now = time.monotonic()
        buf.extend(chunk)
        # Hunt for complete frames: 'B' 'C' <13 bytes> 0x0D
        while True:
            idx = buf.find(b"BC")
            if idx < 0 or len(buf) - idx < FRAME_SIZE:
                if idx > 0:
                    del buf[:idx]
                break
            frame = bytes(buf[idx:idx + FRAME_SIZE])
            if frame[-1] == 0x0D:
                frames.append((now, frame[2:15]))
                del buf[:idx + FRAME_SIZE]
            else:
                del buf[:idx + 2]  # false sync, keep hunting
    port.close()

    if len(frames) < 3:
        print("too few frames captured")
        return 2

    all_iv, distinct_iv, dup_iv = [], [], []
    bursts = gaps = dup_count = 0
    last_t, last_payload = frames[0]
    last_distinct_t = frames[0][0]
    for t, payload in frames[1:]:
        iv = (t - last_t) * 1000.0
        all_iv.append(iv)
        if iv < 3.0:
            bursts += 1
        if payload == last_payload:
            dup_count += 1
            dup_iv.append(iv)
        else:
            div = (t - last_distinct_t) * 1000.0
            distinct_iv.append(div)
            if div > 25.0:
                gaps += 1
            last_distinct_t = t
        last_t, last_payload = t, payload

    print(f"captured {len(frames)} frames, {dup_count} duplicate payloads (keepalives)")
    report("all frames     ", all_iv)
    distinct_stats = report("distinct updates", distinct_iv)
    if dup_iv:
        report("keepalives     ", dup_iv)
    print(f"bursts (<3 ms): {bursts}   gaps (>25 ms between distinct updates): {gaps}")

    ok = bursts == 0
    if distinct_stats:
        med, p99 = distinct_stats
        if med > 0 and p99 > 1.5 * med:
            print(f"FAIL contribution: distinct p99 {p99:.1f} > 1.5x median {med:.1f}")
            ok = False
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
