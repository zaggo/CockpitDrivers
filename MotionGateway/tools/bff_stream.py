#!/usr/bin/env python3
"""Stream BFF demand frames to the MotionGateway at a precise, fixed cadence.

Replaces X-Plane/MotionProviderPlugin as the serial source for stage-2 timing
tests: 16-byte frames ('B' 'C' 0x00, 6x MSB, 6x LSB, 0x0D) with all six
actuators following the same triangle or sine wave. Uses absolute-deadline
scheduling on time.monotonic(), so sender jitter stays out of the measurement.

Requires pyserial:  pip install pyserial

Usage:
    python3 bff_stream.py /dev/tty.usbmodemXXXX [--rate 60] [--wave tri|sine]
                          [--amp 30] [--duration 300] [--settle 5]
"""

import argparse
import math
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial missing: pip install pyserial")
    sys.exit(2)

DEMAND_MAX = 65280
DEMAND_HOME = 32640
WAVE_PERIOD_S = 6.0  # full peak-to-peak traversal in 3 s per leg, like the benches


def wave_value(elapsed_s, wave, amp_pct):
    half_amp = DEMAND_MAX * amp_pct / 200.0
    phase = (elapsed_s % WAVE_PERIOD_S) / WAVE_PERIOD_S
    if wave == "sine":
        offset = half_amp * math.sin(2 * math.pi * phase)
    else:
        offset = (-half_amp + 4 * half_amp * phase) if phase < 0.5 else (3 * half_amp - 4 * half_amp * phase)
    value = int(round(DEMAND_HOME + offset))
    return max(0, min(DEMAND_MAX, value))


def encode_frame(demands):
    frame = bytearray(16)
    frame[0] = ord("B")
    frame[1] = ord("C")
    frame[2] = 0x00
    for i, d in enumerate(demands):
        frame[3 + i] = (d >> 8) & 0xFF
        frame[9 + i] = d & 0xFF
    frame[15] = 0x0D
    return bytes(frame)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", help="gateway USB serial port")
    ap.add_argument("--rate", type=float, default=60.0, help="frame rate in Hz (default 60)")
    ap.add_argument("--wave", choices=["tri", "sine"], default="tri")
    ap.add_argument("--amp", type=float, default=30.0, help="amplitude in %% of full range")
    ap.add_argument("--duration", type=float, default=300.0, help="seconds to stream")
    ap.add_argument("--settle", type=float, default=5.0,
                    help="seconds to wait after opening the port (AVR reset + gateway LED boot delay)")
    args = ap.parse_args()

    port = serial.Serial(args.port, 115200, timeout=0)
    print(f"Opened {args.port}, settling {args.settle:.1f} s (gateway boots ~4.5 s of LED cycling)...")
    time.sleep(args.settle)

    period = 1.0 / args.rate
    start = time.monotonic()
    next_deadline = start
    sent = 0
    late = 0
    max_late = 0.0

    print(f"Streaming {args.wave} @ {args.rate:.0f} Hz, amp {args.amp:.0f}%, {args.duration:.0f} s. Ctrl-C stops.")
    try:
        while True:
            now = time.monotonic()
            if now - start >= args.duration:
                break
            if now < next_deadline:
                time.sleep(min(next_deadline - now, 0.005))
                continue
            lateness = now - next_deadline
            if lateness > period / 2:
                late += 1
                max_late = max(max_late, lateness)
            value = wave_value(now - start, args.wave, args.amp)
            port.write(encode_frame([value] * 6))
            sent += 1
            next_deadline += period
            if next_deadline < now - period:  # fell far behind (USB stall): resync
                next_deadline = now + period
            # drain the gateway's HB frames so the OS buffer never fills
            port.read(64)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        # park at home before leaving
        port.write(encode_frame([DEMAND_HOME] * 6))
        port.close()

    elapsed = time.monotonic() - start
    print(f"Sent {sent} frames in {elapsed:.1f} s ({sent / elapsed:.2f} Hz effective), "
          f"{late} late ticks (max {max_late * 1000:.1f} ms).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
