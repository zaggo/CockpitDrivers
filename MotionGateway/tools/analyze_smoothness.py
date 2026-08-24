#!/usr/bin/env python3
"""Analyze a MotionActor test-bench dump captured from the gateway bench console.

Input: a text file containing the TH/TD/TS CSV lines the gateway prints after a
`td <node>` dump request (other console lines are ignored):

    TH,<node>,<strategy>,<sampleKind>,<sampleCount>,<rateHz>,<chmask>,<result>
    TD,<node>,<seq>,<tOffsetMs>,<value>
    TS,<node>,<state>,<cyclesDone>,<maxCmdDurUs10>,<missedCycles>

sampleKind 1 = position series (value = 0..65535 across the logical stroke),
sampleKind 2 = timing series (value = command duration in 10 us units, sample
timestamps give the achieved cycle cadence).

Smoothness metric (position series): the motion is segmented into legs, each leg
interior is fitted with a quadratic, and the RMS of the fit residuals is compared
against the commanded step size (mean velocity x demand period). A smooth
Kangaroo ramp leaves residuals at the sensor-noise level (ratio ~0.05); a
stop-start staircase leaves residuals of about half a step (ratio ~0.4). This is
robust against getP quantization noise, which velocity-ratio metrics are not.

Usage:
    python3 analyze_smoothness.py run.csv [--residual-max 0.25] [--plot]

Exit code 0 = PASS, 1 = FAIL, 2 = input error.
"""

import argparse
import math
import sys


def parse_capture(path):
    header = None
    samples = []
    status = None
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if line.startswith("TH,"):
                p = line.split(",")
                if len(p) >= 8:
                    header = {
                        "node": int(p[1]),
                        "strategy": int(p[2]),
                        "kind": int(p[3]),
                        "count": int(p[4]),
                        "rate_hz": int(p[5]),
                        "chmask": int(p[6]),
                        "result": int(p[7]),
                    }
                    samples = []  # a new header starts a new dump
            elif line.startswith("TD,"):
                p = line.split(",")
                if len(p) >= 5:
                    samples.append((int(p[2]), int(p[3]), int(p[4])))  # seq, t, v
            elif line.startswith("TS,"):
                p = line.split(",")
                if len(p) >= 6:
                    status = {
                        "state": int(p[2]),
                        "cycles": int(p[3]),
                        "max_cmd_us": int(p[4]) * 10,
                        "missed": int(p[5]),
                    }
    return header, samples, status


def check_sequence(samples):
    missing = 0
    for i in range(1, len(samples)):
        expected = (samples[i - 1][0] + 1) % 256
        got = samples[i][0]
        if got != expected:
            missing += (got - expected) % 256
    return missing


def percentile(values, pct):
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = min(int(len(ordered) * pct), len(ordered) - 1)
    return ordered[idx]


def dewrap_times(samples):
    pts = [(t, v) for _, t, v in samples]
    for i in range(1, len(pts)):
        if pts[i][0] < pts[i - 1][0]:
            pts[i] = (pts[i][0] + 65536, pts[i][1])
    return pts


def fit_quadratic(points):
    """Least-squares quadratic fit around the centered time axis.

    Returns (predict(t), velocity_at(t) in counts/ms). Falls back to a linear
    fit if the normal-equation matrix is singular.
    """
    t0 = sum(t for t, _ in points) / len(points)
    xs = [t - t0 for t, _ in points]
    ys = [v for _, v in points]
    n = len(xs)
    s1 = sum(xs)
    s2 = sum(x * x for x in xs)
    s3 = sum(x ** 3 for x in xs)
    s4 = sum(x ** 4 for x in xs)
    b0 = sum(ys)
    b1 = sum(x * y for x, y in zip(xs, ys))
    b2 = sum(x * x * y for x, y in zip(xs, ys))

    # Solve [[n,s1,s2],[s1,s2,s3],[s2,s3,s4]] * c = [b0,b1,b2] via Cramer's rule.
    def det3(m):
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))

    A = [[n, s1, s2], [s1, s2, s3], [s2, s3, s4]]
    d = det3(A)
    if abs(d) < 1e-9:
        # Linear fallback
        denom = n * s2 - s1 * s1
        if abs(denom) < 1e-9:
            return None
        slope = (n * b1 - s1 * b0) / denom
        inter = (b0 - slope * s1) / n
        return (lambda t: inter + slope * (t - t0)), (lambda t: slope)

    def col_replaced(k, rhs):
        return [[rhs[r] if c == k else A[r][c] for c in range(3)] for r in range(3)]

    rhs = [b0, b1, b2]
    c0 = det3(col_replaced(0, rhs)) / d
    c1 = det3(col_replaced(1, rhs)) / d
    c2 = det3(col_replaced(2, rhs)) / d
    return (lambda t: c0 + c1 * (t - t0) + c2 * (t - t0) ** 2), \
           (lambda t: c1 + 2 * c2 * (t - t0))


def analyze_position(samples, header, args):
    pts = dewrap_times(samples)
    if len(pts) < 16:
        print("Not enough position samples for analysis.")
        return None

    # Segment into legs using the sign of a lightly smoothed velocity.
    vel = []
    for i in range(1, len(pts)):
        dt = pts[i][0] - pts[i - 1][0]
        if dt <= 0:
            continue
        vel.append(((pts[i][0] + pts[i - 1][0]) / 2.0, (pts[i][1] - pts[i - 1][1]) / dt))
    smoothed = []
    for i in range(len(vel)):
        window = [vel[j][1] for j in range(max(0, i - 2), min(len(vel), i + 3))]
        smoothed.append(sum(window) / len(window))

    legs = []
    start = 0
    for i in range(1, len(smoothed)):
        if (smoothed[i] > 0) != (smoothed[start] > 0):
            if i - start >= 8:
                legs.append((start, i))
            start = i
    if len(smoothed) - start >= 8:
        legs.append((start, len(smoothed)))
    if not legs:
        print("Could not segment any movement leg.")
        return None

    demand_period_ms = 1000.0 / header["rate_hz"] if header["rate_hz"] else 0.0

    worst_ratio = 0.0
    total_stalls = 0
    total_reversals = 0
    velocities = []
    resid_all = []
    for s, e in legs:
        # Leg indices refer to vel[]; map back to pts[] (vel[i] spans pts[i]..pts[i+1]).
        n = e - s
        lo = s + n // 5
        hi = e - n // 5
        leg_pts = pts[lo:hi + 1]
        if len(leg_pts) < 8:
            continue

        fit = fit_quadratic(leg_pts)
        if fit is None:
            continue
        predict, velocity_at = fit

        residuals = [v - predict(t) for t, v in leg_pts]
        rms = math.sqrt(sum(r * r for r in residuals) / len(residuals))
        resid_all.extend(residuals)

        t_mid = (leg_pts[0][0] + leg_pts[-1][0]) / 2.0
        v_mid = velocity_at(t_mid)  # counts/ms
        velocities.append(abs(v_mid) * 1000.0)
        if demand_period_ms > 0 and abs(v_mid) > 1e-6:
            step = abs(v_mid) * demand_period_ms
            worst_ratio = max(worst_ratio, rms / step)

        # Stalls: >=2 consecutive samples that move <20% of the expected step.
        # Reversals: a sample moving >50% of a step against the leg direction.
        direction = 1.0 if v_mid > 0 else -1.0
        run = 0
        for i in range(1, len(leg_pts)):
            dt = leg_pts[i][0] - leg_pts[i - 1][0]
            dpos = leg_pts[i][1] - leg_pts[i - 1][1]
            expected = abs(velocity_at(leg_pts[i][0])) * dt
            if expected < 1e-6:
                continue
            if abs(dpos) < 0.2 * expected:
                run += 1
                if run == 2:
                    total_stalls += 1
            else:
                run = 0
            if dpos * direction < -0.5 * expected:
                total_reversals += 1

    if not velocities:
        print("No usable legs after fitting.")
        return None

    mean_v = sum(velocities) / len(velocities)
    noise_rms = math.sqrt(sum(r * r for r in resid_all) / len(resid_all))
    dts = [pts[i][0] - pts[i - 1][0] for i in range(1, len(pts))]

    print(f"Position series: {len(pts)} samples, {len(legs)} legs")
    print(f"  mean leg velocity    : {mean_v:8.0f} counts/s (0..65535 scale)")
    print(f"  residual RMS         : {noise_rms:6.1f} counts")
    if demand_period_ms > 0:
        print(f"  residual / step size : {worst_ratio:6.3f}  (limit {args.residual_max:.2f}; "
              f"step = vmean x {demand_period_ms:.1f} ms)")
    print(f"  stalls (>=2 samples <20% step): {total_stalls}")
    print(f"  reversals (>50% step backwards): {total_reversals}")
    print(f"  sample dt mean/p95/max: {sum(dts)/len(dts):.1f} / {percentile(dts, 0.95)} / {max(dts)} ms")

    ok = worst_ratio <= args.residual_max and total_stalls == 0 and total_reversals == 0
    return ok, pts, vel


def analyze_timing(samples, header, args):
    pts = dewrap_times(samples)
    periods = [pts[i][0] - pts[i - 1][0] for i in range(1, len(pts))]
    durations_us = [v * 10 for _, v in pts]
    if not periods:
        print("Not enough timing samples.")
        return None

    nominal_ms = 1000.0 / header["rate_hz"] if header["rate_hz"] else 0.0
    mean_p = sum(periods) / len(periods)
    p95 = percentile(periods, 0.95)
    print(f"Timing series: {len(pts)} cycles, nominal period {nominal_ms:.1f} ms")
    print(f"  period mean/p95/max: {mean_p:.1f} / {p95} / {max(periods)} ms")
    print(f"  cmd duration mean/max: {sum(durations_us)/len(durations_us)/1000:.2f} / {max(durations_us)/1000:.2f} ms")

    ok = nominal_ms == 0.0 or p95 <= args.cadence_p95_factor * nominal_ms
    return ok, pts, None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture", help="console capture containing TH/TD/TS lines")
    ap.add_argument("--residual-max", type=float, default=0.25,
                    help="max fit-residual RMS as fraction of one commanded step (default 0.25)")
    ap.add_argument("--cadence-p95-factor", type=float, default=1.2, help="p95 period limit vs nominal")
    ap.add_argument("--plot", action="store_true", help="show matplotlib plots (optional)")
    args = ap.parse_args()

    header, samples, status = parse_capture(args.capture)
    if header is None:
        print("No TH header line found - is this a bench console capture?")
        return 2
    if not samples:
        print("No TD sample lines found.")
        return 2

    strategy_names = {0: "single-move", 1: "current-algo", 2: "exact-speed",
                      3: "stream+current", 4: "stream+exact", 9: "passthrough"}
    print(f"Node {header['node']}, strategy {header['strategy']} "
          f"({strategy_names.get(header['strategy'], '?')}), rate {header['rate_hz']} Hz, "
          f"chmask {header['chmask']}, result {header['result']}")

    missing = check_sequence(samples)
    if missing:
        print(f"WARNING: {missing} dump frames missing (seq gaps) - metrics may be skewed")
    if header["count"] != len(samples):
        print(f"WARNING: header announced {header['count']} samples, captured {len(samples)}")

    if status:
        print(f"Status: cycles {status['cycles']}, missed {status['missed']}, "
              f"max cmd duration {status['max_cmd_us']/1000:.2f} ms")

    if header["kind"] == 1:
        result = analyze_position(samples, header, args)
    elif header["kind"] == 2:
        result = analyze_timing(samples, header, args)
    else:
        print("Unknown sampleKind - nothing to analyze.")
        return 2
    if result is None:
        return 2
    ok, pts, vel = result

    if status and status["missed"] > 0:
        print(f"FAIL contribution: {status['missed']} missed cycles")
        ok = False

    if args.plot:
        try:
            import matplotlib.pyplot as plt
            fig, axes = plt.subplots(2 if vel else 1, 1, sharex=True)
            axes = axes if hasattr(axes, "__len__") else [axes]
            axes[0].plot([t for t, _ in pts], [v for _, v in pts], ".-", ms=3)
            axes[0].set_ylabel("value")
            if vel:
                axes[1].plot([t for t, _ in vel], [v * 1000 for _, v in vel], ".-", ms=3)
                axes[1].set_ylabel("counts/s")
                axes[1].set_xlabel("t [ms]")
            plt.suptitle(f"strategy {header['strategy']} @ {header['rate_hz']} Hz")
            plt.show()
        except ImportError:
            print("(matplotlib not installed - skipping plot)")

    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
