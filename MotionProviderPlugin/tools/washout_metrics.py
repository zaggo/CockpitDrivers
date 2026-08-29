#!/usr/bin/env python3
"""Metrics for motion-platform tuning runs.

Reads telemetry CSVs (from the plugin or from washout_replay) and prints one
row per file. Everything is numpy-only so it runs wherever numpy is installed.

Metric notes:

  wrms       RMS of heave acceleration band-limited to 0.1-0.63 Hz. This is a
             documented band emphasis, NOT a conformant ISO-2631 Wk weighting.
             It is used only to rank candidates against each other.
  lag_ms     Shift maximising the cross-correlation between the drive cue and
             the commanded pose. The washout is a high-pass, not a delay line,
             so the absolute value is not a physical latency -- the meaningful
             quantity is the DELTA against the baseline run.
"""
import argparse
import csv
import sys

import numpy as np

BAND_LO_HZ = 0.1
BAND_HI_HZ = 0.63


def load(path):
    with open(path, newline="") as fh:
        rows = list(csv.reader(fh))
    header, data = rows[0], rows[1:]
    cols = {name: i for i, name in enumerate(header)}
    arr = np.array([[float(v) for v in r] for r in data if r], dtype=float)
    return cols, arr


def column(cols, arr, name, default=0.0):
    if name not in cols:
        return np.full(arr.shape[0], default)
    return arr[:, cols[name]]


def band_limited_rms(signal, fs, lo, hi):
    """RMS of `signal` after keeping only the [lo, hi] Hz band."""
    n = signal.size
    if n < 4:
        return 0.0
    spec = np.fft.rfft(signal - signal.mean())
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    spec[(freqs < lo) | (freqs > hi)] = 0.0
    return float(np.sqrt(np.mean(np.fft.irfft(spec, n) ** 2)))


def band_ratio(signal, fs, lo, hi):
    n = signal.size
    if n < 4:
        return 0.0
    spec = np.abs(np.fft.rfft(signal - signal.mean())) ** 2
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    total = spec.sum()
    if total <= 0.0:
        return 0.0
    return float(spec[(freqs >= lo) & (freqs <= hi)].sum() / total)


def xcorr_lag_ms(drive, response, fs, max_lag_sec=1.0):
    """Shift (ms) of `response` relative to `drive` maximising correlation."""
    d = drive - drive.mean()
    r = response - response.mean()
    if d.std() == 0 or r.std() == 0:
        return 0.0
    max_lag = int(max_lag_sec * fs)
    best_lag, best_val = 0, -np.inf
    for lag in range(0, max_lag + 1):
        a = d[: d.size - lag]
        b = r[lag:]
        if a.size < 16:
            break
        val = float(np.dot(a, b) / a.size)
        if val > best_val:
            best_val, best_lag = val, lag
    return 1000.0 * best_lag / fs


def metrics(path):
    cols, arr = load(path)
    if arr.shape[0] < 8:
        raise SystemExit(f"{path}: too few rows")

    dt = column(cols, arr, "dt_real", 1.0 / 60.0)
    fs = 1.0 / float(np.median(dt))
    n = arr.shape[0]

    heave_mm = column(cols, arr, "live_heave")
    heave_m = heave_mm / 1000.0
    # Second difference -> acceleration. Uniform-dt assumption is fine here:
    # dt jitter is a few percent and this is a comparative metric.
    accel = np.gradient(np.gradient(heave_m, 1.0 / fs), 1.0 / fs)

    g_cue = column(cols, arr, "g_nrml", 1.0) - 1.0

    sat = {
        "heave": 100.0 * column(cols, arr, "heave_clamped").mean(),
        "rot_r": 100.0 * column(cols, arr, "rot_roll_clamped").mean(),
        "rot_p": 100.0 * column(cols, arr, "rot_pitch_clamped").mean(),
        "rot_y": 100.0 * column(cols, arr, "rot_yaw_clamped").mean(),
        "tilt_rate": 100.0 * column(cols, arr, "tilt_rate_active").mean(),
        "envelope": 100.0 * (column(cols, arr, "reach_scale", 1.0) < 1.0).mean(),
        "sl_vel": 100.0 * (column(cols, arr, "sl_vel_clip") > 0).mean(),
        "sl_acc": 100.0 * (column(cols, arr, "sl_acc_clip") > 0).mean(),
    }

    jerks = []
    for i in range(6):
        sent = column(cols, arr, f"sent{i}")
        if sent.size > 3:
            jerks.append(np.percentile(np.abs(np.diff(sent, n=3)), 95))
    jerk_p95 = float(max(jerks)) if jerks else 0.0

    peak_raw = float(np.max(np.abs(column(cols, arr, "heave_pos_raw"))))
    limit_hint = float(np.max(np.abs(heave_mm)))

    return {
        "file": path,
        "rows": n,
        "sec": float(np.sum(dt)),
        "fs": fs,
        "sat_heave": sat["heave"],
        "sat_rot": max(sat["rot_r"], sat["rot_p"], sat["rot_y"]),
        "sat_tilt_rate": sat["tilt_rate"],
        "sat_envelope": sat["envelope"],
        "sat_sl_vel": sat["sl_vel"],
        "sat_sl_acc": sat["sl_acc"],
        "wrms": band_limited_rms(accel, fs, BAND_LO_HZ, BAND_HI_HZ),
        "band_ratio": band_ratio(accel, fs, BAND_LO_HZ, BAND_HI_HZ),
        "jerk_p95": jerk_p95,
        "lag_ms": xcorr_lag_ms(g_cue, heave_mm, fs),
        "peak_raw_mm": peak_raw,
        "peak_out_mm": limit_hint,
    }


# (column name, header format, value format)
HEADERS = [
    ("file", "{:<34}", "{:<34}"),
    ("sec", "{:>7}", "{:>7.1f}"),
    ("sat_heave", "{:>10}", "{:>10.2f}"),
    ("sat_rot", "{:>8}", "{:>8.2f}"),
    ("sat_envelope", "{:>13}", "{:>13.2f}"),
    ("sat_sl_acc", "{:>11}", "{:>11.2f}"),
    ("wrms", "{:>9}", "{:>9.4f}"),
    ("band_ratio", "{:>11}", "{:>11.3f}"),
    ("jerk_p95", "{:>9}", "{:>9.1f}"),
    ("lag_ms", "{:>8}", "{:>8.1f}"),
    ("peak_raw_mm", "{:>12}", "{:>12.1f}"),
]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+")
    ap.add_argument("--csv", action="store_true", help="emit CSV instead of a table")
    args = ap.parse_args()

    results = [metrics(p) for p in args.files]

    if args.csv:
        w = csv.DictWriter(sys.stdout, fieldnames=list(results[0].keys()))
        w.writeheader()
        w.writerows(results)
        return

    print("".join(hfmt.format(name) for name, hfmt, _ in HEADERS))
    for r in results:
        print("".join(vfmt.format(r[name]) for name, _, vfmt in HEADERS))


if __name__ == "__main__":
    main()
