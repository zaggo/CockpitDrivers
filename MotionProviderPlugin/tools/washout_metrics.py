#!/usr/bin/env python3
"""Metrics for motion-platform tuning runs.

Reads telemetry CSVs (from the plugin or from washout_replay) and prints one
row per file. Everything is numpy-only so it runs wherever numpy is installed.

Every column is required except dt_real, g_nrml, and reach_scale (the three
that pass an explicit `default=` to `column()`): a file missing one of them --
an older recorder, a truncated capture -- aborts with a clear error instead
of silently reporting a zero. A missing heave_clamped must never look
identical to a run with zero saturation.

Metric notes:

  wrms        RMS of heave acceleration band-limited to 0.1-0.63 Hz (Hann-
              windowed FFT mask, RMS-corrected for the window's power loss so
              the number stays meaningful). This is a documented band
              emphasis, NOT a conformant ISO-2631 Wk weighting. It is used
              only to rank candidates against each other.
  band_ratio  Fraction of heave-acceleration spectral power inside that same
              0.1-0.63 Hz band (Hann-windowed to limit edge leakage).
  jerk_p95    95th-percentile |third difference| of the six streamed BFF
              demand channels, normalised to counts/s^3 via the file's own
              fs so runs recorded at different frame rates stay comparable.
              Still raw demand-count units -- no counts-to-mm conversion is
              available here -- so treat it as comparative, like wrms.
  lag_ms      Shift maximising the Pearson-normalised cross-correlation
              between the drive cue and the commanded pose, computed after
              band-limiting BOTH signals to LAG_LO_HZ-LAG_HI_HZ (0.3-1 Hz) --
              a narrower, higher band than wrms's, chosen so a single run
              gives enough cycles to converge. The washout is a high-pass,
              not a delay line, so the absolute value is not a physical
              latency -- the meaningful quantity is the DELTA against the
              baseline run. Refuses (NaN, with a stderr warning) when the
              recording is too short to give LAG_MIN_PERIODS cycles at
              LAG_LO_HZ, or when either signal has no energy left in that
              band (e.g. heave frozen against a clamp) -- a stuck signal must
              never read as "zero added lag".
  sat_*       Percentage of *unpaused* ticks (the required `paused` column
              equal to 0) where the corresponding clamp/clip flag was set.
              While the sim is paused the recorder keeps writing rows, but
              the washout trace just holds whatever the last unpaused tick
              left behind (a frozen heave_clamped, a frozen tilt_rate_active,
              ...); counting those rows would let time spent on a loading
              screen or a menu inflate sat_heave, the campaign's headline
              number. If a file is paused throughout, the unpaused-row count
              is zero and every sat_* is NaN (with a stderr warning naming
              the file) rather than a divide-by-zero or a misleadingly clean
              0%.

              wrms, band_ratio, jerk_p95, and lag_ms are deliberately NOT
              filtered this way: they operate on the full time series, and
              excising scattered paused rows would corrupt the time axis
              feeding the derivative/FFT/cross-correlation math -- worse
              than the problem it would solve. So sat_* and the
              spectral/lag metrics are not measured over identical sample
              sets; that is intentional, not an oversight.
"""
import argparse
import csv
import sys

import numpy as np

BAND_LO_HZ = 0.1
BAND_HI_HZ = 0.63

# Lag estimation uses its own, narrower/higher band: it needs several cycles
# to converge within one run, and a frequency change between candidate runs
# (e.g. retuning the washout corner) shifts the finite-window bias of a
# broadband estimate enough to sit on top of the campaign's lag-delta gate.
LAG_LO_HZ = 0.3
LAG_HI_HZ = 1.0
LAG_MIN_PERIODS = 8

# Columns allowed to fall back to a default when absent from a CSV. Every
# other column is required -- see column() and the module docstring.
OPTIONAL_COLUMNS = {"dt_real", "g_nrml", "reach_scale"}


def load(path):
    with open(path, newline="") as fh:
        rows = list(csv.reader(fh))
    header, data = rows[0], rows[1:]
    cols = {name: i for i, name in enumerate(header)}
    arr = np.array([[float(v) for v in r] for r in data if r], dtype=float)
    return cols, arr


def column(cols, arr, name, default=0.0, path="<unknown>"):
    """Fetch column `name`.

    Only names in OPTIONAL_COLUMNS may fall back to `default` when the
    column is missing. Every other column is required: its absence aborts
    the run with a message naming the file and the column, rather than
    silently returning a zero-filled array that a caller could mistake for a
    genuine (and, for a saturation metric, favourable) measurement.
    """
    if name not in cols:
        if name in OPTIONAL_COLUMNS:
            return np.full(arr.shape[0], default)
        raise SystemExit(f"{path}: missing required column '{name}'")
    return arr[:, cols[name]]


def _windowed_spectrum(signal, fs):
    """Hann-windowed FFT of `signal` (mean removed first).

    Returns (spec, freqs, rms_correction): rms_correction undoes the RMS
    reduction caused by the taper (1/sqrt(mean(window**2)), ~1.63 for Hann)
    so a signal reconstructed from a masked version of `spec` still reports
    a meaningful RMS.
    """
    n = signal.size
    window = np.hanning(n)
    spec = np.fft.rfft((signal - signal.mean()) * window)
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    rms_correction = 1.0 / np.sqrt(np.mean(window ** 2))
    return spec, freqs, rms_correction


def band_limit(signal, fs, lo, hi):
    """Time-domain `signal` restricted to the [lo, hi] Hz band."""
    n = signal.size
    spec, freqs, rms_correction = _windowed_spectrum(signal, fs)
    spec[(freqs < lo) | (freqs > hi)] = 0.0
    return np.fft.irfft(spec, n) * rms_correction


def band_limited_rms(signal, fs, lo, hi):
    """RMS of `signal` after keeping only the [lo, hi] Hz band."""
    filtered = band_limit(signal, fs, lo, hi)
    return float(np.sqrt(np.mean(filtered ** 2)))


def band_ratio(signal, fs, lo, hi):
    spec, freqs, _ = _windowed_spectrum(signal, fs)
    power = np.abs(spec) ** 2
    total = power.sum()
    if total <= 0.0:
        return 0.0
    return float(power[(freqs >= lo) & (freqs <= hi)].sum() / total)


def xcorr_lag_ms(drive, response, fs, max_lag_sec=1.0):
    """Shift (ms) of `response` relative to `drive` maximising the
    Pearson-normalised cross-correlation, after band-limiting both signals
    to [LAG_LO_HZ, LAG_HI_HZ]. See the module docstring for the refusal
    conditions (NaN)."""
    n = drive.size
    periods_at_floor = (n / fs) * LAG_LO_HZ
    if periods_at_floor < LAG_MIN_PERIODS:
        print(f"xcorr_lag_ms: only {periods_at_floor:.1f} cycles at "
              f"{LAG_LO_HZ} Hz available (need {LAG_MIN_PERIODS}); "
              f"refusing to estimate lag_ms", file=sys.stderr)
        return float("nan")

    d = band_limit(drive, fs, LAG_LO_HZ, LAG_HI_HZ)
    r = band_limit(response, fs, LAG_LO_HZ, LAG_HI_HZ)
    if d.std() == 0 or r.std() == 0:
        print("xcorr_lag_ms: no signal energy in the lag band on one or "
              "both channels; refusing to estimate lag_ms", file=sys.stderr)
        return float("nan")

    max_lag = int(max_lag_sec * fs)
    best_lag, best_val = 0, -np.inf
    for lag in range(0, max_lag + 1):
        a = d[: d.size - lag]
        b = r[lag:]
        if a.size < 16:
            break
        na, nb = np.linalg.norm(a), np.linalg.norm(b)
        if na == 0.0 or nb == 0.0:
            continue
        val = float(np.dot(a, b) / (na * nb))
        if val > best_val:
            best_val, best_lag = val, lag
    return 1000.0 * best_lag / fs


def metrics(path):
    cols, arr = load(path)
    if arr.shape[0] < 8:
        raise SystemExit(f"{path}: too few rows")

    dt = column(cols, arr, "dt_real", 1.0 / 60.0, path=path)
    fs = 1.0 / float(np.median(dt))
    n = arr.shape[0]

    # Sample-time axis built from the real per-row dt_real, not a uniform
    # 1/fs spacing: for live plugin recordings, frame-time jitter correlates
    # with the high-workload manoeuvres under study, so a uniform-spacing
    # assumption would distort acceleration exactly when it matters most.
    t = np.cumsum(dt)

    heave_mm = column(cols, arr, "live_heave", path=path)
    heave_m = heave_mm / 1000.0
    accel = np.gradient(np.gradient(heave_m, t), t)

    g_cue = column(cols, arr, "g_nrml", 1.0, path=path) - 1.0

    # sat_* is measured over unpaused ticks only -- see the module docstring.
    # `paused` is required (not in OPTIONAL_COLUMNS): an older recorder
    # missing it must abort, not silently behave as "never paused".
    paused = column(cols, arr, "paused", path=path)
    unpaused = paused < 0.5
    n_unpaused = int(unpaused.sum())
    if n_unpaused == 0:
        print(f"{path}: every row is paused; sat_* metrics are undefined "
              f"(NaN)", file=sys.stderr)

    def sat_pct(flags):
        if n_unpaused == 0:
            return float("nan")
        return 100.0 * flags[unpaused].mean()

    sat = {
        "heave": sat_pct(column(cols, arr, "heave_clamped", path=path)),
        "rot_r": sat_pct(column(cols, arr, "rot_roll_clamped", path=path)),
        "rot_p": sat_pct(column(cols, arr, "rot_pitch_clamped", path=path)),
        "rot_y": sat_pct(column(cols, arr, "rot_yaw_clamped", path=path)),
        "tilt_rate": sat_pct(column(cols, arr, "tilt_rate_active", path=path)),
        "envelope": sat_pct(column(cols, arr, "reach_scale", 1.0, path=path) < 1.0),
        "sl_vel": sat_pct(column(cols, arr, "sl_vel_clip", path=path) > 0),
        "sl_acc": sat_pct(column(cols, arr, "sl_acc_clip", path=path) > 0),
    }

    jerks = []
    for i in range(6):
        sent = column(cols, arr, f"sent{i}", path=path)
        if sent.size > 3:
            jerks.append(np.percentile(np.abs(np.diff(sent, n=3)), 95))
    # Normalise by (1/fs)**3 so this is counts/s^3 rather than a raw
    # per-sample third difference, which would otherwise scale with each
    # file's own frame rate and make cross-file comparison meaningless.
    jerk_p95 = float(max(jerks)) / (1.0 / fs) ** 3 if jerks else 0.0

    peak_raw = float(np.max(np.abs(column(cols, arr, "heave_pos_raw", path=path))))
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
