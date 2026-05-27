#!/usr/bin/env python3
"""
Aggregate and compare HARM logs produced by the σ-sweep experiment.

The NR scenario emits one CSV per run via the --harm-log-file flag,
with columns:

    time,car1ID,car2ID,harm

This script:
  1. discovers every harm_log_*.csv in the working directory,
  2. for each, computes the per-tick sum and the time-integrated total,
  3. prints a comparison table (baseline + each σ value),
  4. optionally plots Σ harm(t) for visual inspection.

Usage:
    python3 sigma_sweep_analyze.py [--glob PATTERN] [--plot]

Conventions:
  - File named harm_log_baseline.csv is treated as the no-algo baseline.
  - File named harm_log_sigma_<value>.csv contributes the σ point <value>.
  - All other matched files are listed under their stem.

Time integration uses the trapezoidal rule on the per-tick sums.
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import sys
from collections import defaultdict


def load_harm_log(path: str):
    """Return (sorted_times, sum_per_tick) for one CSV."""
    sums: dict[float, float] = defaultdict(float)
    with open(path, "r") as f:
        header = f.readline()
        if not header.lower().startswith("time"):
            raise ValueError(f"{path}: unexpected header {header!r}")
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            if len(parts) < 4:
                continue
            try:
                t = float(parts[0])
                h = float(parts[3])
            except ValueError:
                continue
            sums[t] += h
    if not sums:
        return [], []
    ts = sorted(sums)
    return ts, [sums[t] for t in ts]


def trapezoid(xs, ys) -> float:
    if len(xs) < 2:
        return 0.0
    out = 0.0
    for i in range(1, len(xs)):
        out += 0.5 * (ys[i - 1] + ys[i]) * (xs[i] - xs[i - 1])
    return out


def label_for(path: str) -> tuple[str, float | None]:
    """Map filename to (label, sigma_value-or-None)."""
    stem = os.path.splitext(os.path.basename(path))[0]
    if stem == "harm_log_baseline":
        return "baseline", None
    m = re.match(r"harm_log_sigma_([0-9eE+\-.]+)", stem)
    if m:
        try:
            return f"σ={m.group(1)}", float(m.group(1))
        except ValueError:
            return stem, None
    return stem, None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--glob", default="harm_log_*.csv",
                    help="Pattern for HARM log files (default: harm_log_*.csv)")
    ap.add_argument("--plot", action="store_true",
                    help="Render Σ harm(t) curves with matplotlib")
    args = ap.parse_args()

    files = sorted(glob.glob(args.glob))
    if not files:
        print(f"No files matched {args.glob}", file=sys.stderr)
        return 1

    rows = []   # (sigma_or_None, label, integral, peak, n_ticks, path)
    series = {} # label -> (ts, ys)
    skipped = []
    for path in files:
        label, sigma = label_for(path)
        try:
            ts, ys = load_harm_log(path)
        except Exception as e:
            print(f"  {path}: parse error {e}", file=sys.stderr)
            skipped.append((path, f"parse error: {e}"))
            continue
        if not ts:
            skipped.append((path, "empty"))
            continue
        integral = trapezoid(ts, ys)
        peak = max(ys)
        rows.append((sigma, label, integral, peak, len(ts), path))
        series[label] = (ts, ys)

    if not rows:
        print(f"\nAll {len(files)} matching files were empty or unreadable.",
              file=sys.stderr)
        for path, why in skipped:
            print(f"  {path}: {why}", file=sys.stderr)
        return 1
    if skipped:
        print(f"Skipped {len(skipped)} file(s):", file=sys.stderr)
        for path, why in skipped:
            print(f"  {path}: {why}", file=sys.stderr)

    # Sort: baseline first, then σ values ascending, then others alphabetically.
    def sort_key(r):
        sigma, label, *_ = r
        if label == "baseline":
            return (0, 0.0, "")
        if sigma is not None:
            return (1, sigma, label)
        return (2, 0.0, label)
    rows.sort(key=sort_key)

    baseline_integral = next((r[2] for r in rows if r[1] == "baseline"), None)

    # Print comparison table.
    print(f"{'label':<14} {'integral':>12} {'peak':>10} {'ticks':>7}  {'Δvs baseline':>14}")
    print("-" * 64)
    for sigma, label, integral, peak, ntick, _ in rows:
        delta_str = ""
        if baseline_integral is not None and label != "baseline":
            delta = integral - baseline_integral
            pct = (delta / baseline_integral * 100.0) if baseline_integral else 0.0
            delta_str = f"{delta:+.2f} ({pct:+.1f}%)"
        print(f"{label:<14} {integral:>12.3f} {peak:>10.3f} {ntick:>7d}  {delta_str:>14}")

    if args.plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not available; skipping --plot", file=sys.stderr)
            return 0
        fig, ax = plt.subplots(figsize=(9, 5))
        for sigma, label, *_ in rows:
            ts, ys = series[label]
            ax.plot(ts, ys, label=label, linewidth=1.0)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Σ pairwise HARM [m/s]")
        ax.set_title("Pairwise HARM, baseline vs σ sweep")
        ax.legend(loc="best", fontsize=8)
        ax.grid(True, alpha=0.3)
        out = "sigma_sweep_harm.png"
        fig.tight_layout()
        fig.savefig(out, dpi=120)
        print(f"plot saved to {out}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
