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
    """Parse a HARM log.

    Returns (ts, harm_sum, gap_min, ttc_min, energy_peak) where:
      ts            sorted list of tick times
      harm_sum[t]   Σ pairwise harm at tick t
      gap_min[t]    minimum pair-gap at tick t (or 1e9 if no rows)
      ttc_min[t]    minimum pair-TTC at tick t (or 1e9 if no rows)
      energy_peak[t] maximum pair-collision-energy at tick t

    Backward-compatible: if the CSV has only the legacy 4 columns
    (time,car1ID,car2ID,harm), gap/ttc/energy fall back to sentinels.
    """
    INF = 1e9
    harm_sum: dict[float, float] = defaultdict(float)
    gap_min: dict[float, float] = defaultdict(lambda: INF)
    ttc_min: dict[float, float] = defaultdict(lambda: INF)
    energy_peak: dict[float, float] = defaultdict(float)
    with open(path, "r") as f:
        header = f.readline().strip().lower().split(",")
        if not header or header[0] != "time":
            raise ValueError(f"{path}: unexpected header {header!r}")
        # column indexes (negative if absent)
        idx_harm = header.index("harm") if "harm" in header else -1
        idx_gap = header.index("gap") if "gap" in header else -1
        idx_ttc = header.index("ttc") if "ttc" in header else -1
        idx_energy = header.index("energy") if "energy" in header else -1
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            if len(parts) < 4:
                continue
            try:
                t = float(parts[0])
            except ValueError:
                continue
            if idx_harm >= 0 and idx_harm < len(parts):
                try:
                    harm_sum[t] += float(parts[idx_harm])
                except ValueError:
                    pass
            if idx_gap >= 0 and idx_gap < len(parts):
                try:
                    g = float(parts[idx_gap])
                    if g < gap_min[t]:
                        gap_min[t] = g
                except ValueError:
                    pass
            if idx_ttc >= 0 and idx_ttc < len(parts):
                try:
                    tt = float(parts[idx_ttc])
                    if tt < ttc_min[t]:
                        ttc_min[t] = tt
                except ValueError:
                    pass
            if idx_energy >= 0 and idx_energy < len(parts):
                try:
                    e = float(parts[idx_energy])
                    if e > energy_peak[t]:
                        energy_peak[t] = e
                except ValueError:
                    pass
    if not harm_sum:
        return [], [], [], [], []
    ts = sorted(harm_sum)
    return (ts,
            [harm_sum[t] for t in ts],
            [gap_min[t] for t in ts],
            [ttc_min[t] for t in ts],
            [energy_peak[t] for t in ts])


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

    rows = []   # tuple per run, see fields below
    series = {} # label -> (ts, harm, gap, ttc, energy)
    skipped = []
    for path in files:
        label, sigma = label_for(path)
        try:
            ts, harm, gap, ttc, energy = load_harm_log(path)
        except Exception as e:
            print(f"  {path}: parse error {e}", file=sys.stderr)
            skipped.append((path, f"parse error: {e}"))
            continue
        if not ts:
            skipped.append((path, "empty"))
            continue
        # Reduce per-tick series to per-run scalars.
        harm_int = trapezoid(ts, harm)
        harm_peak = max(harm) if harm else 0.0
        # gap/ttc are "min over pairs at each tick"; take min over time too.
        # If the column was absent the loader fills with 1e9 sentinels.
        gap_min = min(gap) if gap else 1e9
        ttc_min = min(ttc) if ttc else 1e9
        energy_peak = max(energy) if energy else 0.0
        rows.append((sigma, label, harm_int, harm_peak,
                     gap_min, ttc_min, energy_peak, len(ts), path))
        series[label] = (ts, harm, gap, ttc, energy)

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

    baseline_int = next((r[2] for r in rows if r[1] == "baseline"), None)
    baseline_gap = next((r[4] for r in rows if r[1] == "baseline"), None)
    baseline_ttc = next((r[5] for r in rows if r[1] == "baseline"), None)
    baseline_energy = next((r[6] for r in rows if r[1] == "baseline"), None)

    # Print comparison table.
    # harm_int          smaller = less momentum-imbalance over time (paper formula 3)
    # harm_peak         worst single-tick total
    # gap_min           closest the worst pair ever got (m) — smaller = closer to crash
    # ttc_min           shortest time-to-collision (s) — smaller = riskier
    # energy_peak       worst-case plastic-collision energy (J) at any tick
    hdr = (f"{'label':<14} {'harm_int':>10} {'harm_pk':>9} "
           f"{'gap_min':>8} {'ttc_min':>8} {'energy_pk':>10} {'ticks':>6}")
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        sigma, label, harm_int, harm_pk, gap_min, ttc_min, energy_pk, ntick, _ = r
        def _fmt(v, scale):
            return "—" if v is None else f"{v / scale:>.3f}" if False else f"{v:.3f}"
        gap_str = "—" if gap_min >= 1e9 else f"{gap_min:.3f}"
        ttc_str = "—" if ttc_min >= 1e9 else f"{ttc_min:.3f}"
        print(f"{label:<14} {harm_int:>10.3f} {harm_pk:>9.3f} "
              f"{gap_str:>8} {ttc_str:>8} {energy_pk:>10.3f} {ntick:>6d}")
    print()

    # Best-σ summary: rank σ runs by each metric and call out the winner.
    sigma_rows = [r for r in rows if r[1] != "baseline"]
    if sigma_rows and baseline_int is not None:
        print("Algorithm vs baseline (smaller = safer for gap_min/ttc_min,")
        print("                       smaller = better for harm_int/energy_pk):")
        for r in sigma_rows:
            sigma, label, harm_int, harm_pk, gap_min, ttc_min, energy_pk, *_ = r
            def _delta(value, base, lower_is_better):
                if base is None or base == 0:
                    return ""
                if value is None or value >= 1e9 or base >= 1e9:
                    return ""
                d = value - base
                pct = d / base * 100.0
                sign = "↓" if (d < 0) == lower_is_better else "↑"
                return f"{sign}{abs(pct):.0f}%"
            print(f"  {label:<14}  "
                  f"harm_int {_delta(harm_int, baseline_int, True):>7}  "
                  f"gap_min  {_delta(gap_min, baseline_gap, False):>7}  "
                  f"ttc_min  {_delta(ttc_min, baseline_ttc, False):>7}  "
                  f"energy_pk {_delta(energy_pk, baseline_energy, True):>7}")
    print()

    if args.plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not available; skipping --plot", file=sys.stderr)
            return 0
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        for sigma, label, *_ in rows:
            ts, harm, gap, ttc, energy = series[label]
            axes[0, 0].plot(ts, harm,   label=label, linewidth=1.0)
            axes[0, 1].plot(ts, gap,    label=label, linewidth=1.0)
            axes[1, 0].plot(ts, ttc,    label=label, linewidth=1.0)
            axes[1, 1].plot(ts, energy, label=label, linewidth=1.0)
        axes[0, 0].set_title("Σ pairwise HARM (momentum diff)")
        axes[0, 0].set_ylabel("HARM")
        axes[0, 1].set_title("min pair-gap")
        axes[0, 1].set_ylabel("m")
        axes[1, 0].set_title("min pair-TTC (capped)")
        axes[1, 0].set_ylabel("s"); axes[1, 0].set_yscale("log")
        axes[1, 1].set_title("peak collision energy (½μΔv²)")
        axes[1, 1].set_ylabel("J")
        for ax in axes.flat:
            ax.set_xlabel("time [s]")
            ax.grid(True, alpha=0.3)
            ax.legend(loc="best", fontsize=7)
        fig.tight_layout()
        out = "sigma_sweep_harm.png"
        fig.savefig(out, dpi=120)
        print(f"plot saved to {out}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
