#!/usr/bin/env python3
"""Aggregate and compare baseline + σ-sweep HARM logs.

CSV schema (newer logs, written by HarmLogger after F15):
    time, car1ID, car2ID, harm, gap, ttc, energy

This analyzer keeps only the two metrics that turned out to be useful in
practice — Σ pairwise HARM (paper formula 3) and peak collision energy
(½·μ·Δv²). gap and ttc are still in the CSV in case you want to look
at them by hand, but the analyzer no longer summarises them; they were
noisy and dominated by trivial pair geometry rather than by the
algorithm's safety effect.

Outputs (when --plot is passed):
    <plots-dir>/sigma_sweep_harm.png       Σ harm(t) overlays
    <plots-dir>/sigma_sweep_energy.png     peak energy(t) overlays
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import sys
from collections import defaultdict


def load_harm_log(path: str):
    """Return (ts, harm_sum_per_tick, energy_peak_per_tick)."""
    harm_sum: dict[float, float] = defaultdict(float)
    energy_peak: dict[float, float] = defaultdict(float)
    with open(path, "r") as f:
        header = f.readline().strip().lower().split(",")
        if not header or header[0] != "time":
            raise ValueError(f"{path}: unexpected header {header!r}")
        idx_harm = header.index("harm") if "harm" in header else -1
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
            if 0 <= idx_harm < len(parts):
                try:
                    harm_sum[t] += float(parts[idx_harm])
                except ValueError:
                    pass
            if 0 <= idx_energy < len(parts):
                try:
                    e = float(parts[idx_energy])
                    if e > energy_peak[t]:
                        energy_peak[t] = e
                except ValueError:
                    pass
    if not harm_sum:
        return [], [], []
    ts = sorted(harm_sum)
    return ts, [harm_sum[t] for t in ts], [energy_peak[t] for t in ts]


def trapezoid(xs, ys) -> float:
    if len(xs) < 2:
        return 0.0
    out = 0.0
    for i in range(1, len(xs)):
        out += 0.5 * (ys[i - 1] + ys[i]) * (xs[i] - xs[i - 1])
    return out


def label_for(path: str) -> tuple[str, float | None]:
    stem = os.path.splitext(os.path.basename(path))[0]
    if stem == "harm_log_baseline":
        return "baseline", None
    m = re.match(r"harm_log_sigma_([0-9eE+\-.]+)", stem)
    if m:
        try:
            return f"σ={m.group(1)}", float(m.group(1))
        except ValueError:
            return stem, None
    if stem == "harm_log_sigma_computed":
        return "σ=computed", None
    return stem, None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--glob", default="harm_log_*.csv",
                    help="Glob pattern for HARM log files.")
    ap.add_argument("--plots-dir", default="sweep_results/plots",
                    help="Directory for the figures written by --plot.")
    ap.add_argument("--plot", action="store_true",
                    help="Render Σ harm(t) and peak energy(t) overlays.")
    args = ap.parse_args()

    files = sorted(glob.glob(args.glob))
    if not files:
        print(f"No files matched {args.glob}", file=sys.stderr)
        return 1

    rows = []
    series = {}  # label -> (ts, harm, energy)
    skipped = []
    for path in files:
        label, sigma = label_for(path)
        try:
            ts, harm, energy = load_harm_log(path)
        except Exception as e:
            skipped.append((path, f"parse error: {e}"))
            continue
        if not ts:
            skipped.append((path, "empty"))
            continue
        harm_int = trapezoid(ts, harm)
        harm_peak = max(harm) if harm else 0.0
        energy_int = trapezoid(ts, energy)
        energy_peak = max(energy) if energy else 0.0
        rows.append((sigma, label, harm_int, harm_peak,
                     energy_int, energy_peak, len(ts), path))
        series[label] = (ts, harm, energy)

    if not rows:
        print("All matching files were empty or unreadable.", file=sys.stderr)
        for path, why in skipped:
            print(f"  {path}: {why}", file=sys.stderr)
        return 1
    if skipped:
        print(f"Skipped {len(skipped)} file(s):", file=sys.stderr)
        for path, why in skipped:
            print(f"  {path}: {why}", file=sys.stderr)

    def sort_key(r):
        sigma, label, *_ = r
        if label == "baseline":
            return (0, 0.0, "")
        if sigma is not None:
            return (1, sigma, label)
        return (2, 0.0, label)
    rows.sort(key=sort_key)

    baseline_harm_int = next((r[2] for r in rows if r[1] == "baseline"), None)
    baseline_energy_peak = next((r[5] for r in rows if r[1] == "baseline"), None)

    # Table: harm_int / harm_pk / energy_int / energy_pk
    hdr = (f"{'label':<14} {'harm_int':>10} {'harm_pk':>9} "
           f"{'energy_int':>12} {'energy_pk':>11} {'ticks':>6}")
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        sigma, label, harm_int, harm_pk, energy_int, energy_pk, ntick, _ = r
        print(f"{label:<14} {harm_int:>10.3f} {harm_pk:>9.3f} "
              f"{energy_int:>12.1f} {energy_pk:>11.1f} {ntick:>6d}")
    print()

    # Algorithm vs baseline delta (lower harm/energy = better).
    sigma_rows = [r for r in rows if r[1] != "baseline"]
    if sigma_rows and baseline_harm_int is not None:
        print("Algorithm vs baseline (↓ means smaller = better):")
        for r in sigma_rows:
            sigma, label, harm_int, _, _, energy_pk, *_ = r
            def _delta(value, base):
                if base is None or base == 0:
                    return ""
                d = value - base
                pct = d / base * 100.0
                sign = "↓" if d < 0 else "↑"
                return f"{sign}{abs(pct):.0f}%"
            print(f"  {label:<14}  "
                  f"harm_int {_delta(harm_int, baseline_harm_int):>7}   "
                  f"energy_pk {_delta(energy_pk, baseline_energy_peak):>7}")
        print()

    if args.plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not available; skipping --plot", file=sys.stderr)
            return 0
        os.makedirs(args.plots_dir, exist_ok=True)

        # Σ harm(t) overlay.
        fig, ax = plt.subplots(figsize=(9, 5))
        for r in rows:
            label = r[1]
            ts, harm, _ = series[label]
            ax.plot(ts, harm, label=label, linewidth=1.0)
        ax.set_xlabel("time [s]")
        ax.set_ylabel("Σ pairwise HARM  (paper formula 3)")
        ax.set_title("Σ pairwise HARM vs time")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8)
        fig.tight_layout()
        out1 = os.path.join(args.plots_dir, "sigma_sweep_harm.png")
        fig.savefig(out1, dpi=120)
        plt.close(fig)
        print(f"plot saved: {out1}")

        # Peak energy(t) overlay.
        fig, ax = plt.subplots(figsize=(9, 5))
        for r in rows:
            label = r[1]
            ts, _, energy = series[label]
            ax.plot(ts, energy, label=label, linewidth=1.0)
        ax.set_xlabel("time [s]")
        ax.set_ylabel("peak pair collision energy ½μΔv²  [J]")
        ax.set_title("Worst-pair collision energy vs time")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8)
        fig.tight_layout()
        out2 = os.path.join(args.plots_dir, "sigma_sweep_energy.png")
        fig.savefig(out2, dpi=120)
        plt.close(fig)
        print(f"plot saved: {out2}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
