#!/usr/bin/env python3
"""Analyse one or more sweep result directories.

A result directory follows the layout produced by run_sweep.sh:

    <results_dir>/
        <date>_<scenario>_<variant>/
            cam_msg_logs/   run-veh*-CAM.csv  -MSGLOG.csv  -COOP.csv
            harm_log/       harm_log.csv
            run_log/        run.log

Usage:
    analyze.py <results_dir> [--plots-dir DIR] [--plot]
"""

from __future__ import annotations

import argparse
import csv
import os
import re
import sys
from collections import defaultdict
from dataclasses import dataclass
from glob import glob
from typing import Iterable


@dataclass(frozen=True)
class RunSummary:
    label: str
    variant_key: float | None
    harm_int: float
    harm_peak: float
    energy_int: float
    energy_peak: float
    msg_total: int
    msg_accepted: int
    msg_prr: float
    n_ticks: int
    harm_series: list[tuple[float, float]]
    energy_series: list[tuple[float, float]]


def load_harm_log(path: str) -> tuple[list[float], list[float], list[float]]:
    harm_sum: dict[float, float] = defaultdict(float)
    energy_peak: dict[float, float] = defaultdict(float)
    with open(path, "r") as f:
        header = next(csv.reader([f.readline()]))
        i_harm = header.index("harm") if "harm" in header else -1
        i_energy = header.index("energy") if "energy" in header else -1
        for row in csv.reader(f):
            if not row:
                continue
            try:
                t = float(row[0])
            except (ValueError, IndexError):
                continue
            if 0 <= i_harm < len(row):
                try:
                    harm_sum[t] += float(row[i_harm])
                except ValueError:
                    pass
            if 0 <= i_energy < len(row):
                try:
                    e = float(row[i_energy])
                    if e > energy_peak[t]:
                        energy_peak[t] = e
                except ValueError:
                    pass
    if not harm_sum:
        return [], [], []
    ts = sorted(harm_sum)
    return ts, [harm_sum[t] for t in ts], [energy_peak[t] for t in ts]


def load_msglog_dir(path: str) -> tuple[int, int]:
    total = 0
    accepted = 0
    for fname in glob(os.path.join(path, "*MSGLOG.csv")):
        with open(fname, "r") as f:
            header_line = f.readline()
            if not header_line:
                continue
            header = header_line.strip().split(",")
            try:
                i_decoded = header.index("decoded")
            except ValueError:
                continue
            for row in csv.reader(f):
                if not row:
                    continue
                total += 1
                if i_decoded < len(row) and row[i_decoded] == "Successfully":
                    accepted += 1
    return total, accepted


def trapezoid(xs: list[float], ys: list[float]) -> float:
    if len(xs) < 2:
        return 0.0
    return sum(
        0.5 * (ys[i - 1] + ys[i]) * (xs[i] - xs[i - 1])
        for i in range(1, len(xs))
    )


def label_from_variant_dir(dirname: str) -> str:
    name = os.path.basename(dirname.rstrip("/"))
    parts = name.split("_")
    return "_".join(parts[2:]) if len(parts) > 2 else name


def variant_sort_key(label: str) -> tuple[int, float, str]:
    if label == "baseline":
        return (0, 0.0, label)
    m = re.match(r"sigma_([0-9eE+\-.]+)$", label)
    if m:
        try:
            return (1, float(m.group(1)), label)
        except ValueError:
            return (2, 0.0, label)
    if label == "sigma_computed":
        return (3, 0.0, label)
    m = re.match(r"mcs_([0-9]+)$", label)
    if m:
        return (4, float(m.group(1)), label)
    m = re.match(r"txpower_([0-9.]+)$", label)
    if m:
        return (5, float(m.group(1)), label)
    if label == "dynamic":
        return (6, -1.0, label)
    m = re.match(r"sps_pkeep_([0-9.]+)$", label)
    if m:
        return (6, float(m.group(1)), label)
    return (7, 0.0, label)


def natural_key(vehicle_id: str) -> tuple[int, int, str]:
    """Sort 'veh2' before 'veh10' by the trailing integer."""
    m = re.search(r"(\d+)", vehicle_id)
    return (0, int(m.group(1)), vehicle_id) if m else (1, 0, vehicle_id)


def discover_runs(root: str) -> list[str]:
    if os.path.isdir(os.path.join(root, "harm_log")):
        return [root]
    return sorted(
        d for d in glob(os.path.join(root, "*"))
        if os.path.isdir(os.path.join(d, "harm_log"))
    )


def summarise(run_dir: str) -> RunSummary | None:
    label = label_from_variant_dir(run_dir)
    harm_path = os.path.join(run_dir, "harm_log", "harm_log.csv")
    if not os.path.isfile(harm_path):
        return None
    ts, harm, energy = load_harm_log(harm_path)
    if not ts:
        return None
    msg_total, msg_accepted = load_msglog_dir(os.path.join(run_dir, "cam_msg_logs"))
    prr = (msg_accepted / msg_total) if msg_total else 0.0
    key = None
    m = re.match(r"sigma_([0-9eE+\-.]+)$", label)
    if m:
        try:
            key = float(m.group(1))
        except ValueError:
            pass
    return RunSummary(
        label=label,
        variant_key=key,
        harm_int=trapezoid(ts, harm),
        harm_peak=max(harm),
        energy_int=trapezoid(ts, energy),
        energy_peak=max(energy),
        msg_total=msg_total,
        msg_accepted=msg_accepted,
        msg_prr=prr,
        n_ticks=len(ts),
        harm_series=list(zip(ts, harm)),
        energy_series=list(zip(ts, energy)),
    )


def print_table(rows: list[RunSummary]) -> None:
    header = (
        f"{'label':<18} {'harm_int':>10} {'harm_pk':>9} "
        f"{'energy_int':>12} {'energy_pk':>11} {'PRR':>6} {'msgs':>7}"
    )
    print(header)
    print("-" * len(header))
    for r in rows:
        print(
            f"{r.label:<18} {r.harm_int:>10.3f} {r.harm_peak:>9.3f} "
            f"{r.energy_int:>12.1f} {r.energy_peak:>11.1f} "
            f"{r.msg_prr:>6.3f} {r.msg_total:>7d}"
        )


def print_deltas(rows: list[RunSummary]) -> None:
    baseline = next((r for r in rows if r.label == "baseline"), None)
    if not baseline:
        return
    others = [r for r in rows if r.label != "baseline"]
    if not others:
        return
    print()
    print("Algorithm vs baseline:")
    for r in others:
        def delta(value: float, base: float) -> str:
            if not base:
                return ""
            d = value - base
            pct = d / base * 100.0
            sign = "↓" if d < 0 else "↑"
            return f"{sign}{abs(pct):.0f}%"
        print(
            f"  {r.label:<18}  "
            f"harm_int {delta(r.harm_int, baseline.harm_int):>7}   "
            f"energy_pk {delta(r.energy_peak, baseline.energy_peak):>7}   "
            f"PRR {delta(r.msg_prr, baseline.msg_prr):>7}"
        )


def load_pairwise_harm(
    path: str,
) -> tuple[dict[tuple[str, str], float], dict[tuple[str, str], float]]:
    """Aggregate the pairwise HARM log into per-pair time-integral and peak.

    The HarmLogger writes one row per in-range vehicle pair per tick
    (time,car1ID,car2ID,harm,energy). For each unordered pair we integrate the
    HARM over time (trapezoid) and track its peak, giving two compact per-pair
    scalars that are easy to render as a matrix / heatmap.
    """
    series: dict[tuple[str, str], list[tuple[float, float]]] = defaultdict(list)
    with open(path, "r") as f:
        header = next(csv.reader([f.readline()]))
        if "harm" not in header:
            return {}, {}
        i_harm = header.index("harm")
        for row in csv.reader(f):
            if len(row) <= i_harm:
                continue
            try:
                t = float(row[0])
                h = float(row[i_harm])
            except (ValueError, IndexError):
                continue
            a, b = row[1], row[2]
            key = (a, b) if a <= b else (b, a)
            series[key].append((t, h))

    pair_integral: dict[tuple[str, str], float] = {}
    pair_peak: dict[tuple[str, str], float] = {}
    for key, pts in series.items():
        pts.sort()
        ts = [p[0] for p in pts]
        hs = [p[1] for p in pts]
        pair_integral[key] = trapezoid(ts, hs)
        pair_peak[key] = max(hs) if hs else 0.0
    return pair_integral, pair_peak


def make_pairwise_harm_heatmap(
    run_dir: str, plots_dir: str, top_k: int = 30, metric: str = "integral"
) -> None:
    """Render a pairwise HARM matrix (heatmap + CSV) for a single run.

    Rows/columns are the top_k vehicles by total HARM involvement; cell (i, j)
    is the chosen per-pair aggregate (time-integrated HARM by default). This
    surfaces *which* vehicle pairs dominate the risk, which the time-series plot
    alone cannot show.
    """
    harm_path = os.path.join(run_dir, "harm_log", "harm_log.csv")
    if not os.path.isfile(harm_path):
        return
    pair_integral, pair_peak = load_pairwise_harm(harm_path)
    pairs = pair_integral if metric == "integral" else pair_peak
    if not pairs:
        return

    totals: dict[str, float] = defaultdict(float)
    for (a, b), v in pairs.items():
        totals[a] += v
        totals[b] += v
    order = sorted(totals, key=lambda k: totals[k], reverse=True)[:top_k]
    order = sorted(order, key=natural_key)
    idx = {v: i for i, v in enumerate(order)}
    n = len(order)
    matrix = [[0.0] * n for _ in range(n)]
    for (a, b), v in pairs.items():
        if a in idx and b in idx:
            matrix[idx[a]][idx[b]] = v
            matrix[idx[b]][idx[a]] = v

    os.makedirs(plots_dir, exist_ok=True)
    label = label_from_variant_dir(run_dir)

    csv_out = os.path.join(plots_dir, f"pairwise_harm_matrix_{label}.csv")
    with open(csv_out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["vehicle", *order])
        for v in order:
            w.writerow([v, *(f"{matrix[idx[v]][idx[u]]:.6g}" for u in order)])
    print(f"pairwise HARM matrix saved: {csv_out}")

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available; skipping HARM heatmap", file=sys.stderr)
        return
    fig, ax = plt.subplots(figsize=(max(6, n * 0.35), max(5, n * 0.32)))
    im = ax.imshow(matrix, cmap="hot_r", aspect="auto")
    ax.set_xticks(range(n))
    ax.set_yticks(range(n))
    ax.set_xticklabels(order, rotation=90, fontsize=6)
    ax.set_yticklabels(order, fontsize=6)
    metric_name = "∫ HARM dt" if metric == "integral" else "peak HARM"
    ax.set_title(f"Pairwise {metric_name} — {label} (top {n} vehicles)")
    fig.colorbar(im, ax=ax, label=metric_name)
    fig.tight_layout()
    out = os.path.join(plots_dir, f"pairwise_harm_heatmap_{label}.png")
    fig.savefig(out, dpi=120)
    plt.close(fig)
    print(f"pairwise HARM heatmap saved: {out}")


def make_plots(rows: list[RunSummary], plots_dir: str) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available; skipping --plot", file=sys.stderr)
        return
    os.makedirs(plots_dir, exist_ok=True)

    fig, ax = plt.subplots(figsize=(9, 5))
    for r in rows:
        if not r.harm_series:
            continue
        ts, ys = zip(*r.harm_series)
        ax.plot(ts, ys, label=r.label, linewidth=1.0)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("Σ pairwise HARM")
    ax.set_title("Σ pairwise HARM vs time")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    p1 = os.path.join(plots_dir, "harm_vs_time.png")
    fig.savefig(p1, dpi=120)
    plt.close(fig)
    print(f"plot saved: {p1}")

    fig, ax = plt.subplots(figsize=(9, 5))
    for r in rows:
        if not r.energy_series:
            continue
        ts, ys = zip(*r.energy_series)
        ax.plot(ts, ys, label=r.label, linewidth=1.0)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("peak collision energy ½μΔv² [J]")
    ax.set_title("Worst-pair collision energy vs time")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    p2 = os.path.join(plots_dir, "energy_vs_time.png")
    fig.savefig(p2, dpi=120)
    plt.close(fig)
    print(f"plot saved: {p2}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("results_dir")
    parser.add_argument("--plots-dir", default=None)
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()

    runs = discover_runs(args.results_dir)
    if not runs:
        print(f"no result folders under {args.results_dir}", file=sys.stderr)
        return 1

    summaries: list[RunSummary] = []
    for run in runs:
        s = summarise(run)
        if s is None:
            print(f"skipped (no harm_log): {run}", file=sys.stderr)
            continue
        summaries.append(s)
    if not summaries:
        return 1
    summaries.sort(key=lambda r: variant_sort_key(r.label))

    print_table(summaries)
    print_deltas(summaries)

    if args.plot:
        plots_dir = args.plots_dir or os.path.join(args.results_dir, "plots")
        make_plots(summaries, plots_dir)
        for run in runs:
            make_pairwise_harm_heatmap(run, plots_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
