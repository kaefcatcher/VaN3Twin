#!/usr/bin/env python3
"""Aggregate a multi-seed simulation campaign into academic, presentation-ready figures.

Consumes the result trees from ``run_sweep.sh campaign_main`` / ``campaign_network`` plus
the ``manifest.csv`` files from ``gen_campaign.py``. Each config is run under several seeds
(manifest ``seed`` column; ``config_id`` is the seed-independent key), so the figures show
**box plots / error bars** over seeds rather than single points.

  figures/
    fig1_harm_by_scenario.*        box plots: HARM per scenario, no-algo vs algo (headline)
    fig2_harm_reduction_pct.*      box plots: per-seed % HARM reduction the algorithm achieves
    fig3_network_metrics_by_scenario.*  PRR / DENM-PRR / CBR per scenario, mean±std, algo vs no-algo
    fig4_harm_vs_cbr.*             HARM & DENM-PRR vs measured CBR (highway), mean±std, algo vs no-algo
    fig5_sched_pkeep.*             PRR / CBR / HARM vs pKeep, dynamic vs SPS, ALGO vs NO-ALGO
    fig6_denm_copies.*             DENM-PRR & HARM vs DENM copies, ALGO vs NO-ALGO
    fig7_harm_timeseries.*         HARM(t), all seeds overlaid, algo vs no-algo per scenario
    pairwise_harm_heatmap_*.png    per-pair HARM heatmaps for selected runs
    campaign_summary.csv           every run x every metric (one row per seed)
    campaign_summary_agg.csv       per-config mean/std/n for every metric (for tables)

Usage:  python3 scripts/campaign_analyze.py [--results-dir results] [--out figures]
"""

from __future__ import annotations

import argparse
import csv
import glob
import os
import re
import statistics
import sys

from analyze import (  # scripts/ is sys.path[0] when run as a script
    load_harm_log,
    load_msglog_dir,
    make_pairwise_harm_heatmap,
    trapezoid,
)

SCENARIO_ORDER = ["basic", "highway_low", "highway_mid", "highway_high"]
SCENARIO_LABEL = {"basic": "basic", "highway_low": "hw-low", "highway_mid": "hw-mid",
                  "highway_high": "hw-high"}
ALGO_COLOR = {0: "#9e9e9e", 1: "#1f77b4"}   # no-algo grey, algo blue
ALGO_NAME = {0: "no-algo", 1: "algo"}
METRICS = ["prr", "denm_prr", "cam_prr", "cbr", "latency_ms",
           "harm_int", "harm_peak", "denm_tx", "msg_total", "msg_decoded"]


# ----------------------------- loading -----------------------------

def parse_runlog(path: str) -> dict:
    out = {"prr": float("nan"), "cbr": float("nan"), "latency_ms": float("nan")}
    if not os.path.isfile(path):
        return out
    with open(path, errors="ignore") as f:
        text = f.read()
    for key, pat in {
        "prr": r"Average PRR:\s*([-\d.eE+]+)",
        "cbr": r"Average CBR:\s*([-\d.eE+]+)",
        "latency_ms": r"Average latency \(ms\):\s*([-\d.eE+]+)",
    }.items():
        m = re.search(pat, text)
        if m:
            try:
                out[key] = float(m.group(1))
            except ValueError:
                pass
    return out


def load_perveh_msgtype(run_dir: str) -> dict:
    out = {"cam_prr": float("nan"), "denm_prr": float("nan"), "denm_tx": 0}
    hits = glob.glob(os.path.join(run_dir, "*_prr_per_vehicle_messagetype.csv"))
    if not hits:
        return out
    cam, denm, denm_tx = [], [], 0
    with open(hits[0]) as f:
        for row in csv.DictReader(f):
            try:
                prr = float(row.get("avg_prr", "nan"))
                ntx = int(row.get("n_tx", "0"))
            except ValueError:
                continue
            if row.get("message_type") == "CAM":
                cam.append(prr)
            elif row.get("message_type") == "DENM":
                denm.append(prr)
                denm_tx += ntx
    if cam:
        out["cam_prr"] = sum(cam) / len(cam)
    if denm:
        out["denm_prr"] = sum(denm) / len(denm)
    out["denm_tx"] = denm_tx
    return out


def harm_metrics(run_dir: str) -> dict:
    path = os.path.join(run_dir, "harm_log", "harm_log.csv")
    if not os.path.isfile(path):
        return {"harm_int": float("nan"), "harm_peak": float("nan")}
    ts, harm, _ = load_harm_log(path)
    if not ts:
        return {"harm_int": float("nan"), "harm_peak": float("nan")}
    return {"harm_int": trapezoid(ts, harm), "harm_peak": max(harm)}


def find_campaign(results_dir: str, group: str) -> str | None:
    hits = sorted(glob.glob(os.path.join(results_dir, f"*_campaign_{group}")))
    return hits[-1] if hits else None


def collect(results_root: str, manifest_path: str) -> list[dict]:
    if not results_root or not os.path.isfile(manifest_path):
        return []
    root_base = os.path.basename(results_root.rstrip("/"))
    with open(manifest_path) as f:
        manifest = list(csv.DictReader(f))
    rows = []
    for m in manifest:
        run_dir = os.path.join(results_root, f"{root_base}_{m['label']}")
        if not os.path.isdir(run_dir):
            print(f"  missing run dir for {m['label']}", file=sys.stderr)
            continue
        rec = dict(m)
        rec["algo"] = int(m["algo"])
        rec["pkeep"] = float(m["pkeep"])
        rec["denm_copies"] = int(m["denm_copies"])
        rec["seed"] = int(m["seed"])
        rec.update(parse_runlog(os.path.join(run_dir, "run_log", "run.log")))
        rec.update(harm_metrics(run_dir))
        rec.update(load_perveh_msgtype(run_dir))
        rec["msg_total"], rec["msg_decoded"] = load_msglog_dir(
            os.path.join(run_dir, "cam_msg_logs"))
        rec["run_dir"] = run_dir
        rows.append(rec)
    return rows


# ----------------------------- aggregation -----------------------------

def _clean(xs):
    return [x for x in xs if isinstance(x, (int, float)) and x == x]


def aggregate(rows: list[dict]) -> dict:
    """config_id -> {fields, runs, vals{metric:[per-seed]}}."""
    groups: dict[str, dict] = {}
    for r in rows:
        g = groups.setdefault(r["config_id"], {
            "config_id": r["config_id"], "scenario": r["scenario"],
            "cbr_level": r["cbr_level"], "algo": r["algo"], "scheduling": r["scheduling"],
            "pkeep": r["pkeep"], "denm_copies": r["denm_copies"], "axis": r["axis"],
            "group": r["group"], "runs": [], "vals": {m: [] for m in METRICS},
        })
        g["runs"].append(r)
        for m in METRICS:
            g["vals"][m].append(r.get(m, float("nan")))
    return groups


def gvals(groups, metric, **filt):
    """Cleaned per-seed values of the (single) group matching filt."""
    for g in groups.values():
        if all(g.get(k) == v for k, v in filt.items()):
            return _clean(g["vals"][metric])
    return []


def gmean(groups, metric, **filt):
    v = gvals(groups, metric, **filt)
    return statistics.mean(v) if v else float("nan")


def gstd(groups, metric, **filt):
    v = gvals(groups, metric, **filt)
    return statistics.stdev(v) if len(v) > 1 else 0.0


# ----------------------------- figures -----------------------------

def _save(fig, out, name):
    os.makedirs(out, exist_ok=True)
    for ext in ("png", "pdf"):
        fig.savefig(os.path.join(out, f"{name}.{ext}"), dpi=150, bbox_inches="tight")
    print(f"  figure: {name}")


def _box(ax, data, pos, color, width=0.3):
    if not data:
        return
    bp = ax.boxplot([data], positions=[pos], widths=width, patch_artist=True,
                    showfliers=False, medianprops=dict(color="black"))
    for b in bp["boxes"]:
        b.set_facecolor(color)
        b.set_alpha(0.85)


def _algo_legend(ax):
    import matplotlib.patches as mpatches
    ax.legend(handles=[mpatches.Patch(color=ALGO_COLOR[a], label=ALGO_NAME[a])
                       for a in (0, 1)])


def fig1_harm_by_scenario(groups, out, plt):
    scen = [s for s in SCENARIO_ORDER if any(g["scenario"] == s for g in groups.values())]
    fig, ax = plt.subplots(figsize=(10, 5.5))
    for i, s in enumerate(scen):
        for algo, off in ((0, -0.18), (1, 0.18)):
            _box(ax, gvals(groups, "harm_int", scenario=s, algo=algo),
                 i + off, ALGO_COLOR[algo])
    ax.set_xticks(range(len(scen)))
    ax.set_xticklabels([SCENARIO_LABEL[s] for s in scen])
    ax.set_ylabel("∫ Σ pairwise HARM dt")
    ax.set_title("Cooperative-braking algorithm: integrated HARM by scenario "
                 "(box = spread over seeds)")
    ax.grid(True, axis="y", alpha=0.3)
    _algo_legend(ax)
    _save(fig, out, "fig1_harm_by_scenario")
    plt.close(fig)


def fig2_harm_reduction(groups, out, plt):
    scen = [s for s in SCENARIO_ORDER if any(g["scenario"] == s for g in groups.values())]
    fig, ax = plt.subplots(figsize=(10, 5.5))
    data, labels = [], []
    for s in scen:
        # pair noalgo/algo by seed
        no = {r["seed"]: r["harm_int"] for g in groups.values()
              if g["scenario"] == s and g["algo"] == 0 for r in g["runs"]}
        al = {r["seed"]: r["harm_int"] for g in groups.values()
              if g["scenario"] == s and g["algo"] == 1 for r in g["runs"]}
        red = [(no[k] - al[k]) / no[k] * 100.0 for k in sorted(set(no) & set(al))
               if no.get(k) and no[k] == no[k] and al.get(k) == al.get(k)]
        data.append(_clean(red))
        labels.append(SCENARIO_LABEL[s])
    bp = ax.boxplot([d if d else [0] for d in data], patch_artist=True,
                    showfliers=False, medianprops=dict(color="black"))
    for b in bp["boxes"]:
        b.set_facecolor("#2ca02c")
        b.set_alpha(0.8)
    ax.set_xticks(range(1, len(labels) + 1))
    ax.set_xticklabels(labels)
    ax.axhline(0, color="k", lw=0.8)
    ax.set_ylabel("HARM reduction vs no-algo [%]")
    ax.set_title("Per-seed HARM reduction achieved by the algorithm")
    ax.grid(True, axis="y", alpha=0.3)
    _save(fig, out, "fig2_harm_reduction_pct")
    plt.close(fig)


def fig3_network_metrics(groups, out, plt):
    import numpy as np
    scen = [s for s in SCENARIO_ORDER if any(g["scenario"] == s for g in groups.values())]
    x = np.arange(len(scen))
    w = 0.38
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    for ax, (key, title) in zip(axes, [("prr", "overall PRR"),
                                        ("denm_prr", "DENM PRR"), ("cbr", "avg CBR")]):
        for algo, off in ((0, -w / 2), (1, w / 2)):
            means = [gmean(groups, key, scenario=s, algo=algo) for s in scen]
            errs = [gstd(groups, key, scenario=s, algo=algo) for s in scen]
            ax.bar(x + off, means, w, yerr=errs, capsize=3,
                   label=ALGO_NAME[algo], color=ALGO_COLOR[algo])
        ax.set_xticks(x)
        ax.set_xticklabels([SCENARIO_LABEL[s] for s in scen], rotation=30, ha="right")
        ax.set_title(title)
        ax.grid(True, axis="y", alpha=0.3)
    axes[0].set_ylabel("mean ± std over seeds")
    axes[0].legend()
    fig.suptitle("Network metrics by scenario (algo vs no-algo)")
    _save(fig, out, "fig3_network_metrics_by_scenario")
    plt.close(fig)


def fig4_harm_vs_cbr(groups, out, plt):
    levels = ["highway_low", "highway_mid", "highway_high"]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.8))
    for algo in (0, 1):
        pts = []
        for s in levels:
            cbr = gmean(groups, "cbr", scenario=s, algo=algo)
            if cbr == cbr:
                pts.append((cbr * 100,
                            gmean(groups, "harm_int", scenario=s, algo=algo),
                            gstd(groups, "harm_int", scenario=s, algo=algo),
                            gmean(groups, "denm_prr", scenario=s, algo=algo),
                            gstd(groups, "denm_prr", scenario=s, algo=algo)))
        pts.sort()
        if not pts:
            continue
        xs = [p[0] for p in pts]
        ax1.errorbar(xs, [p[1] for p in pts], yerr=[p[2] for p in pts], fmt="o-",
                     capsize=3, label=ALGO_NAME[algo], color=ALGO_COLOR[algo])
        ax2.errorbar(xs, [p[3] for p in pts], yerr=[p[4] for p in pts], fmt="o-",
                     capsize=3, label=ALGO_NAME[algo], color=ALGO_COLOR[algo])
    for ax, ylab, title in ((ax1, "∫ Σ HARM dt", "HARM vs channel load"),
                            (ax2, "DENM PRR", "DENM reliability vs channel load")):
        ax.set_xlabel("measured CBR [%]")
        ax.set_ylabel(ylab)
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.legend()
    fig.suptitle("Highway low/mid/high CBR sweep (mean ± std over seeds)")
    _save(fig, out, "fig4_harm_vs_cbr")
    plt.close(fig)


def fig5_sched_pkeep(groups, out, plt):
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    for ax, (key, title) in zip(axes, [("prr", "overall PRR"),
                                       ("cbr", "avg CBR"), ("harm_int", "∫ Σ HARM dt")]):
        for algo in (0, 1):
            sps = sorted(
                [g for g in groups.values()
                 if g["axis"] == "sched_pkeep" and g["scheduling"] == "sps" and g["algo"] == algo],
                key=lambda g: g["pkeep"])
            if sps:
                xs = [g["pkeep"] for g in sps]
                ys = [statistics.mean(_clean(g["vals"][key])) if _clean(g["vals"][key])
                      else float("nan") for g in sps]
                es = [statistics.stdev(_clean(g["vals"][key])) if len(_clean(g["vals"][key])) > 1
                      else 0.0 for g in sps]
                ax.errorbar(xs, ys, yerr=es, fmt="o-", capsize=3, color=ALGO_COLOR[algo],
                            label=f"SPS, {ALGO_NAME[algo]}")
            dyn = gmean(groups, key, axis="sched_pkeep", scheduling="dynamic", algo=algo)
            if dyn == dyn:
                ax.axhline(dyn, ls="--", color=ALGO_COLOR[algo], alpha=0.7,
                           label=f"dynamic, {ALGO_NAME[algo]}")
        ax.set_xlabel("pKeep")
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
    axes[0].set_ylabel("mean ± std over seeds")
    axes[2].legend(fontsize=8)
    fig.suptitle("Scheduling study (highway-mid): dynamic vs SPS over pKeep, algo vs no-algo")
    _save(fig, out, "fig5_sched_pkeep")
    plt.close(fig)


def fig6_denm_copies(groups, out, plt):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.8))
    for algo in (0, 1):
        cp = sorted([g for g in groups.values()
                     if g["axis"] == "denm_copies" and g["algo"] == algo],
                    key=lambda g: g["denm_copies"])
        if not cp:
            continue
        xs = [g["denm_copies"] for g in cp]
        for ax, key in ((ax1, "denm_prr"), (ax2, "harm_int")):
            ys = [statistics.mean(_clean(g["vals"][key])) if _clean(g["vals"][key])
                  else float("nan") for g in cp]
            es = [statistics.stdev(_clean(g["vals"][key])) if len(_clean(g["vals"][key])) > 1
                  else 0.0 for g in cp]
            ax.errorbar(xs, ys, yerr=es, fmt="o-", capsize=3,
                        color=ALGO_COLOR[algo], label=ALGO_NAME[algo])
    ax1.set_ylabel("DENM PRR")
    ax1.set_title("DENM reliability vs copies")
    ax2.set_ylabel("∫ Σ HARM dt")
    ax2.set_title("HARM vs DENM copies")
    for ax in (ax1, ax2):
        ax.set_xlabel("DENM copies per trigger")
        ax.set_xticks([1, 2, 3])
        ax.grid(True, alpha=0.3)
        ax.legend()
    fig.suptitle("DENM-copies study (highway-mid): algo vs no-algo, mean ± std over seeds")
    _save(fig, out, "fig6_denm_copies")
    plt.close(fig)


def fig7_harm_timeseries(rows, out, plt):
    scen = [s for s in SCENARIO_ORDER if any(r["scenario"] == s for r in rows)]
    if not scen:
        return
    n = len(scen)
    fig, axes = plt.subplots(1, n, figsize=(3.2 * n, 4), squeeze=False)
    seen = {0: False, 1: False}
    for ax, s in zip(axes[0], scen):
        for r in rows:
            if r["scenario"] != s:
                continue
            path = os.path.join(r["run_dir"], "harm_log", "harm_log.csv")
            if not os.path.isfile(path):
                continue
            ts, harm, _ = load_harm_log(path)
            if ts:
                lab = ALGO_NAME[r["algo"]] if not seen[r["algo"]] else None
                seen[r["algo"]] = True
                ax.plot(ts, harm, color=ALGO_COLOR[r["algo"]], lw=0.8, alpha=0.5, label=lab)
        ax.set_title(SCENARIO_LABEL[s])
        ax.set_xlabel("t [s]")
        ax.grid(True, alpha=0.3)
    axes[0][0].set_ylabel("Σ pairwise HARM")
    axes[0][0].legend(fontsize=8)
    fig.suptitle("HARM over time: all seeds overlaid (algo vs no-algo)")
    _save(fig, out, "fig7_harm_timeseries")
    plt.close(fig)


# ----------------------------- summaries -----------------------------

def write_summaries(rows, groups, out):
    os.makedirs(out, exist_ok=True)
    cols = ["group", "axis", "config_id", "scenario", "cbr_level", "algo", "scheduling",
            "pkeep", "denm_copies", "seed"] + METRICS
    with open(os.path.join(out, "campaign_summary.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for r in sorted(rows, key=lambda r: (r["group"], r["config_id"], r["seed"])):
            w.writerow([r.get(c, "") for c in cols])
    # aggregated (per config_id)
    acols = (["group", "axis", "config_id", "scenario", "cbr_level", "algo",
              "scheduling", "pkeep", "denm_copies", "n_seeds"]
             + [f"{m}_{stat}" for m in METRICS for stat in ("mean", "std")])
    with open(os.path.join(out, "campaign_summary_agg.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(acols)
        for g in sorted(groups.values(), key=lambda g: (g["group"], g["config_id"])):
            base = [g["group"], g["axis"], g["config_id"], g["scenario"], g["cbr_level"],
                    g["algo"], g["scheduling"], g["pkeep"], g["denm_copies"],
                    len(g["runs"])]
            stats = []
            for m in METRICS:
                v = _clean(g["vals"][m])
                stats.append(round(statistics.mean(v), 6) if v else "")
                stats.append(round(statistics.stdev(v), 6) if len(v) > 1 else "")
            w.writerow(base + stats)
    print("  summary: campaign_summary.csv, campaign_summary_agg.csv")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--results-dir", default="results")
    ap.add_argument("--configs-dir", default="sweep_configs")
    ap.add_argument("--out", default="figures")
    ap.add_argument("--no-heatmaps", action="store_true")
    args = ap.parse_args()

    main_rows = collect(find_campaign(args.results_dir, "main"),
                        os.path.join(args.configs_dir, "campaign_main", "manifest.csv"))
    net_rows = collect(find_campaign(args.results_dir, "network"),
                       os.path.join(args.configs_dir, "campaign_network", "manifest.csv"))
    all_rows = main_rows + net_rows
    if not all_rows:
        print("no campaign runs found; run ./run_campaign.sh first", file=sys.stderr)
        return 1
    main_groups = aggregate(main_rows)
    net_groups = aggregate(net_rows)
    print(f"collected {len(main_rows)} main + {len(net_rows)} network runs "
          f"({len(main_groups)} + {len(net_groups)} configs)")

    write_summaries(all_rows, {**main_groups, **net_groups}, args.out)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available; wrote summary CSVs only", file=sys.stderr)
        return 0

    if main_groups:
        fig1_harm_by_scenario(main_groups, args.out, plt)
        fig2_harm_reduction(main_groups, args.out, plt)
        fig3_network_metrics(main_groups, args.out, plt)
        fig4_harm_vs_cbr(main_groups, args.out, plt)
        fig7_harm_timeseries(main_rows, args.out, plt)
    if net_groups:
        fig5_sched_pkeep(net_groups, args.out, plt)
        fig6_denm_copies(net_groups, args.out, plt)

    if not args.no_heatmaps:
        for s in ("highway_mid", "highway_high"):
            for algo in (0, 1):
                hit = [g for g in main_groups.values()
                       if g["scenario"] == s and g["algo"] == algo]
                if hit and hit[0]["runs"]:
                    make_pairwise_harm_heatmap(hit[0]["runs"][0]["run_dir"], args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
