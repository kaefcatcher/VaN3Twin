#!/usr/bin/env python3
"""Generate the simulation-campaign config folders + manifests.

Produces ready-to-run ``config.json`` variants (consumed by ``run_sweep.sh``), plus a
``manifest.csv`` in each folder mapping every variant label to its structured fields so
``campaign_analyze.py`` never has to parse filenames.

  sweep_configs/campaign_main/      Group A: the 10 headline scenario/algo combos
                                    (basic / highway low,mid,high / moscow) x {no-algo, algo}
  sweep_configs/campaign_network/   Group B: highway-mid, x {no-algo, algo}, OFAT sweeps of
                                    scheduling/pKeep and DENM-copy count

Every combo is emitted once per **seed** in ``SEEDS`` (each config sets a distinct ``seed``
that drives both the SUMO mobility RNG and the ns-3 radio RNG), so the analyzer can draw
box plots / error bars across seeds. ``config_id`` is the seed-independent key the analyzer
groups by; ``label`` = ``<config_id>_s<seed>``.

"algo" = cooperative_detection=True (sigma_mode=computed); "no-algo" = cooperative_detection=False.

Brake policy: the example's force brake is POSITION based (``force_brake_time`` is ignored),
so each map gets a reachable ``force_brake_position`` and brakes veh0..veh9 to a stop — the
cooperative algorithm only acts on a hard-brake DENM, so every scenario needs a firing hazard.
"""

from __future__ import annotations

import csv
import json
import os

from gen_highway_density import (
    DENSITY_LEVELS,  # noqa: F401  (documents the highway levels; values used via SCENARIOS)
    HIGHWAY_BRAKE_POS,
    HIGHWAY_SIM_TIME,
)

CONFIG_ROOT = "sweep_configs"
EX = "src/automotive/examples"

# Seeds per config. THIS IS THE RUNTIME MULTIPLIER: total runs = (#combos) * len(SEEDS).
# 5 gives reasonable box plots; drop to [1, 2, 3] for a faster first pass.
SEEDS = [1, 2, 3, 4, 5]

# Network baseline shared by the Group-A scenario study.
BASE = {
    "realtime": False,
    "sumo_gui": False,
    "sumo_updates": 0.01,
    "vehicle_vis": False,
    "penetrationRate": 1.0,
    "simTime": 22.5,
    "tx_power": 23.0,
    "slSubchannelSize": 10,
    "slMaxNumPerReserve": 3,
    "t1": 2,
    "t2": 81,
    "mcs": 14,
    "reservationPeriod": 20,
    "slProbResourceKeep": 0.0,
    "reselection_counter": 0,          # 0 = SPS (standard random Cresel); 1 = dynamic
    "cbr_enabled": True,
    "cbr_window_ms": 100.0,
    "cbr_alpha": 0.5,
    "m_baseline_prr": 150.0,
    "m_metric_sup": True,
    "send_denm": True,
    "denm_copies": 1,
    "denm_copy_spacing_ms": 20.0,
    "cooperative_detection": False,
    "sigma_mode": "computed",
    "fixed_sigma": 0.5,
    "include_ethical_alacarte": False,
    "chain_brake_fraction": 1.0,
    "force_brake_time": -1,
    "force_brake_position": 50.0,
    "force_brake_vehicles": "veh0,veh1,veh2,veh3,veh4,veh5,veh6,veh7,veh8,veh9",
    "force_brake_duration": 1.0,
    "force_brake_target_speed": 0.0,
    "harm_log_file": "harm_log.csv",
    "harm_log_period_s": 0.1,
    "harm_log_radius_m": 150.0,
    "csv_log": "run",
    "speed_drop_threshold": 3.0,
    "stationary_speed": 1.0,
    "was_moving_speed": 5.0,
    "csv_name_cumulative": "results",
    "seed": SEEDS[0],
}

# scenario family -> SUMO map fields + scenario-specific brake/sim overrides
SCENARIOS = {
    "basic": {
        "sumo_folder": f"{EX}/sumo_files_v2v_cooperative/",
        "mob_trace": "cooperative.rou.xml",
        "sumo_config": f"{EX}/sumo_files_v2v_cooperative/cooperative.sumo.cfg",
        "simTime": 22.5,
        "force_brake_position": 50.0,
        "cbr_level": "na",
    },
    "highway_low": {
        "sumo_folder": f"{EX}/highway/",
        "mob_trace": "highway_low.rou.xml",
        "sumo_config": f"{EX}/highway/highway_low.sumocfg",
        "simTime": HIGHWAY_SIM_TIME,
        "force_brake_position": HIGHWAY_BRAKE_POS,
        "cbr_level": "low",
    },
    "highway_mid": {
        "sumo_folder": f"{EX}/highway/",
        "mob_trace": "highway_mid.rou.xml",
        "sumo_config": f"{EX}/highway/highway_mid.sumocfg",
        "simTime": HIGHWAY_SIM_TIME,
        "force_brake_position": HIGHWAY_BRAKE_POS,
        "cbr_level": "mid",
    },
    "highway_high": {
        "sumo_folder": f"{EX}/highway/",
        "mob_trace": "highway_high.rou.xml",
        "sumo_config": f"{EX}/highway/highway_high.sumocfg",
        "simTime": HIGHWAY_SIM_TIME,
        "force_brake_position": HIGHWAY_BRAKE_POS,
        "cbr_level": "high",
    },
    "moscow_large": {
        "sumo_folder": f"{EX}/moscow/",
        "mob_trace": "routes1000.rou.xml",   # both algo on/off use the SAME (large) traffic
        "sumo_config": f"{EX}/moscow/moscow.sumocfg",
        "simTime": 22.5,
        "force_brake_position": 50.0,
        "cbr_level": "na",
    },
}

# Group-B network points (applied to highway_mid): label-stub -> (scheduling, pkeep, copies, overrides)
NETWORK_POINTS = [
    ("dynamic",  "dynamic", 0.0, 1, {"reselection_counter": 1, "slProbResourceKeep": 0.0}),
    ("sps_pk0.0", "sps", 0.0, 1, {"reselection_counter": 0, "slProbResourceKeep": 0.0}),
    ("sps_pk0.2", "sps", 0.2, 1, {"reselection_counter": 0, "slProbResourceKeep": 0.2}),
    ("sps_pk0.4", "sps", 0.4, 1, {"reselection_counter": 0, "slProbResourceKeep": 0.4}),
    ("sps_pk0.6", "sps", 0.6, 1, {"reselection_counter": 0, "slProbResourceKeep": 0.6}),
    ("sps_pk0.8", "sps", 0.8, 1, {"reselection_counter": 0, "slProbResourceKeep": 0.8}),
    ("copies1", "sps", 0.0, 1, {"reselection_counter": 0, "slProbResourceKeep": 0.0, "denm_copies": 1}),
    ("copies2", "sps", 0.0, 2, {"reselection_counter": 0, "slProbResourceKeep": 0.0, "denm_copies": 2}),
    ("copies3", "sps", 0.0, 3, {"reselection_counter": 0, "slProbResourceKeep": 0.0, "denm_copies": 3}),
]
NETWORK_AXIS = {p[0]: ("denm_copies" if p[0].startswith("copies") else "sched_pkeep")
                for p in NETWORK_POINTS}


def _cfg(scenario: str, algo: bool, seed: int, **overrides) -> dict:
    cfg = dict(BASE)
    cfg.update({k: v for k, v in SCENARIOS[scenario].items() if k != "cbr_level"})
    cfg["cooperative_detection"] = bool(algo)
    cfg["seed"] = seed
    cfg.update(overrides)
    return cfg


def _write(folder: str, label: str, cfg: dict) -> None:
    with open(os.path.join(folder, f"{label}.json"), "w") as f:
        json.dump(cfg, f, indent=2)
        f.write("\n")


def _manifest(folder: str, rows: list[dict]) -> None:
    fields = ["label", "config_id", "scenario", "cbr_level", "algo", "scheduling",
              "pkeep", "denm_copies", "seed", "group", "axis"]
    with open(os.path.join(folder, "manifest.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)


def gen_main() -> list[dict]:
    folder = os.path.join(CONFIG_ROOT, "campaign_main")
    os.makedirs(folder, exist_ok=True)
    rows = []
    for scenario in SCENARIOS:
        for algo in (False, True):
            config_id = f"{scenario}_{'algo' if algo else 'noalgo'}"
            for seed in SEEDS:
                label = f"{config_id}_s{seed}"
                _write(folder, label, _cfg(scenario, algo, seed))
                rows.append({
                    "label": label, "config_id": config_id, "scenario": scenario,
                    "cbr_level": SCENARIOS[scenario]["cbr_level"], "algo": int(algo),
                    "scheduling": "sps", "pkeep": 0.0, "denm_copies": 1, "seed": seed,
                    "group": "main", "axis": "scenario",
                })
    _manifest(folder, rows)
    return rows


def gen_network() -> list[dict]:
    """Group B: highway_mid, both algo states, OFAT over scheduling/pKeep and DENM copies."""
    folder = os.path.join(CONFIG_ROOT, "campaign_network")
    os.makedirs(folder, exist_ok=True)
    rows = []
    for stub, sched, pk, copies, ov in NETWORK_POINTS:
        for algo in (False, True):
            config_id = f"hwmid_{stub}_{'algo' if algo else 'noalgo'}"
            for seed in SEEDS:
                label = f"{config_id}_s{seed}"
                _write(folder, label, _cfg("highway_mid", algo, seed, **ov))
                rows.append({
                    "label": label, "config_id": config_id, "scenario": "highway_mid",
                    "cbr_level": "mid", "algo": int(algo), "scheduling": sched,
                    "pkeep": pk, "denm_copies": copies, "seed": seed,
                    "group": "network", "axis": NETWORK_AXIS[stub],
                })
    _manifest(folder, rows)
    return rows


def main() -> int:
    main_rows = gen_main()
    net_rows = gen_network()
    n_main_cfg = len({r["config_id"] for r in main_rows})
    n_net_cfg = len({r["config_id"] for r in net_rows})
    print(f"seeds: {SEEDS}  ({len(SEEDS)} per config)")
    print(f"campaign_main:    {n_main_cfg} configs x {len(SEEDS)} = {len(main_rows)} runs")
    print(f"campaign_network: {n_net_cfg} configs x {len(SEEDS)} = {len(net_rows)} runs")
    print(f"TOTAL RUNS: {len(main_rows) + len(net_rows)}  "
          f"(each ns-3+SUMO run is minutes; moscow/highway-high are the long poles)")
    print("\nRun:  ./run_campaign.sh   (or ./run_sweep.sh campaign_main && ./run_sweep.sh campaign_network)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
