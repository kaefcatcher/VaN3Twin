#!/usr/bin/env python3
"""Generate highway SUMO route + config files at low / mid / high vehicle density.

The highway net (`src/automotive/examples/highway/highway.net.xml`) is a trivial
5 km straight road with edges ``AB`` and ``BA`` (3 lanes each). A SUMO route file
only needs ``<vehicle id="vehN" depart="t"><route edges="AB"/></vehicle>`` rows, so
these density variants are plain XML and need **no SUMO to generate** (SUMO only
consumes them at run time).

CBR is driven by channel load, i.e. by how many V2X transmitters are within sensing
range. We control that with the vehicle-insertion period: a short period packs more
cars onto the road -> higher CBR. The three levels here target low / mid / high
channel occupancy. The *predicted* CBR printed below uses the closed-form model from
``docs/nr_v2x_cbr_analysis.md``; the **measured** ``Average CBR`` printed by the
simulation (now that CBR output is wired in) is authoritative and should be used to
label/retune the levels — the insertion periods in ``DENSITY_LEVELS`` are the knob.

All vehicles travel the same direction (``AB``, 0 -> 5000 m) so that vehicles
``veh0..veh9`` form the lead platoon that brakes at ``HIGHWAY_BRAKE_POS`` while later
vehicles approach from behind, producing the chain-braking hazard the cooperative
algorithm is meant to mitigate.
"""

from __future__ import annotations

import os

# --- Highway scenario parameters (single source of truth, imported by gen_campaign) ---

HIGHWAY_DIR = "src/automotive/examples/highway"
ROUTE_EDGES = "AB"          # single direction, 0 -> 5000 m
ROAD_LEN_M = 5000.0
FREEFLOW_SPEED_MS = 33.3    # ~120 km/h (from highway.net.xml lane speed)

# Brake the lead platoon at a position reachable within the sim window. At
# 33.3 m/s a car covers ~800 m in ~24 s, leaving aftermath before SIM_TIME.
# (The example's force brake is POSITION based; force_brake_time is ignored.)
HIGHWAY_BRAKE_POS = 800.0
HIGHWAY_SIM_TIME = 40.0      # s — long enough for the platoon to reach the brake + aftermath
INSERT_HORIZON_S = HIGHWAY_SIM_TIME  # keep inserting vehicles for the whole run

# level -> vehicle insertion period [s]. Shorter period => denser => higher CBR.
# Retune these from the measured "Average CBR" after the first run.
DENSITY_LEVELS = {
    "low": 1.5,    # ~27 vehicles over the run
    "mid": 0.5,    # ~80 vehicles
    "high": 0.18,  # ~220 vehicles
}

# CBR model constants (see docs/nr_v2x_cbr_analysis.md)
_N_SCH = 5            # sub-channels
_T_SLOT_S = 0.25e-3   # slot duration (numerology 2)
_CAM_RATE_HZ = 10.0   # representative worst-case CAM cadence
_TX_PER_TB = 2        # 1 tx + 1 blind re-tx (representative; up to 5 configured)
_SUBCH_PER_MSG = 2    # secured CAM footprint


def _predicted_cbr(n_vehicles: int) -> float:
    """Rough analytic CBR if all inserted vehicles are mutually in sensing range."""
    per_veh = _CAM_RATE_HZ * _TX_PER_TB * _SUBCH_PER_MSG * _T_SLOT_S / _N_SCH
    return min(1.0, n_vehicles * per_veh)


def _write_rou(path: str, period: float) -> int:
    depart = 0.0
    rows = []
    i = 0
    while depart < INSERT_HORIZON_S:
        rows.append(
            f'    <vehicle id="veh{i}" depart="{depart:.2f}">\n'
            f'        <route edges="{ROUTE_EDGES}"/>\n'
            f"    </vehicle>"
        )
        i += 1
        depart += period
    with open(path, "w") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write(
            '<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" '
            'xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">\n'
        )
        f.write("\n".join(rows))
        f.write("\n</routes>\n")
    return i


def _write_sumocfg(path: str, rou_name: str) -> None:
    with open(path, "w") as f:
        f.write(
            "<configuration>\n"
            "    <input>\n"
            '        <net-file value="highway.net.xml"/>\n'
            f'        <route-files value="{rou_name}"/>\n'
            "    </input>\n"
            "    <time>\n"
            '        <begin value="0"/>\n'
            f'        <end value="{int(HIGHWAY_SIM_TIME) + 10}"/>\n'
            "    </time>\n"
            "</configuration>\n"
        )


def main() -> int:
    os.makedirs(HIGHWAY_DIR, exist_ok=True)
    print(f"Highway density generator -> {HIGHWAY_DIR}")
    print(f"  road {ROAD_LEN_M:.0f} m, dir '{ROUTE_EDGES}', brake @ {HIGHWAY_BRAKE_POS:.0f} m, "
          f"simTime {HIGHWAY_SIM_TIME:.0f} s\n")
    print(f"  {'level':<6}{'period[s]':>10}{'vehicles':>10}{'pred.CBR':>10}")
    for level, period in DENSITY_LEVELS.items():
        rou = os.path.join(HIGHWAY_DIR, f"highway_{level}.rou.xml")
        cfg = os.path.join(HIGHWAY_DIR, f"highway_{level}.sumocfg")
        n = _write_rou(rou, period)
        _write_sumocfg(cfg, f"highway_{level}.rou.xml")
        print(f"  {level:<6}{period:>10.2f}{n:>10}{_predicted_cbr(n) * 100:>9.0f}%")
    print("\n  NOTE: predicted CBR assumes all vehicles are mutually in range and a")
    print("  representative CAM rate; use the simulation's measured 'Average CBR' to")
    print("  label/retune the levels (edit DENSITY_LEVELS periods).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
