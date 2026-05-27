# Moscow visual demo

A large-scale demo of the cooperative DENM / σ-budget algorithm on real
OpenStreetMap road geometry in central Moscow (Tverskaya / Pushkin
Square), with ~80 randomly-routed vehicles.

## One-time setup

```bash
cd src/automotive/examples/sumo_files_v2v_moscow
./setup_moscow.sh
```

Requirements: `sumo`, `netconvert`, `polyconvert`, `randomTrips.py`,
`curl`, and Python 3 in `$PATH`. `setup_moscow.sh` downloads a real
OSM extract via Overpass, converts it to a SUMO net with UTM zone-37
projection (DENMs need a working projection for ASN.1 encoding), runs
`randomTrips.py` with a period chosen to give roughly `MOSCOW_TRIPS`
(default 80) vehicles over 60 s, and writes `moscow.sumo.cfg` with
`collision.action warn` so a single accident doesn't halt the run.

Tunables (env vars):

| variable          | default      | meaning                                 |
|-------------------|--------------|-----------------------------------------|
| `MOSCOW_LAT`      | `55.7647`    | bbox centre latitude                    |
| `MOSCOW_LON`      | `37.6058`    | bbox centre longitude                   |
| `MOSCOW_RADIUS_M` | `600`        | half-side of the OSM bbox (m)           |
| `MOSCOW_TRIPS`    | `80`         | target number of vehicles               |
| `MOSCOW_SEED`     | `42`         | random-trip seed                        |

## Running the demo

Two ways:

### Quick run with the existing scenario binary

```bash
cd ../../../../..   # back to project root
./ns3 build
./ns3 run "v2v-emergencyVehicleAlert-nrv2x \
  --sumo-folder=src/automotive/examples/sumo_files_v2v_moscow/ \
  --mob-trace=moscow.rou.xml \
  --sumo-config=src/automotive/examples/sumo_files_v2v_moscow/moscow.sumo.cfg \
  --simTime=60.0 \
  --send-denm=true \
  --cooperative-detection=true \
  --sumo-gui=true \
  --force-brake-time=30.0 \
  --force-brake-vehicle=veh.0 \
  --force-brake-duration=1.5 \
  --force-brake-target-speed=0.0 \
  --harm-log-radius-m=500.0 \
  --csv-log=moscow_algo \
  --harm-log-file=harm_log_moscow_algo.csv"
```

Note `--force-brake-vehicle=veh.0` — `randomTrips.py` generates ids of
the form `veh.0`, `veh.1`, … (different from the cooperative scene's
`veh0`, `veh1`, `veh2`). If `veh.0` isn't in the route file at t=30,
pick another id from `moscow.rou.xml`.

### Baseline vs algorithm pair

For a side-by-side visual comparison, run twice:

```bash
# Baseline — DENMs broadcast but receivers ignore them
./ns3 run "v2v-emergencyVehicleAlert-nrv2x \
  --sumo-folder=src/automotive/examples/sumo_files_v2v_moscow/ \
  --mob-trace=moscow.rou.xml \
  --sumo-config=src/automotive/examples/sumo_files_v2v_moscow/moscow.sumo.cfg \
  --simTime=60.0 \
  --send-denm=true \
  --cooperative-detection=false \
  --sumo-gui=true \
  --force-brake-time=30.0 \
  --force-brake-vehicle=veh.0 \
  --csv-log=moscow_baseline \
  --harm-log-file=harm_log_moscow_baseline.csv"

# Algorithm
./ns3 run "v2v-emergencyVehicleAlert-nrv2x \
  --sumo-folder=src/automotive/examples/sumo_files_v2v_moscow/ \
  --mob-trace=moscow.rou.xml \
  --sumo-config=src/automotive/examples/sumo_files_v2v_moscow/moscow.sumo.cfg \
  --simTime=60.0 \
  --send-denm=true \
  --cooperative-detection=true \
  --sumo-gui=true \
  --force-brake-time=30.0 \
  --force-brake-vehicle=veh.0 \
  --csv-log=moscow_algo \
  --harm-log-file=harm_log_moscow_algo.csv"

# Compare
python3 sigma_sweep_analyze.py --glob 'harm_log_moscow_*.csv' --plot
```

The `--plot` flag will write `sigma_sweep_harm.png` with four panels
(HARM, min-gap, min-TTC, peak-energy) overlaying baseline and algorithm.

## What to look for in the GUI

At `force_brake_time` the brake event fires on `veh.0`. The DENM
propagates through the NR sidelink. Watch the per-vehicle colour code
(set by the application):
- blue/teal — connected, normal
- red       — emergency event triggered
- yellow    — cooperative optimal deceleration applied (rear DENM within σ)
- orange    — cooperative suboptimal (σ timed out)

In the algorithm run, vehicles near `veh.0` should change colour and
slow down within a few hundred milliseconds of the brake event. In the
baseline run, only `veh.0` reacts; the others continue normally until
their local car-follower forces them to brake.
