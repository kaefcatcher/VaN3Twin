#!/usr/bin/env bash
# Drive the baseline + σ-sweep experiment for the cooperative NR-V2X scenario.
#
# Outputs (in the working directory, i.e. project root):
#   - harm_log_baseline.csv
#   - harm_log_sigma_<value>.csv  for each value in SIGMAS
#   - harm_log_sigma_computed.csv
#   - <prefix>-vehX-CAM.csv, -MSGLOG.csv, -COOP.csv  (per-run, per-vehicle)
#
# Notes:
#   - The script assumes ./ns3 is configured + built. Run ./ns3 build once
#     after switching to task/logs_fix.
#   - It overrides the relevant config.json keys via command-line flags so
#     you don't have to edit the JSON between runs.
#   - SUMO GUI is forced OFF for the sweep; with GUI on, each run blocks
#     waiting for a user click.
#   - The sweep deliberately does NOT use `set -e` for the per-run loop —
#     one crashing σ value should not throw away the other runs.

set -uo pipefail

EXAMPLE="v2v-emergencyVehicleAlert-nrv2x"
SIGMAS=("0.0" "0.1" "0.2" "0.5" "1.0" "2.0")
SIM_TIME="${SIM_TIME:-40.0}"   # 40 s is long enough: brake at 15 s + ~25 s tail

# Every run uses the same forced brake event so HARM totals are comparable
# across runs.
COMMON="--send-denm=true \
  --sumo-gui=false \
  --simTime=${SIM_TIME} \
  --force-brake-time=15.0 \
  --force-brake-vehicle=veh0 \
  --force-brake-duration=1.0 \
  --force-brake-target-speed=0.0"

# Run helper: echo a header, run, and tolerate non-zero exit.
run_one () {
  local label="$1"
  shift
  echo "===================================================================="
  echo "==> ${label}"
  echo "===================================================================="
  if ./ns3 run "${EXAMPLE} ${COMMON} $*"; then
    echo "==> ${label}: OK"
  else
    echo "==> ${label}: FAILED (rc=$?)" >&2
  fi
}

# Baseline: DENMs sent, algorithm OFF. Establishes the no-algo reference.
run_one "Baseline (cooperative_detection=false)" \
  --cooperative-detection=false \
  --csv-log=baseline \
  --harm-log-file=harm_log_baseline.csv

# σ sweep: algorithm ON, σ chosen by SigmaMode=fixed at each value.
for sigma in "${SIGMAS[@]}"; do
  run_one "Algorithm run, σ=${sigma}" \
    --cooperative-detection=true \
    --sigma-mode=fixed \
    --fixed-sigma="${sigma}" \
    --csv-log="algo_sigma_${sigma}" \
    --harm-log-file="harm_log_sigma_${sigma}.csv"
done

# Bonus: also run with the computed (closed-form) σ for reference.
run_one "Algorithm run, σ=computed" \
  --cooperative-detection=true \
  --sigma-mode=computed \
  --csv-log=algo_sigma_computed \
  --harm-log-file=harm_log_sigma_computed.csv

echo
echo "===================================================================="
echo "==> Aggregating results"
echo "===================================================================="
python3 sigma_sweep_analyze.py "$@"
