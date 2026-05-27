#!/usr/bin/env bash
# Drive the baseline + σ-sweep experiment for the cooperative NR-V2X scenario.
#
# Outputs (in the working directory, i.e. project root):
#   - harm_log_baseline.csv
#   - harm_log_sigma_<value>.csv  for each value in SIGMAS
#   - <prefix>-vehX-CAM.csv, -MSGLOG.csv, -COOP.csv  (per-run, per-vehicle)
#
# Notes:
#   - The script assumes ./ns3 is configured + built. Run ./ns3 build once
#     after switching to task/logs_fix.
#   - It overrides the relevant config.json keys via command-line flags so
#     you don't have to edit the JSON between runs.
#   - It does not touch send_denm / cooperative_detection in config.json
#     for the sweep runs — those come from --send-denm / --cooperative-detection.

set -euo pipefail

EXAMPLE="v2v-emergencyVehicleAlert-nrv2x"
SIGMAS=("0.0" "0.1" "0.2" "0.5" "1.0" "2.0")

# Every run uses the same forced brake event so HARM totals are comparable
# across runs. Override config.json values that might otherwise drift.
COMMON="--send-denm=true \
  --force-brake-time=15.0 \
  --force-brake-vehicle=veh0 \
  --force-brake-duration=1.0 \
  --force-brake-target-speed=0.0"

# Baseline: DENMs sent, algorithm OFF. Establishes the no-algo reference.
echo "==> Baseline run (cooperative_detection=false)"
./ns3 run "${EXAMPLE} ${COMMON} \
  --cooperative-detection=false \
  --csv-log=baseline \
  --harm-log-file=harm_log_baseline.csv"

# σ sweep: algorithm ON, σ chosen by SigmaMode=fixed at each value.
for sigma in "${SIGMAS[@]}"; do
  echo "==> Algorithm run, σ=${sigma}"
  ./ns3 run "${EXAMPLE} ${COMMON} \
    --cooperative-detection=true \
    --sigma-mode=fixed \
    --fixed-sigma=${sigma} \
    --csv-log=algo_sigma_${sigma} \
    --harm-log-file=harm_log_sigma_${sigma}.csv"
done

# Bonus: also run with the computed (closed-form) σ for reference.
echo "==> Algorithm run, σ=computed"
./ns3 run "${EXAMPLE} ${COMMON} \
  --cooperative-detection=true \
  --sigma-mode=computed \
  --csv-log=algo_sigma_computed \
  --harm-log-file=harm_log_sigma_computed.csv"

echo
echo "==> Aggregating results"
python3 sigma_sweep_analyze.py "$@"
