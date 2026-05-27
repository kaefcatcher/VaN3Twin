#!/usr/bin/env bash
# Drive the baseline + σ-sweep experiment.
#
# Two scenarios available, selected by the first positional argument:
#
#   cooperative   (default) — SUMO Krauss model with safe followers.
#                              No crashes ever happen; comparison uses
#                              pairwise HARM, gap, TTC, collision energy.
#
#   crash         — followers have decel=1.5, tau=0.2, minGap=0.1, so
#                              when V1 emergency-brakes the followers
#                              CANNOT stop in time.
#                              SUMO --collision.action warn lets the
#                              run continue past collisions so we can
#                              count them and observe peak energy.
#                              Baseline ⇒ crashes.
#                              Algorithm ⇒ should prevent / soften.
#
# Outputs (in the project root):
#   - harm_log_baseline.csv
#   - harm_log_sigma_<value>.csv  for each σ in SIGMAS
#   - harm_log_sigma_computed.csv
#   - <prefix>-vehX-CAM.csv, -MSGLOG.csv, -COOP.csv  (per-run, per-vehicle)

set -uo pipefail

EXAMPLE="v2v-emergencyVehicleAlert-nrv2x"
SIGMAS=("0.0" "0.1" "0.2" "0.5" "1.0" "2.0")
SIM_TIME="${SIM_TIME:-40.0}"
SCENARIO="${1:-cooperative}"

case "${SCENARIO}" in
  cooperative)
    SUMO_FOLDER="src/automotive/examples/sumo_files_v2v_cooperative/"
    MOB_TRACE="cooperative.rou.xml"
    SUMO_CONFIG="src/automotive/examples/sumo_files_v2v_cooperative/cooperative.sumo.cfg"
    ;;
  crash)
    SUMO_FOLDER="src/automotive/examples/sumo_files_v2v_crash/"
    MOB_TRACE="crash.rou.xml"
    SUMO_CONFIG="src/automotive/examples/sumo_files_v2v_crash/crash.sumo.cfg"
    ;;
  *)
    echo "Unknown scenario: ${SCENARIO}" >&2
    echo "Usage: $0 [cooperative|crash] [--plot]" >&2
    exit 1
    ;;
esac
shift || true   # remaining args go to the analyzer

echo "Sweep scenario: ${SCENARIO}"
echo "  sumo folder:   ${SUMO_FOLDER}"
echo "  rou file:      ${MOB_TRACE}"
echo "  sim time:      ${SIM_TIME} s"
echo

COMMON="--send-denm=true \
  --sumo-gui=false \
  --simTime=${SIM_TIME} \
  --sumo-folder=${SUMO_FOLDER} \
  --mob-trace=${MOB_TRACE} \
  --sumo-config=${SUMO_CONFIG} \
  --force-brake-time=15.0 \
  --force-brake-vehicle=veh0 \
  --force-brake-duration=1.0 \
  --force-brake-target-speed=0.0"

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

run_one "Baseline (cooperative_detection=false)" \
  --cooperative-detection=false \
  --csv-log=baseline \
  --harm-log-file=harm_log_baseline.csv

for sigma in "${SIGMAS[@]}"; do
  run_one "Algorithm run, σ=${sigma}" \
    --cooperative-detection=true \
    --sigma-mode=fixed \
    --fixed-sigma="${sigma}" \
    --csv-log="algo_sigma_${sigma}" \
    --harm-log-file="harm_log_sigma_${sigma}.csv"
done

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
