#!/usr/bin/env bash
# Drive the baseline + σ-sweep experiment by editing config.json between
# runs. This is the JSON-driven flow: every knob that varies sweep-to-
# sweep is set in src/automotive/examples/config.json, the scenario is
# invoked with no CLI overrides, and all output files are routed into
# sweep_results/ so the project root stays clean.
#
# Layout produced by a sweep:
#   sweep_results/
#     logs/
#       harm_log_baseline.csv
#       harm_log_sigma_<value>.csv
#       <prefix>-vehX-CAM.csv  -MSGLOG.csv  -COOP.csv
#       run_*.log              full stdout/stderr of each iteration
#     plots/
#       sigma_sweep_harm.png   four-panel comparison (only with --plot)
#
# Usage:
#   ./run_sigma_sweep.sh [cooperative|crash] [--plot]
#
# Tunables (env vars):
#   SIM_TIME   default 22.5
#   OUT_DIR    default sweep_results

set -uo pipefail

EXAMPLE="v2v-emergencyVehicleAlert-nrv2x"
SIGMAS=("0.0" "0.1" "0.2" "0.5" "1.0" "2.0")
SIM_TIME="${SIM_TIME:-22.5}"
OUT_DIR="${OUT_DIR:-sweep_results}"
LOGS_DIR="${OUT_DIR}/logs"
PLOTS_DIR="${OUT_DIR}/plots"

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

# --- Output directories ----------------------------------------------------
mkdir -p "${LOGS_DIR}" "${PLOTS_DIR}"

# --- Config-file plumbing --------------------------------------------------
CONFIG_PATH="src/automotive/examples/config.json"
CONFIG_BACKUP="${OUT_DIR}/config.json.original"

# One-time backup so the user's original config is restored at the end.
if [[ ! -f "${CONFIG_BACKUP}" ]]; then
  cp "${CONFIG_PATH}" "${CONFIG_BACKUP}"
fi
# On any exit (success, error, or interrupt) put the original back so
# the working tree isn't left in sweep state.
trap 'cp "${CONFIG_BACKUP}" "${CONFIG_PATH}"' EXIT

# Python helper that mutates config.json with the given overrides.
write_config () {
  local cooperative_detection="$1"
  local sigma_mode="$2"
  local fixed_sigma="$3"
  local csv_log="$4"
  local harm_log_file="$5"
  python3 - "${CONFIG_BACKUP}" "${CONFIG_PATH}" \
    "${cooperative_detection}" "${sigma_mode}" "${fixed_sigma}" \
    "${csv_log}" "${harm_log_file}" \
    "${SUMO_FOLDER}" "${MOB_TRACE}" "${SUMO_CONFIG}" "${SIM_TIME}" <<'EOF'
import json, sys
(_, base, dst, coop, sm, fs, csv_log, harm_log, sf, mt, sc, sim_time) = sys.argv
cfg = json.load(open(base))
cfg["cooperative_detection"] = (coop == "true")
cfg["sigma_mode"]            = sm
cfg["fixed_sigma"]           = float(fs)
cfg["csv_log"]               = csv_log
cfg["harm_log_file"]         = harm_log
cfg["sumo_folder"]           = sf
cfg["mob_trace"]             = mt
cfg["sumo_config"]           = sc
cfg["simTime"]               = float(sim_time)
cfg["sumo_gui"]              = False
cfg["send_denm"]             = True
# Force a stable, repeatable brake event across iterations.
cfg["force_brake_time"]      = 15.0
cfg["force_brake_vehicle"]   = "veh0"
cfg["force_brake_duration"]  = 1.0
cfg["force_brake_target_speed"] = 0.0
json.dump(cfg, open(dst, "w"), indent=2)
EOF
}

# Each run is logged in full to sweep_results/logs/run_<label>.log so the
# console output stays manageable but you can still inspect everything.
run_one () {
  local label="$1"; local logfile="${LOGS_DIR}/run_${label}.log"
  shift
  local coop="$1" sm="$2" fs="$3"
  local prefix="${LOGS_DIR}/${label}"
  local harm="${LOGS_DIR}/harm_log_${label}.csv"
  echo "===================================================================="
  echo "==> ${label}  (cooperative=${coop} sigma_mode=${sm} fixed_sigma=${fs})"
  echo "===================================================================="
  write_config "${coop}" "${sm}" "${fs}" "${prefix}" "${harm}"
  if ./ns3 run "${EXAMPLE}" >"${logfile}" 2>&1; then
    echo "    OK    (log: ${logfile})"
  else
    echo "    FAIL  (log: ${logfile})" >&2
  fi
}

echo "Sweep scenario : ${SCENARIO}"
echo "SUMO folder    : ${SUMO_FOLDER}"
echo "Sim time       : ${SIM_TIME} s"
echo "Output dir     : ${OUT_DIR}"
echo "Sigmas         : ${SIGMAS[*]}"
echo

# Baseline run (no algorithm).
run_one "baseline"  "false" "fixed"    "0.0"

# σ sweep with the algorithm on, fixed σ at each value.
for sigma in "${SIGMAS[@]}"; do
  run_one "sigma_${sigma}" "true" "fixed" "${sigma}"
done

# Computed-σ reference.
run_one "sigma_computed" "true" "computed" "0.5"

echo
echo "===================================================================="
echo "==> Aggregating results"
echo "===================================================================="
python3 sigma_sweep_analyze.py \
  --glob "${LOGS_DIR}/harm_log_*.csv" \
  --plots-dir "${PLOTS_DIR}" \
  "$@"
