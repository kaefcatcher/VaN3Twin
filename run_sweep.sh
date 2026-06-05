#!/usr/bin/env bash
set -uo pipefail

SCENARIO="${1:-cooperative}"
shift || true

CONFIGS_DIR="sweep_configs/${SCENARIO}"
if [[ ! -d "${CONFIGS_DIR}" ]]; then
    echo "no config directory: ${CONFIGS_DIR}" >&2
    echo "available: $(ls -1 sweep_configs/ 2>/dev/null | tr '\n' ' ')" >&2
    exit 1
fi

DATE="$(date +%Y-%m-%d)"
RESULTS_ROOT="results/${DATE}_${SCENARIO}"
NRV2X_CONFIG="src/automotive/examples/config.json"
BACKUP="${RESULTS_ROOT}/config.json.original"
EXAMPLE="v2v-emergencyVehicleAlert-nrv2x"

mkdir -p "${RESULTS_ROOT}"
if [[ ! -f "${BACKUP}" ]]; then
    cp "${NRV2X_CONFIG}" "${BACKUP}"
fi
trap 'cp "${BACKUP}" "${NRV2X_CONFIG}"' EXIT

run_variant() {
    local cfg="$1"
    local variant
    variant="$(basename "${cfg}" .json)"
    local run_dir="${RESULTS_ROOT}/${DATE}_${SCENARIO}_${variant}"
    local logs_csv="${run_dir}/cam_msg_logs"
    local logs_harm="${run_dir}/harm_log"
    local logs_run="${run_dir}/run_log"
    mkdir -p "${logs_csv}" "${logs_harm}" "${logs_run}"

    cp "${cfg}" "${NRV2X_CONFIG}"

    echo "==> ${variant}"
    if ./ns3 run "${EXAMPLE}" >"${logs_run}/run.log" 2>&1; then
        echo "    OK    ${run_dir}"
    else
        echo "    FAIL  ${run_dir}" >&2
    fi

    find . -maxdepth 1 -name "run-*-CAM.csv"     -exec mv {} "${logs_csv}/" \; 2>/dev/null || true
    find . -maxdepth 1 -name "run-*-MSGLOG.csv"  -exec mv {} "${logs_csv}/" \; 2>/dev/null || true
    find . -maxdepth 1 -name "run-*-COOP.csv"    -exec mv {} "${logs_csv}/" \; 2>/dev/null || true
    find . -maxdepth 1 -name "*_prr_per_vehicle_messagetype.csv" -exec mv {} "${run_dir}/" \; 2>/dev/null || true
    [[ -f harm_log.csv ]] && mv harm_log.csv "${logs_harm}/" 2>/dev/null || true
}

echo "scenario  : ${SCENARIO}"
echo "configs   : ${CONFIGS_DIR}"
echo "results   : ${RESULTS_ROOT}"
echo

for cfg in "${CONFIGS_DIR}"/*.json; do
    run_variant "${cfg}"
done

echo
echo "==> analysis"
python3 scripts/analyze.py "${RESULTS_ROOT}" --plots-dir "${RESULTS_ROOT}/plots" "$@"
