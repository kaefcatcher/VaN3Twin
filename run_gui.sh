#!/usr/bin/env bash
set -uo pipefail
CONFIG_SOURCE="${1:-sweep_configs/cooperative/sigma_0.5.json}"
NRV2X_CONFIG="src/automotive/examples/config.json"
BACKUP="${NRV2X_CONFIG}.gui_backup"
[[ -f "${BACKUP}" ]] || cp "${NRV2X_CONFIG}" "${BACKUP}"
trap 'cp "${BACKUP}" "${NRV2X_CONFIG}"' EXIT
python3 - "${CONFIG_SOURCE}" "${NRV2X_CONFIG}" <<'PY'
import json, sys
src, dst = sys.argv[1], sys.argv[2]
cfg = json.load(open(src))
cfg["sumo_gui"] = True
json.dump(cfg, open(dst, "w"), indent=2)
PY
./ns3 run "v2v-emergencyVehicleAlert-nrv2x"
