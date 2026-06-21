#!/usr/bin/env bash
# One-command NR-V2X simulation campaign + academic analysis.
#
#   1. generate highway density route files + all campaign configs
#   2. run the two campaign groups (campaign_main, campaign_network) via run_sweep.sh
#   3. aggregate everything into figures/ (+ campaign_summary.csv) via campaign_analyze.py
#
# Usage:
#   ./run_campaign.sh                 full campaign (generate + run + analyze)
#   ./run_campaign.sh --skip-gen      reuse existing configs, run + analyze
#   ./run_campaign.sh --analyze-only  only (re)build figures from existing results
#
# Prerequisites: ns-3 built (./ns3 configure && ./ns3 build) and SUMO installed.
# Plots need matplotlib (pip install matplotlib); without it only the summary CSV is written.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

PYTHON="${PYTHON:-python3}"
FIGURES_DIR="${FIGURES_DIR:-figures}"

ANALYZE_ONLY=0
SKIP_GEN=0
for arg in "$@"; do
    case "$arg" in
        --analyze-only) ANALYZE_ONLY=1 ;;
        --skip-gen)     SKIP_GEN=1 ;;
        -h|--help)      grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown arg: $arg (try --help)" >&2; exit 1 ;;
    esac
done

if [[ "$ANALYZE_ONLY" -eq 0 ]]; then
    if [[ ! -x ./ns3 && ! -f ./ns3 ]]; then
        echo "ERROR: ./ns3 not found — build ns-3 first (./ns3 configure && ./ns3 build)." >&2
        echo "       (or use --analyze-only to rebuild figures from existing results)" >&2
        exit 1
    fi

    if [[ "$SKIP_GEN" -eq 0 ]]; then
        echo "==> generating highway density files + campaign configs"
        "$PYTHON" scripts/gen_highway_density.py || exit 1
        "$PYTHON" scripts/gen_campaign.py || exit 1
        echo
    fi

    echo "==> running campaign_main (10 runs)"
    ./run_sweep.sh campaign_main
    echo
    echo "==> running campaign_network (9 runs)"
    ./run_sweep.sh campaign_network
    echo
fi

echo "==> aggregating campaign into ${FIGURES_DIR}/"
"$PYTHON" scripts/campaign_analyze.py --out "${FIGURES_DIR}"

echo
echo "Done. See ${FIGURES_DIR}/ (fig1-7 + heatmaps) and ${FIGURES_DIR}/campaign_summary.csv"
