#!/usr/bin/env bash
# Build a SUMO scenario from real OpenStreetMap data around central Moscow.
#
# Produces, in this directory:
#   moscow.osm.xml         raw OSM download
#   moscow.net.xml         SUMO net (with UTM projection so DENMs encode)
#   moscow.poly.xml        background polygons (buildings, parks) — eye candy
#                          in sumo-gui; safe to skip if osmPolyconvert is
#                          unavailable.
#   moscow.rou.xml         ~80 random trips
#   moscow.sumo.cfg        ties them together with collision.action warn
#
# Requirements:
#   - SUMO toolchain in PATH: netconvert, polyconvert, randomTrips.py
#   - curl
#
# Tunables: set MOSCOW_RADIUS_M / MOSCOW_TRIPS / MOSCOW_SEED in the env.
set -euo pipefail

cd "$(dirname "$0")"

# Geographic centre — Tverskaya / Pushkin Square, central Moscow.
CENTRE_LAT="${MOSCOW_LAT:-55.7647}"
CENTRE_LON="${MOSCOW_LON:-37.6058}"
RADIUS_M="${MOSCOW_RADIUS_M:-600}"   # ~600 m radius = ~1.2 × 1.2 km box
NUM_TRIPS="${MOSCOW_TRIPS:-80}"
SEED="${MOSCOW_SEED:-42}"

# Compute the OSM bounding box from centre + radius (1° lat ≈ 111 320 m).
DEG_PER_M_LAT=$(python3 -c "print(1.0 / 111320.0)")
DEG_PER_M_LON=$(python3 -c "import math; print(1.0 / (111320.0 * math.cos(math.radians(${CENTRE_LAT}))))")
DLAT=$(python3 -c "print(${RADIUS_M} * ${DEG_PER_M_LAT})")
DLON=$(python3 -c "print(${RADIUS_M} * ${DEG_PER_M_LON})")
LATMIN=$(python3 -c "print(${CENTRE_LAT} - ${DLAT})")
LATMAX=$(python3 -c "print(${CENTRE_LAT} + ${DLAT})")
LONMIN=$(python3 -c "print(${CENTRE_LON} - ${DLON})")
LONMAX=$(python3 -c "print(${CENTRE_LON} + ${DLON})")

echo "Moscow scenario builder"
echo "  centre    : (${CENTRE_LAT}, ${CENTRE_LON})"
echo "  radius    : ${RADIUS_M} m"
echo "  bbox      : ${LONMIN},${LATMIN},${LONMAX},${LATMAX}"
echo "  trips     : ${NUM_TRIPS}"
echo "  seed      : ${SEED}"

# --- 1. Download OSM data via Overpass mirror -----------------------------
if [[ ! -s moscow.osm.xml ]]; then
  echo "==> Downloading OSM extract"
  curl -sS "https://overpass-api.de/api/map?bbox=${LONMIN},${LATMIN},${LONMAX},${LATMAX}" \
    -o moscow.osm.xml
fi

# --- 2. Convert to SUMO net with UTM projection (zone 37 for Moscow) -----
echo "==> netconvert"
netconvert \
  --osm-files moscow.osm.xml \
  --output-file moscow.net.xml \
  --proj.utm \
  --geometry.remove --roundabouts.guess \
  --ramps.guess --junctions.join \
  --tls.guess-signals --tls.discard-simple --tls.join \
  --remove-edges.by-vclass rail_slow,rail_fast,rail,bicycle,pedestrian \
  --keep-edges.by-vclass passenger \
  --no-turnarounds true \
  --no-warnings

# --- 3. Background polygons for the GUI ----------------------------------
if command -v polyconvert >/dev/null 2>&1; then
  echo "==> polyconvert (background polygons for sumo-gui)"
  polyconvert \
    --osm-files moscow.osm.xml \
    --net-file moscow.net.xml \
    --output-file moscow.poly.xml \
    --no-warnings || echo "  (polyconvert failed — continuing without polygons)"
fi

# --- 4. Random trips, sample period chosen to yield ~NUM_TRIPS over 60 s --
echo "==> randomTrips.py"
PERIOD=$(python3 -c "print(60.0 / max(1, ${NUM_TRIPS}))")
python3 "$(dirname "$(which sumo 2>/dev/null || true)")/../tools/randomTrips.py" \
  --net-file moscow.net.xml \
  --output-trip-file moscow.trips.xml \
  --route-file moscow.rou.xml \
  --vehicle-class passenger \
  --period "${PERIOD}" \
  --begin 0 --end 60 \
  --seed "${SEED}" \
  --fringe-factor 5 \
  --validate \
  2>/dev/null || {
    # Fallback if randomTrips is in a non-standard place.
    randomTrips.py \
      --net-file moscow.net.xml \
      --output-trip-file moscow.trips.xml \
      --route-file moscow.rou.xml \
      --vehicle-class passenger \
      --period "${PERIOD}" \
      --begin 0 --end 60 \
      --seed "${SEED}" \
      --fringe-factor 5 \
      --validate
  }

# --- 5. sumo.cfg ----------------------------------------------------------
cat > moscow.sumo.cfg <<EOF
<configuration>
    <input>
        <net-file value="moscow.net.xml"/>
        <route-files value="moscow.rou.xml"/>
$(test -f moscow.poly.xml && echo '        <additional-files value="moscow.poly.xml"/>')
        <gui-settings-file value="file.settings.xml"/>
    </input>
    <time>
        <begin value="0"/>
        <end value="60"/>
        <step-length value="0.01"/>
    </time>
    <processing>
        <collision.action value="warn"/>
        <collision.check-junctions value="false"/>
    </processing>
</configuration>
EOF

# --- 6. Minimal GUI settings file -----------------------------------------
if [[ ! -s file.settings.xml ]]; then
  cat > file.settings.xml <<'EOF'
<viewsettings>
    <scheme name="real world"/>
    <delay value="50"/>
</viewsettings>
EOF
fi

echo
echo "Done. Files in $(pwd):"
ls -la moscow.* file.settings.xml 2>/dev/null
echo
echo "To run the scenario, set these JSON keys (or pass CLI flags) and"
echo "launch the NR example:"
echo "  sumo_folder : src/automotive/examples/sumo_files_v2v_moscow/"
echo "  mob_trace   : moscow.rou.xml"
echo "  sumo_config : src/automotive/examples/sumo_files_v2v_moscow/moscow.sumo.cfg"
echo "  simTime     : 60"
echo "  force_brake_time     : 30.0"
echo "  force_brake_vehicle  : 0  (or another id from moscow.rou.xml)"
echo "  harm_log_radius_m    : 500"
