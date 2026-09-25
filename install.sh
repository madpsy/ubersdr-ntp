#!/usr/bin/env bash
# install.sh — fetch the docker-compose.yml from the ubersdr-ntp repo and start the service
#
# Requires UberSDR to be installed and running first: https://ubersdr.org
#
# Usage:
#   curl -fsSL https://raw.githubusercontent.com/madpsy/ubersdr-ntp/main/install.sh | bash
#   — or —
#   ./install.sh [--force-update]
#
# Options:
#   --force-update   Overwrite an existing docker-compose.yml (default: skip if present).
#                    config/config.json is never overwritten: it is yours.
#
# When piping through bash, pass the flag via env var instead:
#   curl -fsSL ... | FORCE_UPDATE=1 bash

set -euo pipefail

REPO_RAW="https://raw.githubusercontent.com/madpsy/ubersdr-ntp/main"
INSTALL_DIR="${HOME}/ubersdr/ntp"
COMPOSE_FILE="docker-compose.yml"
CONFIG_DIR="config"
CONFIG_FILE="${CONFIG_DIR}/config.json"
FORCE_UPDATE="${FORCE_UPDATE:-0}"

# Parse flags when run directly (not piped)
for arg in "$@"; do
    case "$arg" in
        --force-update) FORCE_UPDATE=1 ;;
        *) echo "Unknown argument: $arg" >&2; exit 1 ;;
    esac
done

die() { echo "error: $*" >&2; exit 1; }

# ---------------------------------------------------------------------------
# WWV, and the longwave stations in range
# ---------------------------------------------------------------------------

# The LF transmitters (Propagation.cpp) and the reach of their groundwave.
# Inside that, each is added to WWV rather than replacing it: longwave does not
# die at night the way HF does, and DCF77's and Allouis's phase modulation
# times the second far more finely than WWV's audio ticks, while WWV keeps the
# clock going if the LF side is lost. 2000 km for all three.
LF_RANGE_KM=2000
#          name     lat        lon        carrier_hz
LF_STATIONS=(
    "dcf77   50.015528  9.008515   77500"
    "msf     54.916667  -3.250000  60000"
    "allouis 47.168056  2.200556   162000"
)
STATIONS=()   # the LF stations chosen; empty on a WWV-only or kept configuration
KEPT_CONFIG=1  # cleared when this install writes the sources

# The receiver's /api/description: from the port UberSDR publishes on the
# host, or failing that from inside its Docker network, where it is always
# "ubersdr:8080".
fetch_description() {
    curl -fsS --max-time 5 http://localhost:8080/api/description 2>/dev/null && return 0
    docker run --rm --network ubersdr_sdr-network --entrypoint wget \
        madpsy/ubersdr-ntp:latest -q -T 5 -O - http://ubersdr:8080/api/description 2>/dev/null
}

# On a fresh configuration only: WWV, plus each LF station whose transmitter
# the receiver is within LF_RANGE_KM of and whose carrier it can tune down to.
# Anything missing or unexpected is WWV alone.
choose_stations() {
    local desc gps lat lon minf km entry name slat slon carrier
    KEPT_CONFIG=0
    STATIONS=()
    echo "Asking the local receiver where it is..."
    if ! desc="$(fetch_description)" || [[ -z "${desc}" ]]; then
        echo "  No answer from the receiver — using WWV"
        return
    fi
    # UberSDR's JSON is compact and these objects are flat, so no parser is
    # needed: receiver.gps is the one "gps" object ("gpsdo" is another key).
    # Each allowed to find nothing: under pipefail a missing key would
    # otherwise end the install.
    gps="$(grep -o '"gps":{[^}]*}' <<<"${desc}" | head -n1 || true)"
    lat="$(grep -oE '"lat":-?[0-9.]+' <<<"${gps}" | cut -d: -f2 || true)"
    lon="$(grep -oE '"lon":-?[0-9.]+' <<<"${gps}" | cut -d: -f2 || true)"
    minf="$(grep -o '"tuning_range":{[^}]*}' <<<"${desc}" \
            | grep -oE '"min_frequency":[0-9.]+' | cut -d: -f2 || true)"
    if [[ -z "${lat}" || -z "${lon}" ]] \
       || awk -v a="${lat}" -v b="${lon}" 'BEGIN { exit !(a*a < 1e-4 && b*b < 1e-4) }'; then
        echo "  The receiver publishes no location — using WWV"
        return
    fi
    echo "  Receiver at ${lat}, ${lon}"
    for entry in "${LF_STATIONS[@]}"; do
        read -r name slat slon carrier <<<"${entry}"
        km="$(awk -v la1="${lat}" -v lo1="${lon}" -v la2="${slat}" -v lo2="${slon}" 'BEGIN {
            r = atan2(0, -1) / 180
            dla = (la2 - la1) * r; dlo = (lo2 - lo1) * r
            h = sin(dla/2)^2 + cos(la1*r) * cos(la2*r) * sin(dlo/2)^2
            printf "%.0f", 2 * 6371 * atan2(sqrt(h), sqrt(1 - h))
        }')"
        if (( km > LF_RANGE_KM )); then
            echo "  ${name}: ${km} km, beyond ${LF_RANGE_KM} km of groundwave — not added"
        elif [[ -z "${minf}" ]] || awk -v f="${minf}" -v c="${carrier}" 'BEGIN { exit !(f > c) }'; then
            echo "  ${name}: ${km} km, but the receiver does not report tuning down to ${carrier} Hz — not added"
        else
            echo "  ${name}: ${km} km — added"
            STATIONS+=("${name}")
        fi
    done
    (( ${#STATIONS[@]} )) || echo "  No longwave station in range — using WWV"
}

# The radio sources for the chosen stations, one per line, as they go in the
# config: the LF stations in range first, then WWV always.
station_sources() {
    local url="http://ubersdr:8080" name
    # ${STATIONS[@]+...}: on bash before 4.4, set -u takes an empty array as unbound.
    for name in ${STATIONS[@]+"${STATIONS[@]}"}; do
        case "${name}" in
            dcf77)   echo "    { \"name\": \"dcf77\",    \"url\": \"${url}\", \"carrier_hz\": 77500, \"extra_delay_ms\": 0.0 }," ;;
            msf)     echo "    { \"name\": \"msf\",      \"url\": \"${url}\", \"carrier_hz\": 60000, \"extra_delay_ms\": 0.0 }," ;;
            allouis) echo "    { \"name\": \"allouis\",  \"url\": \"${url}\", \"carrier_hz\": 162000, \"extra_delay_ms\": 0.0 }," ;;
        esac
    done
    echo "    { \"name\": \"local-5\",  \"url\": \"${url}\", \"carrier_hz\": 5000000, \"extra_delay_ms\": 0.0 },"
    echo "    { \"name\": \"local-10\", \"url\": \"${url}\", \"carrier_hz\": 10000000, \"extra_delay_ms\": 0.0 },"
    echo "    { \"name\": \"local-15\", \"url\": \"${url}\", \"carrier_hz\": 15000000, \"extra_delay_ms\": 0.0 }"
}

# Replace the block config.addon.json marks with "// >>> station" and
# "// <<< station" by the chosen station's sources. The markers are comments,
# so the downloaded file is valid as it stands and stays WWV if they are not
# found exactly once each.
write_sources() {
    local block tmp
    if [[ "$(grep -c '^    // >>> station$' "${CONFIG_FILE}")" != 1 \
          || "$(grep -c '^    // <<< station$' "${CONFIG_FILE}")" != 1 ]]; then
        echo "  ${CONFIG_FILE} has no station block to fill in — leaving it on WWV"
        STATIONS=()
        return
    fi
    block="$(station_sources)"
    tmp="${CONFIG_FILE}.tmp"
    # Through the environment, not -v: -v would interpret backslashes, and
    # not every awk accepts a newline in one.
    BLOCK="${block}" awk '
        /^    \/\/ >>> station$/ { print "    // The station install.sh chose for this receiver."; print ENVIRON["BLOCK"]; skip = 1; next }
        /^    \/\/ <<< station$/ { skip = 0; next }
        !skip
    ' "${CONFIG_FILE}" > "${tmp}"
    mv "${tmp}" "${CONFIG_FILE}"
}

# ---------------------------------------------------------------------------
# Dependency checks
# ---------------------------------------------------------------------------

command -v docker >/dev/null || die "docker not found in PATH — please install Docker first"
docker compose version >/dev/null 2>&1 || die "docker compose plugin not found — please install Docker Compose v2"

# ---------------------------------------------------------------------------
# Prepare install directory
# ---------------------------------------------------------------------------

mkdir -p "${INSTALL_DIR}"
cd "${INSTALL_DIR}"

# ---------------------------------------------------------------------------
# Fetch compose file
# ---------------------------------------------------------------------------

if [[ -f "${COMPOSE_FILE}" && "${FORCE_UPDATE}" != "1" ]]; then
    echo "${COMPOSE_FILE} already exists — skipping download (use --force-update to overwrite)"
else
    echo "Fetching ${COMPOSE_FILE} from GitHub..."
    curl -fsSL "${REPO_RAW}/${COMPOSE_FILE}" -o "${COMPOSE_FILE}"
    echo "Saved ${COMPOSE_FILE}"
fi

# ---------------------------------------------------------------------------
# Fetch the configuration, once
# ---------------------------------------------------------------------------

# The receivers, frequencies and upstream NTP servers. Never overwritten, even
# with --force-update: it is the one file here that is meant to be edited.
mkdir -p "${CONFIG_DIR}"
if [[ -f "${CONFIG_FILE}" ]]; then
    echo "${CONFIG_FILE} already exists — keeping it"
else
    echo "Fetching the default configuration..."
    curl -fsSL "${REPO_RAW}/config.addon.json" -o "${CONFIG_FILE}"
    echo "Saved ${CONFIG_FILE}"
    choose_stations
    write_sources
fi
# Readable by the container's own user, which is not this one.
chmod 755 "${CONFIG_DIR}"
chmod 644 "${CONFIG_FILE}"

# ---------------------------------------------------------------------------
# Fetch helper scripts
# ---------------------------------------------------------------------------

for script in update.sh start.sh stop.sh restart.sh; do
    echo "Fetching ${script}..."
    curl -fsSL "${REPO_RAW}/${script}" -o "${script}"
    chmod +x "${script}"
    echo "Saved ${script}"
done

# ---------------------------------------------------------------------------
# Pull image and start service
# ---------------------------------------------------------------------------

echo "Pulling latest Docker image..."
docker compose pull

echo "Starting ubersdr-ntp..."
docker compose up -d --remove-orphans --force-recreate

echo ""
echo "Done. ubersdr-ntp is running."
echo "  View logs  : docker compose logs -f"
echo "  Stop       : ./stop.sh"
echo "  Start      : ./start.sh"
echo "  Restart    : ./restart.sh"
echo "  Update     : ./update.sh"
echo ""
if (( ! KEPT_CONFIG )); then
    if (( ${#STATIONS[@]} )); then
        lf="${STATIONS[*]}"
        echo "It listens to ${lf// /, } and WWV (5, 10 and 15 MHz) through the local receiver, with Cloudflare"
    else
        echo "It listens to WWV through the local receiver on 5, 10 and 15 MHz, with Cloudflare"
    fi
fi
if (( ! KEPT_CONFIG )); then
    echo "as its network reference. Edit ${INSTALL_DIR}/${CONFIG_FILE}, where every"
    echo "setting is documented, then run ./restart.sh"
else
    echo "It uses your existing ${INSTALL_DIR}/${CONFIG_FILE}; edit it, then run"
    echo "./restart.sh"
fi
echo ""
echo "NTP itself (port 123/udp) is not published outside Docker yet."
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  UBERSDR PROXY CONFIGURATION"
echo ""
echo "  Add this addon via the UberSDR Admin → Addon Proxies interface:"
echo ""
echo "    Name         : ntp"
echo "    Host         : ntp"
echo "    Port         : 6099"
echo "    Enabled      : true"
echo "    Strip prefix : true"
echo "    Rate Limit   : 100"
echo ""
echo "  Then access the status page at: http://your-ubersdr-host/addon/ntp/"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
