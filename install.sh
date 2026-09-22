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
# WWV or DCF77
# ---------------------------------------------------------------------------

# DCF77's primary antenna at Mainflingen (Propagation.cpp, dcf77Site), and the
# reach of its groundwave. Inside that, DCF77 is the better station: longwave
# does not die at night the way HF does, and its phase modulation times the
# second far more finely than WWV's audio ticks.
DCF77_LAT=50.015528
DCF77_LON=9.008515
DCF77_RANGE_KM=2000
STATION=""   # empty when an existing configuration was kept

# The receiver's /api/description: from the port UberSDR publishes on the
# host, or failing that from inside its Docker network, where it is always
# "ubersdr:8080".
fetch_description() {
    curl -fsS --max-time 5 http://localhost:8080/api/description 2>/dev/null && return 0
    docker run --rm --network ubersdr_sdr-network --entrypoint wget \
        madpsy/ubersdr-ntp:latest -q -T 5 -O - http://ubersdr:8080/api/description 2>/dev/null
}

# On a fresh configuration only: DCF77 if the receiver is within its range and
# can tune 77.5 kHz, otherwise WWV. Anything missing or unexpected is WWV.
choose_station() {
    local desc gps lat lon minf km
    STATION="wwv"
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
    km="$(awk -v la1="${lat}" -v lo1="${lon}" -v la2="${DCF77_LAT}" -v lo2="${DCF77_LON}" 'BEGIN {
        r = atan2(0, -1) / 180
        dla = (la2 - la1) * r; dlo = (lo2 - lo1) * r
        h = sin(dla/2)^2 + cos(la1*r) * cos(la2*r) * sin(dlo/2)^2
        printf "%.0f", 2 * 6371 * atan2(sqrt(h), sqrt(1 - h))
    }')"
    echo "  Receiver at ${lat}, ${lon}: ${km} km from DCF77"
    if (( km > DCF77_RANGE_KM )); then
        echo "  Beyond DCF77's ${DCF77_RANGE_KM} km groundwave — using WWV"
        return
    fi
    if [[ -z "${minf}" ]] || awk -v f="${minf}" 'BEGIN { exit !(f > 77500) }'; then
        echo "  The receiver does not report tuning down to 77.5 kHz — using WWV"
        return
    fi
    STATION="dcf77"
    echo "  Within range and the receiver covers LF — using DCF77 instead of WWV"
}

# The radio sources for a station, one per line, as they go in the config.
station_sources() {
    local url="http://ubersdr:8080"
    case "$1" in
        dcf77)
            echo "    { \"name\": \"dcf77\",    \"url\": \"${url}\", \"carrier_hz\": 77500, \"extra_delay_ms\": 0.0 }"
            ;;
        *)
            echo "    { \"name\": \"local-5\",  \"url\": \"${url}\", \"carrier_hz\": 5000000, \"extra_delay_ms\": 0.0 },"
            echo "    { \"name\": \"local-10\", \"url\": \"${url}\", \"carrier_hz\": 10000000, \"extra_delay_ms\": 0.0 },"
            echo "    { \"name\": \"local-15\", \"url\": \"${url}\", \"carrier_hz\": 15000000, \"extra_delay_ms\": 0.0 }"
            ;;
    esac
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
        STATION="wwv"
        return
    fi
    block="$(station_sources "${STATION}")"
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
    choose_station
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
case "${STATION}" in
    dcf77) echo "It listens to DCF77 on 77.5 kHz through the local receiver, with Cloudflare" ;;
    wwv)   echo "It listens to WWV through the local receiver on 5, 10 and 15 MHz, with Cloudflare" ;;
esac
if [[ -n "${STATION}" ]]; then
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
