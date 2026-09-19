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
echo "It listens to the local receiver on 5, 10 and 15 MHz, with time.cloudflare.com"
echo "as its network reference. Edit ${INSTALL_DIR}/${CONFIG_FILE} to change"
echo "that, then run ./restart.sh"
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
