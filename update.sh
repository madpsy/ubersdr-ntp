#!/usr/bin/env bash
# update.sh — pull the latest ubersdr-ntp image and restart the service
#
# Usage:
#   ./update.sh

set -euo pipefail

INSTALL_DIR="${HOME}/ubersdr/ntp"

cd "${INSTALL_DIR}"
echo "Pulling latest ubersdr-ntp image..."
docker compose pull
# The 1PPS output's serial port, mapped in when it is enabled (pps-compose.sh).
if [[ -x ./pps-compose.sh ]]; then ./pps-compose.sh || true; fi
echo "Restarting service..."
docker compose up -d --remove-orphans
echo "Done."
echo "  View logs : docker compose logs -f"
echo "  Web UI    : http://your-ubersdr-host/addon/ntp/"
