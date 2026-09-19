#!/usr/bin/env bash
# start.sh — start the ubersdr-ntp service

set -euo pipefail

INSTALL_DIR="${HOME}/ubersdr/ntp"

cd "${INSTALL_DIR}"
echo "Starting ubersdr-ntp..."
docker compose up -d --remove-orphans
echo "Done."
echo "  View logs : docker compose logs -f"
echo "  Web UI    : http://your-ubersdr-host/addon/ntp/"
