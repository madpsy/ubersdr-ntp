#!/usr/bin/env bash
# stop.sh — stop the ubersdr-ntp service

set -euo pipefail

INSTALL_DIR="${HOME}/ubersdr/ntp"

cd "${INSTALL_DIR}"
echo "Stopping ubersdr-ntp..."
docker compose down
echo "Done."
