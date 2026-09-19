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
echo "Restarting service..."
docker compose up -d --remove-orphans
echo "Done."
echo "  View logs : docker compose logs -f"
echo "  Web UI    : http://your-ubersdr-host/addon/ntp/"
