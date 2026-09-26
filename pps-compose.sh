#!/usr/bin/env bash
# pps-compose.sh — map the 1PPS output's serial port into the container, or not
#
# Run by install.sh, start.sh, restart.sh and update.sh before the container
# starts. Nothing to do by hand.
#
# A device cannot be written into docker-compose.yml itself: Compose refuses to
# start a container whose device is missing, which would stop every install
# without one. So the mapping lives in docker-compose.override.yml, which
# Compose merges by itself, and this writes it from pps.device in
# config/config.json when the 1PPS output is enabled and the device is there --
# and removes it when it is not. The device's group is added too, by number:
# the container's own user is not in the host's dialout group by any name.
#
# The configuration allows comments, so it is read by the daemon's own parser,
# run once in the image (--pps-device).
#
# Never fails the start: whatever goes wrong here, the container still comes
# up, and the status page says why the output is not pulsing.

set -uo pipefail

INSTALL_DIR="${INSTALL_DIR:-${HOME}/ubersdr/ntp}"
OVERRIDE="docker-compose.override.yml"
MARK="# Written by pps-compose.sh"

cd "${INSTALL_DIR}" 2>/dev/null || exit 0

remove_ours() {
    if [[ -f "${OVERRIDE}" ]] && head -n1 "${OVERRIDE}" | grep -qF "${MARK}"; then
        rm -f "${OVERRIDE}"
        echo "1PPS: ${1}; removed the device mapping."
    fi
}

image=$(awk '$1 == "image:" { print $2; exit }' docker-compose.yml 2>/dev/null)
image="${image:-madpsy/ubersdr-ntp:latest}"

if [[ ! -f config/config.json ]]; then
    remove_ours "no config/config.json"
    exit 0
fi

errf=$(mktemp)
if ! device=$(docker run --rm --entrypoint /usr/local/bin/ubersdr-ntp \
                  -v "${INSTALL_DIR}/config:/config:ro" "${image}" \
                  --config /config/config.json --pps-device 2>"${errf}"); then
    # The daemon will refuse the same configuration and say why; leave the
    # mapping as it was rather than guess.
    echo "1PPS: could not read config/config.json: $(tail -n1 "${errf}" 2>/dev/null)"
    rm -f "${errf}"
    exit 0
fi
rm -f "${errf}"

if [[ -z "${device}" ]]; then
    remove_ours "the output is off"
    exit 0
fi

if [[ -f "${OVERRIDE}" ]] && ! head -n1 "${OVERRIDE}" | grep -qF "${MARK}"; then
    echo "1PPS: ${OVERRIDE} exists and is not this script's, so it is left alone."
    echo "      Add the device to it yourself:"
    echo "        services: { ubersdr-ntp: { devices: [\"${device}:${device}\"], group_add: [\"<its gid>\"] } }"
    exit 0
fi

real=$(readlink -f "${device}" 2>/dev/null || true)
if [[ -z "${real}" || ! -c "${real}" ]]; then
    remove_ours "${device} is not present on this host"
    echo "1PPS: ${device} is not present; the output will say so until it is plugged in and"
    echo "      ./restart.sh is run."
    exit 0
fi
gid=$(stat -c %g "${real}")

cat > "${OVERRIDE}.tmp" <<EOF
${MARK} from pps.device in config/config.json.
# Rewritten on every start, and removed when the 1PPS output is off: do not edit.
services:
  ubersdr-ntp:
    devices:
      - "${real}:${device}"
    group_add:
      - "${gid}"
EOF
mv "${OVERRIDE}.tmp" "${OVERRIDE}"
echo "1PPS: mapping ${device} (${real}, group ${gid}) into the container."
exit 0
