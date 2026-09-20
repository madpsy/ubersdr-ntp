#!/bin/sh
# entrypoint.sh — start ubersdr-ntp with the addon's configuration
#
# The configuration is a file, not environment variables: the sources are a
# list, each with its own URL, carrier and password, which does not flatten
# into variables worth having. install.sh puts it at
# ~/ubersdr/ntp/config/config.json, mounted here at /config/config.json; with
# nothing mounted, the default built into the image is used.
#
# Environment variables (all optional):
#   WEB_PORT             Status page / JSON API port (default: the config's, 6099)
#   LOG_LEVEL            trace, debug, info, warn, error (default: the config's)
#   UBERSDR_INGEST_URL   UberSDR's addon MQTT ingest port (default:
#                        http://ubersdr:6926); read by the daemon itself

set -e

config=/config/config.json
if [ ! -f "$config" ]; then
    echo "entrypoint: no $config mounted; using the built-in default" \
         "(the local receiver on 5, 10 and 15 MHz, and three upstream servers)" >&2
    config=/etc/ubersdr-ntp/config.json
fi

set -- --config "$config" "$@"
[ -n "$WEB_PORT" ]  && set -- "$@" --http-port "$WEB_PORT"
[ -n "$LOG_LEVEL" ] && set -- "$@" --log-level "$LOG_LEVEL"

exec /usr/local/bin/ubersdr-ntp "$@"
