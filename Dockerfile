# syntax=docker/dockerfile:1
# ---------------------------------------------------------------------------
# Stage 1: build ubersdr-ntp
#
# ubuntu:24.04 for both stages, as build.sh uses for the release binaries:
# libopus, libcurl and libssl are shared libraries, and the runtime must carry
# the versions the build linked against.
# ---------------------------------------------------------------------------
FROM ubuntu:24.04 AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
        cmake ninja-build g++ pkg-config python3 \
        libopus-dev libcurl4-openssl-dev libssl-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY . .

# The build context has no .git, so there is nothing for the build to describe:
# docker.sh passes the version in. Without it the binary says 0.0.0-unknown.
ARG VERSION=
RUN cmake -S . -B /build -G Ninja -DCMAKE_BUILD_TYPE=Release \
          -DUBERSDR_NTP_VERSION="${VERSION}" \
    && cmake --build /build --target ubersdr-ntp -j "$(nproc)" \
    && /build/ubersdr-ntp --version

# ---------------------------------------------------------------------------
# Stage 2: runtime
# ---------------------------------------------------------------------------
FROM ubuntu:24.04

RUN apt-get update && apt-get install -y --no-install-recommends \
        libopus0 libcurl4t64 libssl3t64 ca-certificates wget libcap2-bin \
    && rm -rf /var/lib/apt/lists/* \
    && useradd -r -s /usr/sbin/nologin ntp

COPY --from=builder /build/ubersdr-ntp /usr/local/bin/ubersdr-ntp
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
# The default configuration, used when /config/config.json is not mounted.
COPY config.addon.json /etc/ubersdr-ntp/config.json

# NTP's port 123 is privileged, and this runs as a user. Docker already lets an
# unprivileged process bind low ports inside a container's own network
# namespace, and the compose file says so explicitly; the capability on the
# binary covers a runtime that does neither.
RUN chmod +x /usr/local/bin/entrypoint.sh \
    && setcap cap_net_bind_service=+ep /usr/local/bin/ubersdr-ntp \
    && apt-get purge -y libcap2-bin && apt-get autoremove -y \
    && mkdir -p /config /var/lib/ubersdr-ntp \
    && chown ntp:ntp /var/lib/ubersdr-ntp

# The drift file. /config is mounted read-only and belongs to the host user, so
# the daemon's own state goes here instead, on a named volume in the compose
# file so it survives the container being recreated by an update. drift_file in
# the configuration overrides it.
ENV UBERSDR_NTP_DRIFT_FILE=/var/lib/ubersdr-ntp/drift

USER ntp

# The status page and JSON API only. NTP (123/udp) is deliberately not exposed.
EXPOSE 6099

# The page, not /api/health: that answers 503 while unsynchronised, which is a
# state to report, not a reason to call the container broken.
HEALTHCHECK --interval=30s --timeout=5s --start-period=10s --retries=3 \
    CMD wget -q -O /dev/null "http://localhost:${WEB_PORT:-6099}/" || exit 1

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
