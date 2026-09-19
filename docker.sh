#!/usr/bin/env bash
# docker.sh — build the ubersdr-ntp Docker image
#
# The binary is built from source inside the Docker image, on ubuntu:24.04 as
# build.sh builds the release binaries. No host binaries are required.
#
# Usage:
#   ./docker.sh [build|push|run|arm64]
#
#   build  — build the image for linux/amd64 (default, local load)
#   arm64  — build the image for linux/arm64 (Raspberry Pi, Apple Silicon, etc.)
#   push   — build multi-platform manifest (amd64 + arm64) via buildx and push
#   run    — run the image locally (set env vars below)
#
# Environment variables (build):
#   IMAGE      Docker image name/tag   (default: madpsy/ubersdr-ntp:latest)
#   PLATFORM   Docker --platform flag  (default: linux/amd64)
#   BUILDER    buildx builder name     (default: ubersdr_ntp_builder)
#   VERSION    version baked into the binary (default: git describe)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

IMAGE="${IMAGE:-madpsy/ubersdr-ntp:latest}"
PLATFORM="${PLATFORM:-linux/amd64}"
BUILDER="${BUILDER:-ubersdr_ntp_builder}"
# The build context has no .git, so the version is worked out here and passed in.
VERSION="${VERSION:-$(git -C "$SCRIPT_DIR" describe --tags --dirty --always 2>/dev/null || echo 0.0.0-unknown)}"
VERSION="${VERSION#v}"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

die() { echo "error: $*" >&2; exit 1; }

check_deps() {
    command -v docker >/dev/null || die "docker not found in PATH"
}

# Ensure a buildx builder that supports multi-platform builds exists.
# Uses the existing builder if already present; creates one otherwise.
ensure_builder() {
    if ! docker buildx inspect "$BUILDER" &>/dev/null; then
        echo "Creating buildx builder '$BUILDER'..."
        docker buildx create --name "$BUILDER" --driver docker-container --bootstrap
    else
        echo "Using existing buildx builder '$BUILDER'."
    fi
}

stage_context() {
    TMPCTX="$(mktemp -d)"
    # shellcheck disable=SC2064
    trap 'rm -rf "$TMPCTX"' EXIT

    echo "Staging build context in $TMPCTX..."
    # Build trees and binaries are large and host-specific; config.json holds
    # receiver passwords and has no business in an image.
    rsync -a --exclude='.git' \
              --exclude='build' \
              --exclude='build-*' \
              --exclude='ubersdr-ntp_*' \
              --exclude='config.json' \
              --exclude='__pycache__' \
              "$SCRIPT_DIR/" "$TMPCTX/"
}

build() {
    check_deps
    stage_context

    echo "Building image $IMAGE (platform=$PLATFORM, version=$VERSION)..."
    docker build \
        --platform "$PLATFORM" \
        --build-arg "VERSION=$VERSION" \
        --tag "$IMAGE" \
        "$TMPCTX"

    echo "Built: $IMAGE"
}

push() {
    check_deps
    ensure_builder
    stage_context

    local platforms="linux/amd64,linux/arm64"
    echo "Building and pushing multi-platform image $IMAGE (platforms=$platforms, version=$VERSION)..."
    docker buildx build \
        --builder "$BUILDER" \
        --platform "$platforms" \
        --build-arg "VERSION=$VERSION" \
        --tag "$IMAGE" \
        --push \
        "$TMPCTX"

    echo "Pushed multi-platform manifest: $IMAGE"
    # Push whatever is already committed — but never commit on the user's
    # behalf. This previously ran "git add -A" and committed everything with
    # a generic "Release" message, which silently swallowed real commit
    # messages and would sweep any unrelated work in progress (or a stray
    # credentials file) into a public push with no chance to review it.
    if [[ -n "$(git status --porcelain)" ]]; then
        echo
        echo "WARNING: uncommitted changes — the image was built from them," >&2
        echo "         but they are NOT being committed or pushed:" >&2
        git status --short >&2
        echo >&2
        echo "         Commit them yourself, then run: git push" >&2
        exit 1
    fi

    echo "Pushing git repository..."
    git push
}

run_image() {
    local args=()

    [[ -n "${WEB_PORT:-}"  ]] && args+=(-e "WEB_PORT=$WEB_PORT")
    [[ -n "${LOG_LEVEL:-}" ]] && args+=(-e "LOG_LEVEL=$LOG_LEVEL")
    # A local configuration, if there is one; otherwise the image's default.
    [[ -f "$SCRIPT_DIR/config.json" ]] && args+=(-v "$SCRIPT_DIR/config.json:/config/config.json:ro")

    # The status page only. NTP (123/udp) is not published.
    docker run --rm -it \
        --platform "$PLATFORM" \
        -p "${WEB_PORT:-6099}:${WEB_PORT:-6099}" \
        "${args[@]}" \
        "$IMAGE" \
        "$@"
}

# ---------------------------------------------------------------------------
# Environment variable reference (for docker run -e ...)
# ---------------------------------------------------------------------------
#
#   WEB_PORT            Status page / JSON API port (default: 6099)
#   LOG_LEVEL           trace, debug, info, warn, error (default: info)
#   UBERSDR_INGEST_URL  UberSDR's addon MQTT ingest port (default: http://ubersdr:6926)
#
# Everything else -- receivers, frequencies, NTP servers -- is config.json,
# mounted at /config/config.json.

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

case "${1:-build}" in
    build) build ;;
    arm64) PLATFORM=linux/arm64 build ;;
    push)  push  ;;
    run)   shift; run_image "$@" ;;
    *)
        echo "Usage: $0 [build|arm64|push|run [args...]]" >&2
        exit 1
        ;;
esac
