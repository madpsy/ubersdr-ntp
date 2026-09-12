#!/usr/bin/env bash
#
# Build ubersdr-ntp for amd64 and arm64, and smoke-test each one.
#
# Built inside ubuntu:24.04 -- the same image UberSDR's container runtime stage
# uses -- so the result runs wherever the other UberSDR binaries run. Building
# on the host works right up until the host is newer than the target: libopus,
# libcurl and libssl are ordinary shared libraries, and a binary linked against
# a newer one fails at startup on a symbol version error that names everything
# except the actual problem. (libstdc++ is linked statically for exactly that
# reason, so it is not one of them.)
#
# arm64 is built by running an arm64 ubuntu:24.04 under binfmt/qemu rather than
# cross-compiling, so the toolchain is the target toolchain and CMake sees the
# target arch. Slow, and correct without a sysroot to keep in step.
#
# Building is the easy half. A binary that links and runs can still answer NTP
# wrongly, so unless told otherwise each one is run against tools/selftest.py,
# which drives its real NTP socket and its real HTTP service -- see that file
# for what it does and does not cover.
#
# Usage:
#   ./build.sh [options]
#
#   --arch LIST     comma-separated: amd64, arm64 (default: both)
#   --native        build on this host with the host toolchain instead of in a
#                   container. This host arch only; for a quick edit-compile
#                   loop, not for anything you intend to ship
#   --clean         delete the build trees first
#   --no-check      build only, skip the smoke test
#   --image IMAGE   build container image (default: ubuntu:24.04)
#   -j N            parallel jobs (default: all cores)
#

set -euo pipefail

repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
arches="amd64 arm64"
image=ubuntu:24.04
native=0
clean=0
check=1
jobs=$(nproc 2>/dev/null || echo 4)

while [ $# -gt 0 ]; do
    case "$1" in
        --arch)     arches=$(echo "$2" | tr ',' ' '); shift 2 ;;
        --native)   native=1; shift ;;
        --clean)    clean=1; shift ;;
        --no-check) check=0; shift ;;
        --image)    image=$2; shift 2 ;;
        -j)         jobs=$2; shift 2 ;;
        -j*)        jobs=${1#-j}; shift ;;
        -h|--help)  sed -n '2,/^$/p' "${BASH_SOURCE[0]}" | sed 's/^#\{0,1\} \{0,1\}//'; exit 0 ;;
        *)          echo "build.sh: unknown option $1" >&2; exit 2 ;;
    esac
done

say()  { printf '\n== %s\n' "$*"; }
fail() { printf '\nbuild.sh: %s\n' "$*" >&2; exit 1; }

for a in $arches; do
    case "$a" in
        amd64|arm64|arm|386) ;;
        *) fail "unknown arch '$a' (expected amd64, arm64, arm or 386)" ;;
    esac
done

# The packages this needs to BUILD, and the ones it needs to RUN. Kept together
# because a container image that installs only the first set produces a binary
# that will not start, and the error it gives does not say which package is
# missing.
build_pkgs="cmake ninja-build g++ python3 pkg-config libopus-dev libcurl4-openssl-dev libssl-dev"
runtime_pkgs="libopus0 libcurl4 libssl3"

# --- The work done inside each container ---------------------------------
#
# Fed to bash on stdin rather than passed as `bash -c '...'`: an apostrophe
# anywhere in here -- including in a comment -- would close the quote and
# silently truncate the rest of the script, and the build would still exit 0.
#
# $1 = arch label, $2 = jobs, $3 = run the check, $4:$5 = host uid:gid,
# $6 = build packages
container_script=$(cat <<'CONTAINER_EOF'
set -euo pipefail
arch=$1; jobs=$2; check=$3; uid=$4; gid=$5; pkgs=$6

export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
# shellcheck disable=SC2086
apt-get install -y -qq --no-install-recommends $pkgs >/dev/null

build=/src/build-$arch

# The container runs as root over a bind-mounted repo, so everything it writes
# comes out root-owned and the next non-root build cannot delete it. Handing it
# back happens in a TRAP rather than at the end of the script: a build that
# fails, or a container that is killed part-way -- which is what happens when
# the caller interrupts a slow qemu arm64 build -- would otherwise leave a
# root-owned tree behind that takes another privileged container to remove.
trap "chown -R $uid:$gid \"$build\" 2>/dev/null || true" EXIT INT TERM

cmake -S /src -B "$build" -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$build" -j "$jobs"

binary="$build/ubersdr-ntp"
if [ ! -x "$binary" ]; then
    echo "no binary was produced" >&2
    exit 1
fi

# Name it after the target architecture using Go's GOARCH spellings, matching
# how ubersdr-clock and the other UberSDR binaries are installed so several
# architectures can share one directory. CMake does this too, from
# CMAKE_SYSTEM_PROCESSOR; done again here so the name is right even if CMake
# and the container disagree about what they are running on.
#
# Inside the build directory only. This container runs as root over a
# bind-mounted repo, so anything written to /src itself comes out root-owned,
# and the chown at the end of this script covers $build and nothing else.
cp "$binary" "$build/ubersdr-ntp_$arch"

if [ "$check" = 1 ]; then
    # The smoke test binds real sockets on high ports and drives them. It needs
    # nothing but the Python standard library.
    python3 /src/tools/selftest.py "$build/ubersdr-ntp_$arch" 2>&1 | sed 's/^/CHECK /'
fi

# The trap above covers this on every exit path, including a kill. Left here as
# well so the ownership is correct before the caller's `cp` runs rather than
# racing the trap.
chown -R "$uid:$gid" "$build"
CONTAINER_EOF
)

if [ "$clean" = 1 ]; then
    say "Removing build trees"
    rm -rf "$repo"/build-* "$repo"/build "$repo"/ubersdr-ntp_*
fi

built=""

if [ "$native" = 1 ]; then
    host_arch=$(dpkg --print-architecture 2>/dev/null || echo amd64)
    say "Building natively for $host_arch (host toolchain)"
    cmake -S "$repo" -B "$repo/build" -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
    cmake --build "$repo/build" -j "$jobs"
    [ -x "$repo/build/ubersdr-ntp" ] || fail "the build produced no binary"
    built_name=$(basename "$(ls -t "$repo"/build/ubersdr-ntp_* 2>/dev/null | head -1)")
    [ -n "$built_name" ] || built_name=ubersdr-ntp
    cp "$repo/build/$built_name" "$repo/"
    binary="$repo/$built_name"

    if [ "$check" = 1 ]; then
        say "Smoke-testing $(basename "$binary")"
        python3 "$repo/tools/selftest.py" "$binary" | sed 's/^/  /'
    fi

    say "Done"
    printf '  %s\n' "$binary"
    echo
    echo "Built with this host's toolchain -- fine for testing here, not for the"
    echo "container. Drop --native for anything you intend to ship."
    exit 0
fi

command -v docker >/dev/null 2>&1 \
    || fail "docker is needed to build for both architectures; use --native to
build only for this host"

# arm64 on an amd64 host needs the qemu binfmt handler, or docker starts the
# container and every process in it dies with "exec format error".
#
# THE REGISTRATION DOES NOT SURVIVE A REBOOT. binfmt_misc is a kernel mount and
# tonistiigi/binfmt writes into it at runtime, so a machine that built arm64
# happily last week will fail on it today with no other change -- and the
# symptom, an exec format error from inside the container, names neither binfmt
# nor the reboot. Worth knowing before concluding the host cannot do it: the
# check below distinguishes "never set up" from "set up and lost", which are
# the same fix but not the same diagnosis.
for a in $arches; do
    [ "$a" = "$(dpkg --print-architecture 2>/dev/null || echo amd64)" ] && continue
    if ! ls /proc/sys/fs/binfmt_misc/ 2>/dev/null | grep -qi "qemu-aarch64\|qemu-arm"; then
        hint=""
        if ls "$repo"/build-arm64/ubersdr-ntp_* >/dev/null 2>&1 \
           || ls "$repo"/ubersdr-ntp_arm64 >/dev/null 2>&1; then
            hint="
This host HAS built $a before -- there is an arm64 binary in the tree -- so the
registration was simply lost, almost certainly to a reboot. The command above
puts it back."
        fi
        fail "no qemu binfmt handler registered, so a $a container cannot run here.
Register it with:
  docker run --privileged --rm tonistiigi/binfmt --install all

It is a runtime registration and is lost on every reboot, so this is a thing
that recurs rather than a thing you set up once.$hint"
    fi
    break
done

failed=0
for arch in $arches; do
    say "Building $arch in $image"
    # The pipeline must not swallow a container failure, and `set -o pipefail`
    # with a `while read` on the right would report the reader's status. So the
    # container's exit status is captured explicitly.
    status_file=$(mktemp)
    {
        printf %s "$container_script" | docker run --rm -i \
            --platform "linux/$arch" \
            -v "$repo:/src" \
            "$image" \
            bash -s -- "$arch" "$jobs" "$check" "$(id -u)" "$(id -g)" "$build_pkgs" \
            2>&1 || echo "DOCKER_FAILED $?" >"$status_file"
    } | while IFS= read -r line; do
            case "$line" in
                "CHECK "*) echo "  ${line#CHECK }" ;;
                *)         echo "  $line" ;;
            esac
        done

    if [ -s "$status_file" ]; then
        rm -f "$status_file"
        echo "  build or check FAILED for $arch" >&2
        failed=1
        continue
    fi
    rm -f "$status_file"

    binary="$repo/build-$arch/ubersdr-ntp_$arch"
    [ -x "$binary" ] || { echo "  $arch: the build produced no binary" >&2; failed=1; continue; }
    cp "$binary" "$repo/"
    built="$built $repo/ubersdr-ntp_$arch"
done

[ "$failed" = 0 ] || fail "at least one architecture did not build or did not pass its check"

say "Done"
for b in $built; do
    printf '  %s -- %s\n' "$(basename "$b")" "$(file -b "$b" | cut -d, -f1-2)"
done

cat <<EOF

To run the binary on a target host, install its runtime libraries:
  sudo apt-get install -y $runtime_pkgs

Port 123 is privileged. Either run as root, or grant just that capability:
  sudo setcap 'cap_net_bind_service=+ep' /usr/local/bin/ubersdr-ntp
EOF
