cmake_minimum_required(VERSION 3.16)
project(ubersdr-ntp CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(CURL REQUIRED IMPORTED_TARGET libcurl)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# IXWebSocket, built from the vendored tree.
#
# The two compile definitions are not optional and their absence is not a build
# error. IXWebSocket compiles its TLS support out unless they are defined --
# the OpenSSL backend is one large #ifdef, and IXSocketFactory consults
# IXWEBSOCKET_USE_TLS before it will create a secure socket at all. Upstream
# sets them from its own option(USE_TLS), which this build never runs because
# it globs the sources rather than add_subdirectory'ing the project. Linking
# OpenSSL is not enough on its own: every wss:// connection fails at runtime
# with "TLS support is not enabled on this platform", and public and tunnelled
# UberSDR receivers are all reached over wss://.
# ---------------------------------------------------------------------------
file(GLOB IXWS_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/third_party/IXWebSocket/ixwebsocket/*.cpp")
add_library(ixwebsocket_vendored STATIC ${IXWS_SOURCES})
target_include_directories(ixwebsocket_vendored PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/IXWebSocket")
target_link_libraries(ixwebsocket_vendored PUBLIC OpenSSL::SSL OpenSSL::Crypto Threads::Threads)
target_compile_definitions(ixwebsocket_vendored PUBLIC
    IXWEBSOCKET_USE_TLS
    IXWEBSOCKET_USE_OPEN_SSL)
# Upstream code, not ours: warnings here are noise that hides warnings in src/.
target_compile_options(ixwebsocket_vendored PRIVATE -w)

# ---------------------------------------------------------------------------
# The time-code decoders.
#
# Taken from ubersdr-clock and since changed
# here: sub-sample edge timing, the WWV/WWVH station tag, leap-second minutes
# and the WWVB DST bits. An upstream fix is no longer a straight `cp` -- diff
# first. The voter's calibration constants, and WwvDecoder's
# kNominalDelaySamples, are still upstream's and should not drift without
# evidence behind the change. tools/decodertest.cpp is that evidence offline.
# Built as their own target with warnings off, as upstream's code was.
# ---------------------------------------------------------------------------
add_library(clockdecoders STATIC
    src/clock/WwvDecoder.cpp
    src/clock/WwvbDecoder.cpp
    src/clock/Dcf77Decoder.cpp
    src/clock/MsfDecoder.cpp
    src/clock/AllouisDecoder.cpp
    src/clock/TimeFrameVoter.cpp)
target_include_directories(clockdecoders PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_compile_options(clockdecoders PRIVATE -w)

# Offline decoder accuracy test: synthesises WWV/WWVH/WWVB at several rates,
# sub-sample phases and noise levels (including a leap-second minute) and checks
# decoded timestamps and edge error. Needs no receiver; not installed.
add_executable(ubersdr-ntp-decodertest tools/decodertest.cpp)
target_link_libraries(ubersdr-ntp-decodertest PRIVATE clockdecoders Threads::Threads)
target_compile_options(ubersdr-ntp-decodertest PRIVATE -Wall -Wextra)

# Offline DCF77 test: synthesised AM+PM complex baseband (noise, carrier offset,
# AM-only, PM-only, inverted I/Q, leap second, CET/CEST change, AM and PM that
# disagree), then the real recording in tools/testdata if it is there. Not
# installed.
add_executable(ubersdr-ntp-dcf77test tools/dcf77test.cpp)
target_link_libraries(ubersdr-ntp-dcf77test PRIVATE clockdecoders Threads::Threads)
target_compile_options(ubersdr-ntp-dcf77test PRIVATE -Wall -Wextra)
target_compile_definitions(ubersdr-ntp-dcf77test PRIVATE
    DCF77_TESTDATA="${CMAKE_CURRENT_SOURCE_DIR}/tools/testdata/dcf77_live.wav")

# Offline MSF test: synthesised complex baseband (noise, sample-grid offsets,
# carrier offset, GMT/BST change, a leap-second minute, a missed marker, WWVB
# fed to it), then the M9PSY-1 recording in tools/testdata, checked against its
# capture stamps. Not installed.
add_executable(ubersdr-ntp-msftest tools/msftest.cpp)
target_link_libraries(ubersdr-ntp-msftest PRIVATE clockdecoders Threads::Threads)
target_compile_options(ubersdr-ntp-msftest PRIVATE -Wall -Wextra)
target_compile_definitions(ubersdr-ntp-msftest PRIVATE
    MSF_TESTDATA="${CMAKE_CURRENT_SOURCE_DIR}/tools/testdata/msf_m9psy1_20260925T0120Z.wav")

# Offline ALS162 (Allouis) test: synthesised phase modulation (noise,
# sample-grid offsets, carrier offset, inverted I/Q, CET/CEST change, a
# maintenance outage), then the M9PSY-1 recording in tools/testdata, checked
# against its capture stamps. Not installed.
add_executable(ubersdr-ntp-allouistest tools/allouistest.cpp)
target_link_libraries(ubersdr-ntp-allouistest PRIVATE clockdecoders Threads::Threads)
target_compile_options(ubersdr-ntp-allouistest PRIVATE -Wall -Wextra)
target_compile_definitions(ubersdr-ntp-allouistest PRIVATE
    ALS162_TESTDATA="${CMAKE_CURRENT_SOURCE_DIR}/tools/testdata/als162_m9psy1_20260925T0120Z.wav")

# Records lossless IQ from an UberSDR receiver, tuned on a carrier, with the
# receiver's capture timestamp for every packet in a CSV beside the WAV. For
# making test recordings (tools/testdata). Not installed.
add_executable(ubersdr-ntp-iqrecord tools/iqrecord.cpp)
target_link_libraries(ubersdr-ntp-iqrecord PRIVATE ixwebsocket_vendored PkgConfig::CURL Threads::Threads)
target_compile_options(ubersdr-ntp-iqrecord PRIVATE -Wall -Wextra)

# Offline test of the daemon's clock: the offset/rate estimator against a
# drifting crystal, a step and outliers; the sample-clock fit on a steady clock
# against one being slewed the way a host's NTP client slews it; and the
# selector extrapolating and coasting along the measured rate. Not installed.
add_executable(ubersdr-ntp-clocktest
    tools/clocktest.cpp
    src/TimeContinuity.cpp
    src/Config.cpp
    src/OffsetEstimator.cpp
    src/SampleClock.cpp
    src/Selector.cpp
    src/Log.cpp)
target_include_directories(ubersdr-ntp-clocktest PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(ubersdr-ntp-clocktest PRIVATE Threads::Threads)
target_compile_options(ubersdr-ntp-clocktest PRIVATE -Wall -Wextra)

# Offline test of the upstream NTP client: an NtpPeer against a fake NTP server
# running in the same process on the loopback, whose clock is wrong by a known
# amount. Checks that the offset is recovered, that the minimum-delay filter
# rejects queued replies, that a forged reply, an unsynchronised server, an
# excessive stratum or root distance and a synchronisation loop are all refused
# with a reason, that kiss-o'-death is honoured, and that cold standby really
# disconnects. Needs no network beyond the loopback; not installed.
add_executable(ubersdr-ntp-ntptest
    tools/ntptest.cpp
    src/NtpClient.cpp
    src/Config.cpp
    src/OffsetEstimator.cpp
    src/SampleClock.cpp
    src/Log.cpp)
target_include_directories(ubersdr-ntp-ntptest PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(ubersdr-ntp-ntptest PRIVATE Threads::Threads resolv)
target_compile_options(ubersdr-ntp-ntptest PRIVATE -Wall -Wextra)

# Offline test of the configuration reader: per-source passwords and other
# per-source settings, the upstream NTP server spellings, the clock block, and
# the validation that refuses a configuration which cannot work. Writes its
# cases to a temporary directory and reads them back through the real loader.
# Not installed.
add_executable(ubersdr-ntp-configtest
    tools/configtest.cpp
    src/Config.cpp
    src/Log.cpp)
target_include_directories(ubersdr-ntp-configtest PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_compile_options(ubersdr-ntp-configtest PRIVATE -Wall -Wextra)

# Offline test of the event monitor: scripted sequences of snapshots and
# combine results, checking which events come out and, as much, which do not.
# Not installed.
add_executable(ubersdr-ntp-eventtest
    tools/eventtest.cpp
    src/Events.cpp
    src/Metrics.cpp
    src/Config.cpp
    src/SampleClock.cpp
    src/Selector.cpp
    src/Log.cpp)
target_include_directories(ubersdr-ntp-eventtest PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(ubersdr-ntp-eventtest PRIVATE Threads::Threads)
target_compile_options(ubersdr-ntp-eventtest PRIVATE -Wall -Wextra)

# ---------------------------------------------------------------------------
# The page is a real file in src/web/ and is served from the binary. This is
# the one line of build glue that costs; a 400-line page living as a string
# literal is a page nobody would edit.
# ---------------------------------------------------------------------------
find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_custom_command(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/IndexHtml.inc"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/generated"
    COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/tools/embed-html.py"
            "${CMAKE_CURRENT_SOURCE_DIR}/src/web/index.html"
            "${CMAKE_CURRENT_BINARY_DIR}/generated/IndexHtml.inc"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/web/index.html"
            "${CMAKE_CURRENT_SOURCE_DIR}/src/web/flow.js"
            "${CMAKE_CURRENT_SOURCE_DIR}/tools/embed-html.py"
    COMMENT "Embedding src/web/index.html"
    VERBATIM)
add_custom_target(index_html DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/generated/IndexHtml.inc")

# The version header, generated on every build from src/Version.h.in: the
# release tag when build.sh passes one, `git describe` otherwise. See
# tools/version.cmake. Rewritten only when the version changes.
set(UBERSDR_NTP_VERSION "" CACHE STRING
    "Version baked into the binary; empty takes it from git describe")
add_custom_target(version_header
    COMMAND ${CMAKE_COMMAND}
            "-DSRC=${CMAKE_CURRENT_SOURCE_DIR}"
            "-DOUT=${CMAKE_CURRENT_BINARY_DIR}/generated/Version.h"
            "-DVERSION=${UBERSDR_NTP_VERSION}"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/tools/version.cmake"
    BYPRODUCTS "${CMAKE_CURRENT_BINARY_DIR}/generated/Version.h"
    COMMENT "Version header"
    VERBATIM)

add_executable(ubersdr-ntp
    src/main.cpp
    src/Config.cpp
    src/Events.cpp
    src/HttpApi.cpp
    src/Log.cpp
    src/Metrics.cpp
    src/Mqtt.cpp
    src/NtpClient.cpp
    src/NtpServer.cpp
    src/OffsetEstimator.cpp
    src/Propagation.cpp
    src/SampleClock.cpp
    src/Selector.cpp
    src/Source.cpp
    src/TimeContinuity.cpp
    src/Status.cpp
    "${CMAKE_CURRENT_BINARY_DIR}/generated/IndexHtml.inc")
add_dependencies(ubersdr-ntp index_html version_header)
add_dependencies(ubersdr-ntp-clocktest version_header)
add_dependencies(ubersdr-ntp-configtest version_header)
add_dependencies(ubersdr-ntp-eventtest version_header)
add_dependencies(ubersdr-ntp-ntptest version_header)   # Selector.h reaches Config.h, which needs it

target_include_directories(ubersdr-ntp PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(ubersdr-ntp PRIVATE
    clockdecoders
    ixwebsocket_vendored
    PkgConfig::CURL
    Threads::Threads
    # res_nsearch and ns_parserr, for the DNS TTL of an upstream NTP server's
    # name (NtpClient.cpp). In libc itself from glibc 2.34; the library is
    # still there, empty, for what links against it.
    resolv)
target_compile_options(ubersdr-ntp PRIVATE -Wall -Wextra)

# libstdc++ and libgcc statically, for the same reason ubersdr-clock builds in
# a container: a binary linked against a newer libstdc++ than the host it runs
# on dies at startup on a GLIBCXX_ version error that names everything except
# the actual problem. Linking them in removes that coupling outright, and costs
# about a megabyte. libcurl and libssl remain dynamic -- they are
# ordinary distribution packages with stable ABIs, and statically linking
# OpenSSL would mean shipping our own view of the CA store and of every TLS
# advisory.
target_link_options(ubersdr-ntp PRIVATE -static-libstdc++ -static-libgcc)

# The binary is named after the target architecture using Go's GOARCH
# spellings, matching how ubersdr-clock and the other UberSDR decoder binaries
# are installed, so several architectures can share one directory.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
    set(UBERSDR_ARCH amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(UBERSDR_ARCH arm64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(armv7l|armv7)$")
    set(UBERSDR_ARCH armv7
)
else()
    set(UBERSDR_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
endif()

# The arch-named copy lands in the BUILD directory, not the source tree.
#
# It used to be copied to the source directory, and that was wrong twice over:
# a build writing into its own source tree is a surprise, and under the
# container build in build.sh -- which bind-mounts the repo and runs as root --
# it left a root-owned binary at the repo root that the next non-root build
# could not overwrite. build.sh copies it out itself, as the owning user.
add_custom_command(TARGET ubersdr-ntp POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
            "$<TARGET_FILE:ubersdr-ntp>"
            "$<TARGET_FILE_DIR:ubersdr-ntp>/ubersdr-ntp_${UBERSDR_ARCH}"
    COMMENT "-> ubersdr-ntp_${UBERSDR_ARCH}")

include(GNUInstallDirs)
install(TARGETS ubersdr-ntp RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
