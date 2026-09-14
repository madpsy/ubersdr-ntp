# Writes the version header from src/Version.h.in. Run at BUILD time, not
# configure time, so a development build reports the commit it was actually
# built from rather than the one it happened to be configured at.
#
#   cmake -DSRC=<source dir> -DOUT=<header> [-DVERSION=<version>] -P version.cmake
#
# VERSION, when given, wins: build.sh passes the release tag, and it must,
# because the container build has no git and would otherwise have nothing to
# describe. Without it, `git describe --tags --dirty --always`. A leading "v" is
# dropped either way, so tag v0.1.9 reports 0.1.9 and the page's own "v" prefix
# does not double it.
#
# configure_file only writes when the content changes, so an unchanged version
# rebuilds nothing.

if(NOT VERSION)
    find_package(Git QUIET)
    if(GIT_FOUND)
        execute_process(
            COMMAND "${GIT_EXECUTABLE}" -C "${SRC}" describe --tags --dirty --always
            OUTPUT_VARIABLE VERSION
            OUTPUT_STRIP_TRAILING_WHITESPACE
            RESULT_VARIABLE rc
            ERROR_QUIET)
        if(NOT rc EQUAL 0)
            set(VERSION "")
        endif()
    endif()
endif()
if(NOT VERSION)
    set(VERSION "0.0.0-unknown")
endif()
string(REGEX REPLACE "^v" "" VERSION "${VERSION}")

set(UBERSDR_NTP_VERSION "${VERSION}")
configure_file("${SRC}/src/Version.h.in" "${OUT}" @ONLY)
