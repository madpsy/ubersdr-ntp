#pragma once

// The version, and the identity this daemon presents to a receiver.
//
// One place for both, because they appear in four: --version, the log banner,
// the status JSON, and the User-Agent on every HTTP and WebSocket request. A
// receiver operator looking at their session list sees the User-Agent, and it
// is the only thing telling them what has taken one of their slots -- so it
// names the program, the version, and where to find out what it is. That is
// also what UberSDR's own clients do (UberSDR-Benchmark/1.0, ubersdr_navtex/2.0),
// and a receiver that bans by User-Agent can then ban or allow this
// specifically rather than having to guess.

namespace ubersdr_ntp {

inline constexpr const char* kVersion = "1.0.0";

inline constexpr const char* kUserAgent =
    "ubersdr-ntp/1.0.0 (+https://github.com/madpsy/ubersdr-ntp)";

} // namespace ubersdr_ntp
