#pragma once

// Calendar arithmetic for turning a decoded WWV frame into an instant.
//
// Howard Hinnant's civil-date algorithms, as used by ubersdr-clock's own front
// end. Deliberately not timegm()/gmtime_r(): both are POSIX rather than ISO C,
// and whether they are visible under -std=c++20 depends on feature-test macros.
// A time daemon whose timestamps depend on how the compiler was invoked is not
// one to debug at 3am.

#include "clock/TimeFrameVoter.h"

#include <cstdint>
#include <cstdio>
#include <string>

namespace ubersdr_ntp {

inline long long daysFromCivil(int y, unsigned m, unsigned d) {
    y -= m <= 2;
    const long long era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned doy = (153u * (m + (m > 2 ? -3u : 9u)) + 2u) / 5u + d - 1u;
    const unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
    return era * 146097LL + static_cast<long long>(doe) - 719468LL;
}

inline void civilFromDays(long long z, int& y, unsigned& m, unsigned& d) {
    z += 719468;
    const long long era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = static_cast<unsigned>(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460u + doe / 36524u - doe / 146096u) / 365u;
    const long long yy = static_cast<long long>(yoe) + era * 400;
    const unsigned doy = doe - (365u * yoe + yoe / 4u - yoe / 100u);
    const unsigned mp = (5u * doy + 2u) / 153u;
    d = doy - (153u * mp + 2u) / 5u + 1u;
    m = mp + (mp < 10u ? 3u : -9u);
    y = static_cast<int>(yy + (m <= 2u));
}

// Floor division: plain / and % truncate toward zero, which is wrong for
// pre-1970 instants. Not reachable from a valid decode, but the fallback paths
// are handed whatever the voter produced.
inline long long floorDiv(long long a, long long b) {
    const long long q = a / b;
    return (a % b != 0 && ((a < 0) != (b < 0))) ? q - 1 : q;
}
inline long long floorMod(long long a, long long b) { return a - floorDiv(a, b) * b; }

// Unix milliseconds for a decoded frame. doy is the 1-based NIST BCD day
// field, year2 the two-digit year in the 20xx century, seconds always zero —
// a time-code frame names the minute, and the second comes from the edge.
inline long long utcMsFromFields(int year2, int doy, int hour, int minute) {
    const long long days = daysFromCivil(2000 + year2, 1, 1) + (doy - 1);
    return (days * 86400LL + hour * 3600LL + minute * 60LL) * 1000LL;
}

inline std::string iso8601(long long ms) {
    const long long secs = floorDiv(ms, 1000);
    const long long days = floorDiv(secs, 86400);
    long long rem = floorMod(secs, 86400);
    int y = 0; unsigned mo = 0, d = 0;
    civilFromDays(days, y, mo, d);
    const int hh = static_cast<int>(rem / 3600); rem %= 3600;
    const int mi = static_cast<int>(rem / 60);
    const int ss = static_cast<int>(rem % 60);
    char buf[48];
    std::snprintf(buf, sizeof buf, "%04d-%02u-%02uT%02d:%02d:%02dZ", y, mo, d, hh, mi, ss);
    return buf;
}

// The host clock in the form the voter's plausibility gate wants. Called from
// inside process() on the audio thread, so it stays four integer divisions.
inline clockdec::TimeFields hostNowFields(long long hostMs) {
    const long long secs = floorDiv(hostMs, 1000);
    const long long days = floorDiv(secs, 86400);
    const long long rem = floorMod(secs, 86400);
    int y = 0; unsigned mo = 0, d = 0;
    civilFromDays(days, y, mo, d);
    clockdec::TimeFields tf;
    tf.minute = static_cast<int>((rem / 60) % 60);
    tf.hour = static_cast<int>(rem / 3600);
    tf.doy = static_cast<int>(days - daysFromCivil(y, 1, 1)) + 1;
    tf.year2 = y % 100;
    return tf;
}

// Whether a Unix-ms instant falls in the last day of a month, which is the
// only time a broadcast leap-second warning can be acted on. WWV asserts the
// warning throughout the month leading up to a leap second; NTP's leap
// indicator means "in the last minute of TODAY", so the two are not the same
// bit and the warning must be narrowed before it is passed on.
inline bool isLastDayOfMonth(long long ms) {
    const long long days = floorDiv(floorDiv(ms, 1000), 86400);
    int y1 = 0, y2 = 0; unsigned m1 = 0, m2 = 0, d1 = 0, d2 = 0;
    civilFromDays(days, y1, m1, d1);
    civilFromDays(days + 1, y2, m2, d2);
    return m1 != m2;
}

} // namespace ubersdr_ntp
