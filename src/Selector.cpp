#include "Selector.h"

#include "Log.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <set>

namespace ubersdr_ntp {

namespace {

// A source's measurements must be no older than this to be a candidate. Three
// minutes is three missed WWV frames: long enough to ride out a fade that the
// decoder itself holds a lock through, short enough that a dead source is not
// still voting ten minutes later.
constexpr double kCandidateMaxAgeSec = 180.0;

// The time constant of a source's running disagreement with the consensus.
// Five minutes: long enough that a fade, or one badly placed second edge, is
// averaged away, short enough that a receiver that has genuinely gone wrong is
// caught before it has served much.
constexpr double kResidualTauSec = 300.0;

// And how long a source must have been measured before that average is allowed
// to convict it. A source that has just become a candidate has an average made
// of one reading, and the reading it starts on is the one taken while its
// offsets were still settling -- which is exactly when it looks worst. Live,
// that refused a good receiver six times in the seconds after a third source
// joined, on a transient that was gone by the next minute.
constexpr double kResidualSettleSec = 120.0;

constexpr const char* kTag = "selector";

// A source must be judged against at least this many others before its
// disagreement means anything. Two sources produce equal and opposite
// residuals whichever of them is wrong, so one peer is not enough to convict.
constexpr int kMinPeersToJudge = 2;

// How far a source may sit from the consensus before it is refused outright.
//
// Everything the sources genuinely do not share is bounded and small: the
// propagation difference between two receivers on this continent is under
// 20 ms, and the largest real disagreement available -- one receiver hearing
// WWVH in Hawaii while the others hear WWV in Colorado -- is about 19 ms, and
// is modelled anyway from the decoder's own station tag. Past 30 ms there is
// no physical arrangement of receivers that explains it: they are listening to
// one transmitter and one second edge. So a source out there is not a source
// with a bad delay model, it is a source that has decoded something else, and
// it is refused rather than averaged in. Measured live, a receiver whose edge
// tracker had locked about 100 ms late sat at -103 ms while the other two
// agreed to half a millisecond.
constexpr double kMaxDisagreementSec = 0.030;

// NTP reference identifiers for a stratum-1 server, as registered in RFC 5905.
// Four ASCII characters, left-justified and null-padded.
std::string refidFor(const std::string& station) {
    if (station == "wwvh") return "WWVH";
    if (station == "wwvb") return "WWVB";
    return "WWV";
}

struct Candidate {
    const SourceSnapshot* s;
    double offset;
    double dist;     // half-width of the asserted interval
    double quality;  // the uncertainty that distinguishes it from the others
    double weight;
};

std::string format(const char* fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    return buf;
}

} // namespace

Selector::Selector(double coastSeconds, double coastDriftPpm, int minSources)
    : m_coastSeconds(coastSeconds),
      m_coastDriftPpm(coastDriftPpm),
      m_minSources(minSources) {}

Combined Selector::combine(const std::vector<SourceSnapshot>& snaps, double nowRealtime) {
    Combined c;

    std::vector<Candidate> cand;
    for (const SourceSnapshot& s : snaps) {
        if (!s.enabled) { c.notUsedReasons[s.name] = "disabled in the configuration"; continue; }
        // Most specific first, so the reason names the step that is actually
        // missing rather than a symptom of it: an unlocked decoder also has no
        // offset, and "no offset" would send someone to the wrong place.
        std::string why;
        if (s.clockState != "locked") {
            why = "decoder not locked yet";
        } else if (s.offsetAgeSec > kCandidateMaxAgeSec) {
            why = format("newest measurement is %.0f s old (limit %.0f s)",
                         s.offsetAgeSec, kCandidateMaxAgeSec);
        } else if (!s.haveOffset) {
            why = format("still filtering: %d offset sample(s) so far", s.offsetSamples);
        } else if (s.dispersionSec <= 0.0) {
            why = "no dispersion estimate yet";
        }
        if (!why.empty()) {
            c.rejectedNames.push_back(s.name);
            c.notUsedReasons[s.name] = why;
            continue;
        }

        // The interval a source asserts. Its dispersion plus the age of its
        // newest measurement times the coast rate: a source whose last reading
        // is a minute old is a minute less certain than it was.
        const double staleness = s.offsetAgeSec * (m_coastDriftPpm * 1e-6);
        const double quality = (s.weightDispersionSec > 0.0 ? s.weightDispersionSec
                                                            : s.dispersionSec) + staleness;
        cand.push_back(Candidate{&s, s.offsetSec, s.dispersionSec + staleness, quality,
                                 s.weight > 0.0 ? s.weight : 1.0});
    }
    c.candidates = static_cast<int>(cand.size());

    // --- agreement ----------------------------------------------------------
    //
    // Against the median of ALL of them, self included, and not against each
    // source's peers alone. Leave-one-out is the obvious construction and it is
    // wrong here: with three sources each one has two peers, the median of two
    // is their mean, and one source that is badly wrong therefore drags the
    // figure every OTHER source is judged against. Measured live, a receiver
    // 103 ms out made the two good ones read 36 ms out. The median of three
    // survives one outlier, which is the whole reason to prefer a median, and
    // the source that is actually wrong is the one left standing apart.
    //
    // With two sources the median is their mean and neither can be told from
    // the other: the residuals come out as equal and opposite whatever the
    // truth is. That is not a defect to be worked around, it is what two
    // measurements of one event can tell you. Hence kMinPeersToJudge.
    std::set<std::string> refused;
    std::vector<double> all;
    all.reserve(cand.size());
    for (const Candidate& k : cand) all.push_back(k.offset);
    std::sort(all.begin(), all.end());
    const std::size_t amid = all.size() / 2;
    const double consensus = all.empty() ? 0.0
                           : ((all.size() % 2) ? all[amid] : 0.5 * (all[amid - 1] + all[amid]));

    for (const Candidate& k : cand) {
        SourceResidual r;
        r.name = k.s->name;
        r.instantSec = k.offset - consensus;
        r.peers = static_cast<int>(cand.size()) - 1;

        auto it = m_residualAvg.find(r.name);
        if (it == m_residualAvg.end()) {
            ResidualState fresh;
            fresh.averagedSec = r.instantSec;
            fresh.lastAtSec = nowRealtime;
            fresh.firstAtSec = nowRealtime;
            it = m_residualAvg.emplace(r.name, fresh).first;
        } else {
            const double dt = nowRealtime - it->second.lastAtSec;
            if (dt > 0.0) {
                const double alpha = 1.0 - std::exp(-dt / kResidualTauSec);
                it->second.averagedSec += alpha * (r.instantSec - it->second.averagedSec);
                it->second.lastAtSec = nowRealtime;
            }
        }
        r.averagedSec = it->second.averagedSec;
        r.settledForSec = nowRealtime - it->second.firstAtSec;
        r.haveAverage = true;
        r.refused = r.peers >= kMinPeersToJudge &&
                    r.settledForSec >= kResidualSettleSec &&
                    std::abs(r.averagedSec) > kMaxDisagreementSec;
        if (r.refused) refused.insert(r.name);
        if (r.refused && !it->second.refusalLogged) {
            LOG_WARN(kTag, "%s disagrees with the other sources by %+.0f ms; "
                     "not a delay model this far out — refusing it",
                     r.name.c_str(), r.averagedSec * 1000.0);
        } else if (!r.refused && it->second.refusalLogged) {
            LOG_INFO(kTag, "%s agrees with the other sources again (%+.0f ms); using it",
                     r.name.c_str(), r.averagedSec * 1000.0);
        }
        it->second.refusalLogged = r.refused;
        c.residuals.push_back(std::move(r));
    }

    // Drop what the consensus refuses. This is deliberately NOT a correction:
    // a source that disagrees is telling you it decoded something else, and
    // learning an offset that makes it agree would turn a detected fault into
    // an undetectable one.
    if (!refused.empty()) {
        std::vector<Candidate> kept;
        for (const Candidate& k : cand) {
            if (refused.count(k.s->name)) {
                c.rejectedNames.push_back(k.s->name);
                c.notUsedReasons[k.s->name] =
                    format("refused: %+.0f ms from the other sources (limit ±%.0f ms)",
                           m_residualAvg[k.s->name].averagedSec * 1000.0,
                           kMaxDisagreementSec * 1000.0);
            } else {
                kept.push_back(k);
            }
        }
        cand.swap(kept);
        c.candidates = static_cast<int>(cand.size());
    }

    // --- Marzullo / Mills intersection -------------------------------------
    //
    // Sweep the interval endpoints in order, counting how many intervals are
    // open. The widest overlap is where that count peaks. Rather than search
    // for a falseticker limit by relaxing the requirement — which is what a
    // full NTP implementation does — this takes the largest agreeing set
    // outright, because the number of sources here is small enough that the
    // difference never arises.
    std::vector<Candidate> survivors;
    if (cand.size() == 1) {
        survivors = cand;
    } else if (cand.size() > 1) {
        struct Edge { double at; int delta; };
        std::vector<Edge> edges;
        edges.reserve(cand.size() * 2);
        for (const Candidate& k : cand) {
            edges.push_back({k.offset - k.dist, +1});
            edges.push_back({k.offset + k.dist, -1});
        }
        // Closing edges before opening ones at the same coordinate, so two
        // intervals that merely touch are not counted as overlapping.
        std::sort(edges.begin(), edges.end(), [](const Edge& a, const Edge& b) {
            return a.at != b.at ? a.at < b.at : a.delta < b.delta;
        });

        int open = 0, best = 0;
        double bestAt = 0.0;
        for (const Edge& e : edges) {
            open += e.delta;
            if (e.delta > 0 && open > best) { best = open; bestAt = e.at; }
        }

        for (const Candidate& k : cand) {
            if (bestAt >= k.offset - k.dist && bestAt <= k.offset + k.dist) survivors.push_back(k);
            else {
                c.rejectedNames.push_back(k.s->name);
                c.notUsedReasons[k.s->name] =
                    format("outside the majority: %+.1f ms ± %.1f ms does not overlap "
                           "the interval %d source(s) agree on",
                           k.offset * 1000.0, k.dist * 1000.0, best);
            }
        }
    }

    if (static_cast<int>(survivors.size()) >= m_minSources && !survivors.empty()) {
        // --- weighted combine ----------------------------------------------
        // Weighted by the part of each source's uncertainty that is its own,
        // not by the interval it asserts: the asserted interval carries the
        // delay model's doubt, which every source carries equally, and dividing
        // by a number they share tells the average nothing while hiding what
        // does not. A source that is measurably noisier, or whose sample clock
        // has just been rebuilt, loses its share here automatically -- which is
        // the whole point of the source reporting honestly.
        double sw = 0.0, swx = 0.0;
        for (const Candidate& k : survivors) {
            const double w = k.weight / (k.quality * k.quality);
            sw += w;
            swx += w * k.offset;
        }
        const double offset = sw > 0.0 ? swx / sw : survivors.front().offset;

        // Dispersion of the combination. Not the weighted mean of the parts:
        // survivors that disagree with each other must widen the answer, or two
        // sources 40 ms apart would produce a result claiming 10 ms. So it is
        // the weighted dispersion plus the spread of the survivors about the
        // combined value — the select dispersion, in NTP's terms.
        double swd = 0.0, spread = 0.0;
        for (const Candidate& k : survivors) {
            const double w = k.weight / (k.quality * k.quality);
            swd += w * k.dist;
            spread = std::max(spread, std::abs(k.offset - offset));
        }
        const double dispersion = (sw > 0.0 ? swd / sw : survivors.front().dist) + spread;

        double newest = 1e9;
        for (const Candidate& k : survivors) {
            newest = std::min(newest, k.s->offsetAgeSec);
            c.usedNames.push_back(k.s->name);
        }

        c.valid = true;
        c.synchronised = true;
        c.offsetSec = offset;
        c.dispersionSec = dispersion;
        c.ageSec = newest;
        c.used = static_cast<int>(survivors.size());

        // The station tag of the best-measured survivor -- by the same figure
        // that decides the weighting, since it is the same question. On the
        // frequencies WWV and WWVH share, which one is being heard genuinely
        // changes through the day, so this follows the decoder rather than the
        // configuration.
        const Candidate* tightest = &survivors.front();
        for (const Candidate& k : survivors) if (k.quality < tightest->quality) tightest = &k;
        c.refid = refidFor(tightest->s->station);

        // A leap warning is honoured only if every survivor agrees. One
        // receiver decoding the bit wrongly would otherwise announce a leap
        // second to every client on the network.
        c.leapPending = true;
        for (const Candidate& k : survivors) if (!k.s->leapPending) c.leapPending = false;

        std::lock_guard<std::mutex> lk(m_mu);
        m_haveLast = true;
        m_lastGoodOffset = offset;
        m_lastGoodDispersion = dispersion;
        m_lastGoodAt = nowRealtime;
        m_lastGoodMeasuredAt = nowRealtime - newest;
        m_last = c;
        return c;
    }

    // --- nothing usable: coast ---------------------------------------------
    // Sources that agreed but were too few to serve are healthy, and should
    // not look like it is their fault.
    for (const Candidate& k : survivors) {
        c.notUsedReasons[k.s->name] =
            format("healthy, but only %d source(s) agree and min_sources is %d",
                   static_cast<int>(survivors.size()), m_minSources);
    }
    std::lock_guard<std::mutex> lk(m_mu);
    if (m_haveLast) {
        const double age = nowRealtime - m_lastGoodAt;
        if (age <= m_coastSeconds) {
            c.valid = true;
            c.synchronised = true;
            c.offsetSec = m_lastGoodOffset;
            // The claimed accuracy decays at the assumed wander rate of an
            // undisciplined clock. This is the whole content of coasting: the
            // offset does not change, the honesty about it does.
            c.dispersionSec = m_lastGoodDispersion + age * (m_coastDriftPpm * 1e-6);
            c.ageSec = nowRealtime - m_lastGoodMeasuredAt;
            c.used = 0;
            c.refid = m_last.refid;
            c.note = cand.empty() ? "coasting: no source has a lock"
                                  : "coasting: no set of sources agreed";
        } else {
            c.valid = true;
            c.synchronised = false;
            c.offsetSec = m_lastGoodOffset;
            c.dispersionSec = m_lastGoodDispersion + age * (m_coastDriftPpm * 1e-6);
            c.ageSec = nowRealtime - m_lastGoodMeasuredAt;
            c.refid = m_last.refid;
            c.note = "unsynchronised: no lock for longer than the coast limit";
        }
    } else {
        c.valid = false;
        c.synchronised = false;
        c.note = cand.empty() ? "no source has produced a timestamp yet"
                              : "no set of sources agreed";
    }

    c.candidates = static_cast<int>(cand.size());

    m_last = c;
    return c;
}

Combined Selector::current() const {
    std::lock_guard<std::mutex> lk(m_mu);
    return m_last;
}

} // namespace ubersdr_ntp
