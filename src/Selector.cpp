#include "Selector.h"

#include "Log.h"

#include <algorithm>
#include <cmath>

namespace ubersdr_ntp {

namespace {

// A source's measurements must be no older than this to be a candidate. Three
// minutes is three missed WWV frames: long enough to ride out a fade that the
// decoder itself holds a lock through, short enough that a dead source is not
// still voting ten minutes later.
constexpr double kCandidateMaxAgeSec = 180.0;

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
    double weight;
};

} // namespace

Selector::Selector(double coastSeconds, double coastDriftPpm, int minSources)
    : m_coastSeconds(coastSeconds),
      m_coastDriftPpm(coastDriftPpm),
      m_minSources(minSources) {}

Combined Selector::combine(const std::vector<SourceSnapshot>& snaps, double nowRealtime) {
    Combined c;

    std::vector<Candidate> cand;
    for (const SourceSnapshot& s : snaps) {
        if (!s.enabled) continue;
        if (!s.haveOffset) { c.rejectedNames.push_back(s.name); continue; }
        if (s.clockState != "locked") { c.rejectedNames.push_back(s.name); continue; }
        if (s.offsetAgeSec > kCandidateMaxAgeSec) { c.rejectedNames.push_back(s.name); continue; }
        if (s.dispersionSec <= 0.0) { c.rejectedNames.push_back(s.name); continue; }

        // The interval a source asserts. Its dispersion plus the age of its
        // newest measurement times the coast rate: a source whose last reading
        // is a minute old is a minute less certain than it was.
        const double staleness = s.offsetAgeSec * (m_coastDriftPpm * 1e-6);
        cand.push_back(Candidate{&s, s.offsetSec, s.dispersionSec + staleness,
                                 s.weight > 0.0 ? s.weight : 1.0});
    }
    c.candidates = static_cast<int>(cand.size());

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
            else c.rejectedNames.push_back(k.s->name);
        }
    }

    if (static_cast<int>(survivors.size()) >= m_minSources && !survivors.empty()) {
        // --- weighted combine ----------------------------------------------
        double sw = 0.0, swx = 0.0;
        for (const Candidate& k : survivors) {
            const double w = k.weight / (k.dist * k.dist);
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
            const double w = k.weight / (k.dist * k.dist);
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

        // The station tag of the survivor with the smallest interval. On the
        // frequencies WWV and WWVH share, which one is being heard genuinely
        // changes through the day, so this follows the decoder rather than the
        // configuration.
        const Candidate* tightest = &survivors.front();
        for (const Candidate& k : survivors) if (k.dist < tightest->dist) tightest = &k;
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
        m_last = c;
        return c;
    }

    // --- nothing usable: coast ---------------------------------------------
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
            c.ageSec = age;
            c.used = 0;
            c.refid = m_last.refid;
            c.note = cand.empty() ? "coasting: no source has a lock"
                                  : "coasting: no set of sources agreed";
        } else {
            c.valid = true;
            c.synchronised = false;
            c.offsetSec = m_lastGoodOffset;
            c.dispersionSec = m_lastGoodDispersion + age * (m_coastDriftPpm * 1e-6);
            c.ageSec = age;
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
