#include "Selector.h"

#include "Log.h"
#include "SampleClock.h"

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

// How long a source may stay refused before it is made to start over, and the
// most that wait may grow to.
//
// Refusal keeps a wrong source out of the served time, but it does not fix the
// source, and nothing inside one decoder can: an edge tracker that has locked
// onto the wrong part of the pulse still puts every minute marker in the right
// second, so the frame decodes, the voter certifies, and the lock holds for as
// long as the connection does. This daemon runs for months unattended, so
// "until someone restarts it" is not an answer. A refused source is therefore
// told to drop its connection and acquire from nothing -- which costs it one
// lock, about five minutes, and costs the served time nothing, since it was not
// contributing.
//
// The wait doubles each time the same source is sent back without having been
// accepted in between, so a receiver that is genuinely broken, or genuinely
// hearing something the others are not, reconnects once an hour rather than
// once every few minutes for ever.
constexpr double kReacquireAfterSec = 300.0;
constexpr double kReacquireMaxSec = 3600.0;

constexpr const char* kTag = "selector";

// NTP's stratum ceiling. 16 means unsynchronised; anything that would come out
// at or past it is not a time this server may claim to be serving.
constexpr int kMaxStratum = 15;

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
    bool primary;    // in the class trusted first
};

// The median of a set of candidate offsets: the class consensus, and what each
// member of the class is judged against.
double medianOffset(const std::vector<Candidate>& v) {
    if (v.empty()) return 0.0;
    std::vector<double> all;
    all.reserve(v.size());
    for (const Candidate& k : v) all.push_back(k.offset);
    std::sort(all.begin(), all.end());
    const std::size_t mid = all.size() / 2;
    return (all.size() % 2) ? all[mid] : 0.5 * (all[mid - 1] + all[mid]);
}

std::string format(const char* fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    return buf;
}

} // namespace

const char* servingClassName(ServingClass c) {
    switch (c) {
        case ServingClass::None:      return "none";
        case ServingClass::Primary:   return "primary";
        case ServingClass::Secondary: return "secondary";
        case ServingClass::Both:      return "both";
        case ServingClass::Coasting:  return "coasting";
    }
    return "?";
}

Selector::Selector(double coastSeconds, double coastDriftPpm, int minSources, ClockConfig clock)
    : m_coastSeconds(coastSeconds),
      m_coastDriftPpm(coastDriftPpm),
      m_minSources(minSources),
      m_clock(clock) {
    // In every mode but cold the secondary is connected from the start, so the
    // initial state has to say so -- main() asks before the first combine, and
    // a default of "inactive" would park a standby that was never meant to be.
    m_secondaryActive = m_clock.secondary != SecondaryMode::Cold;
    m_activationReason = m_secondaryActive
                             ? std::string("secondary mode is ") + secondaryModeName(m_clock.secondary)
                             : "cold standby: the primary sources are healthy";
}

Selector::Activation Selector::activation() const {
    std::lock_guard<std::mutex> lk(m_mu);
    return Activation{m_secondaryActive, m_activationReason};
}

Combined Selector::combine(const std::vector<SourceSnapshot>& snaps, double now) {
    Combined c;

    // Candidates, split by class. Both lists are built the same way and judged
    // the same way; what differs is who each is compared against, and which of
    // them is allowed to serve. See the header.
    std::vector<Candidate> primaryCand, secondaryCand;
    // Whether a secondary class exists at all, candidate or not. Without this
    // an install with no secondary configured would be told, while coasting,
    // that the secondary sources take over in sixty seconds -- which is a
    // promise nothing can keep and the sort of thing that sends someone
    // looking for a fault in the wrong place.
    bool secondaryConfigured = false;
    for (const SourceSnapshot& s : snaps) {
        if (!s.primaryClass) secondaryConfigured = true;
    }
    for (const SourceSnapshot& s : snaps) {
        // Whether the source has anything worth considering is the source's own
        // question, and it answers it in its own vocabulary: a receiver has a
        // decoder lock and a peer has a reach register, and neither concept
        // needs to be understood here. What IS decided here is freshness and
        // whether there is a dispersion to intersect on, because those are
        // this file's policy rather than the source's.
        //
        // Most specific first, so the reason names the step that is actually
        // missing rather than a symptom of it: an unlocked decoder also has no
        // offset, and "no offset" would send someone to the wrong place.
        std::string why;
        if (!s.ready) {
            why = s.notReadyReason.empty() ? "not ready" : s.notReadyReason;
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

        // A source's offset is against the daemon clock as of its newest
        // measurement, and moves at the daemon clock's rate error. Carried
        // along its own rate to NOW, so that every candidate below is
        // compared, intersected and averaged at one instant.
        const double offsetNow = s.offsetSec + s.offsetRate * (now - s.offsetAtSec);

        // The interval a source asserts. Its dispersion plus the age of its
        // newest measurement times the coast rate and the doubt in the rate it
        // was carried along: a source whose last reading is a minute old is a
        // minute less certain than it was.
        const double staleness = s.offsetAgeSec * (m_coastDriftPpm * 1e-6 + s.offsetRateUncertainty);
        const double quality = (s.weightDispersionSec > 0.0 ? s.weightDispersionSec
                                                            : s.dispersionSec) + staleness;
        (s.primaryClass ? primaryCand : secondaryCand)
            .push_back(Candidate{&s, offsetNow, s.dispersionSec + staleness, quality,
                                 s.weight > 0.0 ? s.weight : 1.0, s.primaryClass});
    }
    c.primaryCandidates = static_cast<int>(primaryCand.size());
    c.secondaryCandidates = static_cast<int>(secondaryCand.size());

    // --- agreement ----------------------------------------------------------
    //
    // WITHIN A CLASS, and never across the two. The test below refuses a source
    // that sits more than 30 ms from its fellows, and the only reason that is a
    // sound thing to do is that every radio source is hearing ONE transmitter
    // and ONE second edge: there is no physical arrangement of receivers that
    // explains more, so a source out there has decoded something else. Two NTP
    // peers are likewise two views of one hierarchy. But a radio source and an
    // NTP server share nothing at all -- they are independent measurements of
    // UTC, and the gap between them is the radio delay model's error, a real
    // quantity that can honestly exceed 30 ms on a path the model does not fit.
    // Judged together, a good receiver on a poorly modelled path would be
    // refused for disagreeing with the network, or worse, a network outage
    // would look like every NTP peer suddenly agreeing with each other and not
    // with the radio, and the radio would be the one convicted.
    //
    // So this runs twice, over one class each time, and the two never vote on
    // each other. What the classes say about one another is measured separately
    // and reported rather than acted on: see classDelta below.
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
    std::map<std::string, std::string> refusalWhy;

    auto judgeClass = [&](const std::vector<Candidate>& cand) {
    const double consensus = medianOffset(cand);

    for (const Candidate& k : cand) {
        SourceResidual r;
        r.name = k.s->name;
        r.kind = k.s->kind;
        r.instantSec = k.offset - consensus;
        r.peers = static_cast<int>(cand.size()) - 1;

        // Only readings taken against enough peers go into the average. With
        // one peer the median is the mean of the two and each residual is half
        // their difference -- a source 40 ms out reads 20 -- so blending those
        // in dragged a wrong source's average under the limit whenever a third
        // receiver dropped out, and it was "accepted" on a reading that proves
        // nothing. The time still passes (lastAtSec advances), so a gap is
        // neither evidence nor a reason to weight the next reading heavily.
        auto it = m_residualAvg.find(r.name);
        if (it == m_residualAvg.end()) it = m_residualAvg.emplace(r.name, ResidualState{}).first;
        ResidualState& ra = it->second;
        if (r.peers >= kMinPeersToJudge) {
            if (!ra.seeded) {
                ra.seeded = true;
                ra.averagedSec = r.instantSec;
                ra.firstAtSec = now;
            } else {
                const double dt = now - ra.lastAtSec;
                if (dt > 0.0) {
                    const double alpha = 1.0 - std::exp(-dt / kResidualTauSec);
                    ra.averagedSec += alpha * (r.instantSec - ra.averagedSec);
                }
            }
        }
        ra.lastAtSec = now;
        // Unseeded, the figure shown is the instant one: all there is, and
        // what a two-source install has always displayed.
        r.averagedSec = ra.seeded ? ra.averagedSec : r.instantSec;
        r.settledForSec = ra.seeded ? now - ra.firstAtSec : 0.0;
        r.haveAverage = true;
        r.refused = r.peers >= kMinPeersToJudge &&
                    r.settledForSec >= kResidualSettleSec &&
                    std::abs(r.averagedSec) > kMaxDisagreementSec;
        // Whether the consensus was entitled to a verdict at all. Not refused
        // is only "accepted" when it was; otherwise it is "could not say".
        const bool judged = r.peers >= kMinPeersToJudge && r.settledForSec >= kResidualSettleSec;
        ReacquireState& rq = m_reacquire[r.name];
        if (r.refused) {
            refused.insert(r.name);
            if (rq.refusedSinceSec <= 0.0) rq.refusedSinceSec = now;
            const double wait = std::min(kReacquireAfterSec * std::pow(2.0, rq.count),
                                         kReacquireMaxSec);
            const double refusedFor = now - rq.refusedSinceSec;
            const std::string base =
                format("refused: %+.0f ms from the other %s sources (limit ±%.0f ms)",
                       r.averagedSec * 1000.0, sourceKindName(r.kind),
                       kMaxDisagreementSec * 1000.0);
            if (refusedFor >= wait) {
                ++rq.count;
                rq.refusedSinceSec = 0.0;
                c.reacquireNames.push_back(r.name);
                refusalWhy[r.name] = base + "; re-acquiring it from scratch now";
                LOG_WARN(kTag, "%s has been refused for %.0f min; making it re-acquire from "
                         "scratch (attempt %d, next wait %.0f min if it is still refused)",
                         r.name.c_str(), refusedFor / 60.0, rq.count,
                         std::min(kReacquireAfterSec * std::pow(2.0, rq.count),
                                  kReacquireMaxSec) / 60.0);
            } else {
                refusalWhy[r.name] = base + format("; re-acquiring it from scratch in %.0f min "
                                                   "if it does not come back",
                                                   std::ceil((wait - refusedFor) / 60.0));
            }
        } else if (judged) {
            // Accepted by a consensus that was entitled to judge it: the
            // receiver is fine, the refusal is over, and the next one starts
            // the backoff over.
            rq.refusedSinceSec = 0.0;
            rq.count = 0;
        }
        // Otherwise it could not be judged -- too few peers, or not settled --
        // which is not the same as being accepted, so the timer holds. Live, a
        // third receiver that lost lock every few minutes left a source 40 ms
        // out with a single peer each time; a timer that reset on that never
        // reached five minutes in half an hour, and nothing recovered.
        if (r.refused && !it->second.refusalLogged) {
            LOG_WARN(kTag, "%s disagrees with the other %s sources by %+.0f ms; "
                     "not a delay model this far out — refusing it",
                     r.name.c_str(), sourceKindName(r.kind), r.averagedSec * 1000.0);
        } else if (!r.refused && judged && it->second.refusalLogged) {
            LOG_INFO(kTag, "%s agrees with the other %s sources again (%+.0f ms); using it",
                     r.name.c_str(), sourceKindName(r.kind), r.averagedSec * 1000.0);
        }
        // "Agrees again" only on a real verdict: a source that merely lost its
        // peers has not agreed with anyone, and saying so every time a third
        // receiver dropped lock filled the log with recoveries that were not.
        if (r.refused || judged) it->second.refusalLogged = r.refused;
        c.residuals.push_back(std::move(r));
    }
    };
    judgeClass(primaryCand);
    judgeClass(secondaryCand);

    // A source sent back to start over is judged afresh when it returns: its
    // old average describes the lock it is abandoning, and carried over it
    // would convict the new one before the settling guard had a chance to
    // apply.
    for (const std::string& n : c.reacquireNames) m_residualAvg.erase(n);

    // A refused source that drops out of the candidates (lost lock, went
    // stale) keeps its timer too. It was not accepted while it was away, and a
    // decoder that holds a wrong lock for a minute, loses it, and takes the
    // same wrong lock again is exactly the fault this exists to clear.

    // Drop what the consensus refuses. This is deliberately NOT a correction:
    // a source that disagrees is telling you it decoded something else, and
    // learning an offset that makes it agree would turn a detected fault into
    // an undetectable one.
    auto dropRefused = [&](std::vector<Candidate>& cand) {
        if (refused.empty()) return;
        std::vector<Candidate> kept;
        for (const Candidate& k : cand) {
            if (refused.count(k.s->name)) {
                c.rejectedNames.push_back(k.s->name);
                c.notUsedReasons[k.s->name] = refusalWhy[k.s->name];
            } else {
                kept.push_back(k);
            }
        }
        cand.swap(kept);
    };
    dropRefused(primaryCand);
    dropRefused(secondaryCand);
    c.primaryCandidates = static_cast<int>(primaryCand.size());
    c.secondaryCandidates = static_cast<int>(secondaryCand.size());

    // --- what the two classes say about each other --------------------------
    //
    // The one measurement in this program that can see the constants every
    // radio source shares. The chain delay, the codec delay and the decoder's
    // edge bias are identical on every receiver, so they cancel exactly in the
    // residuals above and no number of receivers can measure them; an upstream
    // NTP server does not share them, so the gap between the two classes'
    // consensuses contains them.
    //
    // Reported, never applied. Correcting the radio to agree with the network
    // would make the two agree by construction and destroy the only independent
    // check in the system -- and it would be correcting a stratum-1 radio clock
    // to match a stratum-2 network one, which is the wrong way round on a
    // machine whose whole purpose is not depending on that network.
    if (!primaryCand.empty() && !secondaryCand.empty()) {
        ClassDelta& d = c.classDelta;
        d.valid = true;
        d.primarySources = static_cast<int>(primaryCand.size());
        d.secondarySources = static_cast<int>(secondaryCand.size());
        d.instantSec = medianOffset(primaryCand) - medianOffset(secondaryCand);
        // Smoothed over the same five minutes a source residual is, and for the
        // same reason: one reading of this is one fade, and the figure is only
        // interesting as a systematic.
        if (!m_classDelta.seeded) {
            m_classDelta.seeded = true;
            m_classDelta.averagedSec = d.instantSec;
            m_classDelta.firstAtSec = now;
        } else {
            const double dt = now - m_classDelta.lastAtSec;
            if (dt > 0.0) {
                const double alpha = 1.0 - std::exp(-dt / kResidualTauSec);
                m_classDelta.averagedSec += alpha * (d.instantSec - m_classDelta.averagedSec);
            }
        }
        m_classDelta.lastAtSec = now;
        d.averagedSec = m_classDelta.averagedSec;
        d.haveAverage = true;
        d.settledForSec = now - m_classDelta.firstAtSec;
    } else {
        // One class is not being measured. The average is not carried across
        // the gap: a figure resumed from before an outage would describe two
        // sets of conditions averaged together with nothing to say where one
        // ended.
        m_classDelta = ClassDeltaState{};
    }

    // --- which class serves -------------------------------------------------
    //
    // See ClockConfig for what the three modes mean and why failing over is
    // cheap while failing back is not.
    const bool primaryUsable = static_cast<int>(primaryCand.size()) >= m_minSources &&
                               !primaryCand.empty();
    const bool secondaryUsable =
        static_cast<int>(secondaryCand.size()) >= m_clock.minSecondarySources &&
        !secondaryCand.empty();

    if (primaryUsable) {
        if (m_primaryHealthySince <= 0.0) m_primaryHealthySince = now;
        m_primaryLostSince = 0.0;
    } else {
        if (m_primaryLostSince <= 0.0) m_primaryLostSince = now;
        m_primaryHealthySince = 0.0;
    }

    std::vector<Candidate> cand;
    if (m_clock.secondary == SecondaryMode::Always) {
        // One pool. The intersection then protects the answer against a bad
        // source of either kind, which is the reason this mode is worth having:
        // a radio source and an NTP server that agree to a few milliseconds are
        // a far stronger statement than either alone.
        cand = primaryCand;
        cand.insert(cand.end(), secondaryCand.begin(), secondaryCand.end());
        m_onSecondary = false;
        m_secondaryActive = true;
        m_activationReason = "secondary mode is always: both classes contribute";
    } else {
        const double lostFor = m_primaryLostSince > 0.0 ? now - m_primaryLostSince : 0.0;
        const double healthyFor = m_primaryHealthySince > 0.0 ? now - m_primaryHealthySince : 0.0;

        if (!m_onSecondary) {
            if (!primaryUsable && secondaryUsable && lostFor >= m_clock.failoverAfterSec) {
                m_onSecondary = true;
                LOG_WARN(kTag, "failing over to the %s sources: the %s sources have had "
                         "nothing usable for %.0f s",
                         sourceKindName(m_clock.primary == SourceKind::Radio ? SourceKind::Ntp
                                                                             : SourceKind::Radio),
                         sourceKindName(m_clock.primary), lostFor);
            } else if (!primaryUsable && secondaryConfigured &&
                       lostFor >= m_clock.failoverAfterSec) {
                c.servingNote = "the primary sources have nothing usable and the secondary "
                                "is not ready either";
            } else if (!primaryUsable && secondaryConfigured) {
                c.failoverInSec = std::max(0.0, m_clock.failoverAfterSec - lostFor);
            }
        } else {
            if (primaryUsable && healthyFor >= m_clock.failbackAfterSec) {
                m_onSecondary = false;
                LOG_INFO(kTag, "failing back to the %s sources: healthy again for %.0f s",
                         sourceKindName(m_clock.primary), healthyFor);
            } else if (primaryUsable) {
                c.failbackInSec = std::max(0.0, m_clock.failbackAfterSec - healthyFor);
            }
        }

        cand = m_onSecondary ? secondaryCand : primaryCand;

        // Whether the secondary should be holding connections at all. Only cold
        // mode ever says no, and it comes up the MOMENT the primary drops
        // rather than after the failover hold-down: acquiring takes minutes, so
        // waiting a minute before starting would spend the hold-down doing
        // nothing. It stays up until the primary has proven itself for the full
        // failback interval, so a receiver that re-locks and loses it again
        // does not tear the standby down between attempts.
        if (m_clock.secondary == SecondaryMode::Cold) {
            // Three ways to be up, and the third is hysteretic ON m_secondaryActive
            // rather than on the clock alone. Without that qualifier the
            // settling clause is also true at startup -- the primary has been
            // healthy for less than the failback interval because the daemon
            // has only just begun -- and a cold standby would connect for five
            // minutes on every start, which is the one thing the mode exists
            // to avoid. It must mean "it is already up, hold it there", not
            // "the primary has not been up long".
            const bool want = m_onSecondary || !primaryUsable ||
                              (m_secondaryActive && m_primaryHealthySince > 0.0 &&
                               healthyFor < m_clock.failbackAfterSec);
            if (want != m_secondaryActive) {
                LOG_INFO(kTag, "%s the %s standby", want ? "bringing up" : "standing down",
                         sourceKindName(m_clock.primary == SourceKind::Radio ? SourceKind::Ntp
                                                                             : SourceKind::Radio));
            }
            m_secondaryActive = want;
            m_activationReason =
                want ? (m_onSecondary ? "serving: the primary sources have nothing usable"
                                      : "warming up: the primary sources have nothing usable")
                     : format("cold standby: the primary sources are healthy%s",
                              healthyFor > 0.0 ? format(" (%.0f s)", healthyFor).c_str() : "");
        } else {
            m_secondaryActive = true;
            m_activationReason = "secondary mode is standby: measured continuously, "
                                 "held out of the served time while the primary is healthy";
        }

        // Held-out secondary candidates are healthy and should not look as
        // though something is wrong with them.
        if (!m_onSecondary) {
            for (const Candidate& k : secondaryCand) {
                c.notUsedReasons[k.s->name] =
                    format("healthy, standing by: the %s sources are serving",
                           sourceKindName(m_clock.primary));
            }
        } else {
            for (const Candidate& k : primaryCand) {
                c.notUsedReasons[k.s->name] =
                    c.failbackInSec > 0.0
                        ? format("healthy again, taking back in %.0f s — held off that long so "
                                 "a source that keeps re-locking cannot step the clock each time",
                                 c.failbackInSec)
                        : std::string("recovering: it must stay healthy for a while before it "
                                      "takes back");
            }
        }
    }
    c.candidates = static_cast<int>(cand.size());

    // The minimum that must agree. min_sources governs the primary class; a set
    // made entirely of secondary sources answers to the secondary's own figure,
    // because "two receivers must agree" is a statement about receivers and
    // cannot be satisfied, or meaningfully applied, by one NTP server.
    bool anyPrimary = false;
    for (const Candidate& k : cand) if (k.primary) anyPrimary = true;
    const int minToServe = anyPrimary ? m_minSources : m_clock.minSecondarySources;

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

    if (static_cast<int>(survivors.size()) >= minToServe && !survivors.empty()) {
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

        // The rate the served offset moves at. Every source measures the same
        // thing -- the daemon clock against UTC -- so their rates are averaged
        // by how well each knows it. A source that has not measured one yet
        // has nothing to add; if none has, the combination has not either.
        double rw = 0.0, rwx = 0.0, rateBound = 0.0;
        for (const Candidate& k : survivors) {
            rateBound = std::max(rateBound, k.s->offsetRateUncertainty);
            if (!k.s->offsetRateMeasured) continue;
            const double u = std::max(k.s->offsetRateUncertainty, 1e-9);
            rw += 1.0 / (u * u);
            rwx += k.s->offsetRate / (u * u);
        }

        c.valid = true;
        c.synchronised = true;
        c.offsetSec = offset;
        c.atSec = now;
        c.rateMeasured = rw > 0.0;
        c.rate = rw > 0.0 ? rwx / rw : 0.0;
        // Survivors whose rates disagree are measuring their own path's wander
        // as well as the crystal, so the combination is no better known than
        // that disagreement, however tight each claims to be -- the same rule
        // as the spread term in the dispersion above.
        double rateSpread = 0.0;
        for (const Candidate& k : survivors) {
            if (k.s->offsetRateMeasured) rateSpread = std::max(rateSpread, std::abs(k.s->offsetRate - c.rate));
        }
        c.rateUncertainty = rw > 0.0 ? std::max(1.0 / std::sqrt(rw), rateSpread) : rateBound;
        c.hostOffsetSec = offset + daemonMinusRealtime();
        c.dispersionSec = dispersion;
        c.ageSec = newest;
        c.used = static_cast<int>(survivors.size());

        // --- stratum, root delay and the reference identifier ---------------
        //
        // NTP's own rule: one more than the lowest stratum among the survivors.
        // A radio source is a REFERENCE rather than a server, so it counts as
        // stratum 0 and serving from one is stratum 1; an upstream at stratum 2
        // makes this stratum 3. A mixed set comes out at 1 because a radio
        // reference really is in it, which is what NTP would say of any server
        // with a refclock among its selected peers.
        //
        // This is the field a client uses to decide how much of the tree it is
        // trusting, so getting it right is not cosmetic. Serving stratum 1 off
        // a pool server would be a lie of exactly the kind the rest of this
        // program takes trouble to avoid.
        int lowest = 16;
        bool anyRadioSurvivor = false;
        for (const Candidate& k : survivors) {
            if (k.s->kind == SourceKind::Radio) { lowest = 0; anyRadioSurvivor = true; }
            else lowest = std::min(lowest, k.s->ntp.stratum);
        }
        c.stratum = std::clamp(lowest + 1, 1, kMaxStratum);

        // The tightest survivor -- by the same figure that decides the
        // weighting, since it is the same question -- names the reference.
        const Candidate* tightest = &survivors.front();
        for (const Candidate& k : survivors) if (k.quality < tightest->quality) tightest = &k;

        if (anyRadioSurvivor) {
            // A radio survivor makes this a stratum-1 radio clock, so the refid
            // is the station's registered identifier and the root delay is
            // genuinely zero: there is no NTP path above us. On the frequencies
            // WWV and WWVH share, which one is being heard genuinely changes
            // through the day, so the tag follows the decoder rather than the
            // configuration -- and the tightest survivor is picked from among
            // the RADIO ones, since an upstream has no station to name.
            const Candidate* tightestRadio = nullptr;
            for (const Candidate& k : survivors) {
                if (k.s->kind != SourceKind::Radio) continue;
                if (!tightestRadio || k.quality < tightestRadio->quality) tightestRadio = &k;
            }
            c.refid = refidFor(tightestRadio->s->station);
            c.refidIsAddress = false;
            c.refidAddress = 0;
            c.rootDelaySec = 0.0;
        } else {
            // Serving from upstreams alone. RFC 5905 wants the reference's
            // address in the refid above stratum 1, and the root delay is the
            // whole path back to the primary reference: what the upstream
            // claimed, plus the round trip to it.
            // The address, without the port: a reference identifier names a
            // machine, and "192.0.2.1:123" is not one.
            c.refid = tightest->s->ntp.addressHost.empty() ? tightest->s->ntp.server
                                                           : tightest->s->ntp.addressHost;
            c.refidIsAddress = true;
            c.refidAddress = tightest->s->ntp.addressRefid;
            c.rootDelaySec = tightest->s->ntp.rootDelaySec + tightest->s->ntp.delaySec;
        }

        // Which class the answer came from, for the status page and the log.
        bool anyPrimarySurvivor = false, anySecondarySurvivor = false;
        for (const Candidate& k : survivors) (k.primary ? anyPrimarySurvivor : anySecondarySurvivor) = true;
        c.serving = anyPrimarySurvivor && anySecondarySurvivor ? ServingClass::Both
                  : anyPrimarySurvivor                         ? ServingClass::Primary
                                                               : ServingClass::Secondary;
        if (c.serving == ServingClass::Secondary && m_clock.secondary != SecondaryMode::Always) {
            c.servingNote = format("failed over: serving from the %s sources",
                                   sourceKindName(m_clock.primary == SourceKind::Radio
                                                      ? SourceKind::Ntp : SourceKind::Radio));
        }

        // A leap warning is honoured only if every survivor agrees. One
        // receiver decoding the bit wrongly would otherwise announce a leap
        // second to every client on the network.
        c.leapPending = true;
        for (const Candidate& k : survivors) if (!k.s->leapPending) c.leapPending = false;

        std::lock_guard<std::mutex> lk(m_mu);
        m_haveLast = true;
        m_lastGoodOffset = offset;
        m_lastGoodRate = c.rate;
        m_lastGoodRateUncertainty = c.rateUncertainty;
        m_lastGoodRateMeasured = c.rateMeasured;
        m_lastGoodDispersion = dispersion;
        m_lastGoodAt = now;
        m_lastGoodMeasuredAt = now - newest;
        m_last = c;
        return c;
    }

    // --- nothing usable: coast ---------------------------------------------
    // Sources that agreed but were too few to serve are healthy, and should
    // not look like it is their fault.
    for (const Candidate& k : survivors) {
        c.notUsedReasons[k.s->name] =
            format("healthy, but only %d source(s) agree and the minimum is %d",
                   static_cast<int>(survivors.size()), minToServe);
    }
    std::lock_guard<std::mutex> lk(m_mu);
    if (m_haveLast) {
        const double age = now - m_lastGoodAt;
        // The offset carries on along the rate it was last measured moving at:
        // the daemon clock is a crystal, and a crystal keeps its rate when the
        // radio goes quiet. The claimed accuracy decays at the assumed wander
        // of an undisciplined oscillator plus the doubt in that rate. This is
        // the whole content of coasting: the clock keeps counting, and the
        // honesty about it decays.
        c.valid = true;
        c.offsetSec = m_lastGoodOffset + m_lastGoodRate * age;
        c.atSec = now;
        c.rate = m_lastGoodRate;
        c.rateUncertainty = m_lastGoodRateUncertainty;
        c.rateMeasured = m_lastGoodRateMeasured;
        c.hostOffsetSec = c.offsetSec + daemonMinusRealtime();
        c.dispersionSec = m_lastGoodDispersion + age * (m_coastDriftPpm * 1e-6 + m_lastGoodRateUncertainty);
        c.ageSec = now - m_lastGoodMeasuredAt;
        // The reference is still the one the last good offset came from: this
        // IS that offset, extrapolated, so naming anything else would attribute
        // it to a source that did not produce it. Stratum and root delay travel
        // with it for the same reason -- a coasted answer that came from an
        // upstream is still that upstream's, however stale.
        c.refid = m_last.refid;
        c.refidIsAddress = m_last.refidIsAddress;
        c.refidAddress = m_last.refidAddress;
        c.stratum = m_last.stratum;
        c.rootDelaySec = m_last.rootDelaySec;
        c.serving = ServingClass::Coasting;
        if (age <= m_coastSeconds) {
            c.synchronised = true;
            c.used = 0;
            c.note = cand.empty() ? "coasting: no source has a lock"
                                  : "coasting: no set of sources agreed";
            // Coasting past a failover that has not happened yet is worth
            // saying, because it is the one state where the answer is getting
            // worse on purpose while a fix is already scheduled.
            if (c.failoverInSec > 0.0) {
                c.note += format("; the secondary sources take over in %.0f s",
                                 c.failoverInSec);
            }
        } else {
            c.synchronised = false;
            c.note = "unsynchronised: no lock for longer than the coast limit";
        }
    } else {
        // Never had a time from the radio. The host's own clock is the only
        // guess there is, and it is what an unsynchronised reply has always
        // carried; LI=3 and stratum 0 say not to believe it.
        c.valid = false;
        c.synchronised = false;
        c.offsetSec = -daemonMinusRealtime();
        c.atSec = now;
        c.note = cand.empty() ? "no source has produced a timestamp yet"
                              : "no set of sources agreed";
        c.serving = ServingClass::None;
        c.stratum = 16;
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
