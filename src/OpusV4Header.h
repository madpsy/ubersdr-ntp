#pragma once

// The version 4 header on an Opus frame.
//
// third_party/pcm_v4.hpp is the lossless path's decoder and is kept byte-
// identical with the copies in ka9q_ubersdr/clients so upstream fixes stay a
// straight `cp`. It has no Opus reader — the C++ clients that share it are IQ
// clients and never negotiate Opus — so this is that half, written here rather
// than patched into the vendored file.
//
// Mirrors PCMv4HeaderEncoder.AppendOpusHeader / PCMv4HeaderDecoder.DecodeOpus
// in ka9q_ubersdr/pcm_v4_header.go.
//
//   [flags u8]                                    1   always
//   [timestamp]                               8 or ~2   full at a resync, zigzag delta otherwise
//   [sampleRate uvarint][channels u8]            ~3   if the metadata bit is set
//   [power i16 LE][noise i16 LE]                  4   if the quality bit is set
//   [opus packet]                                 …
//
//   flags: bit 1 metadata   bit 0 quality   every other bit reserved
//
// Note the flag values are NOT the lossless path's: metadata is 1<<1 here and
// 1<<5 there. They are separate encoders in the server and the Opus one only
// ever needs two bits. Reusing kFlagMetadata from pcm_v4.hpp would parse every
// packet wrongly, which is why these are named and defined here rather than
// borrowed.
//
// There is no magic and no sample count: an Opus packet declares its own frame
// length, and the four-byte "PCM4" magic exists precisely so a reader can tell
// a lossless frame from one of these on a socket that carries both — a session
// that negotiated Opus still receives lossless frames the moment it tunes to an
// IQ mode. So: test isV4Frame() first, and only if that says no is the frame an
// Opus one.

#include "../third_party/pcm_v4.hpp"

#include <cstdint>
#include <string>

namespace ubersdr_ntp {

class OpusV4HeaderDecoder {
public:
    static constexpr std::uint8_t kFlagQuality  = 1u << 0;
    static constexpr std::uint8_t kFlagMetadata = 1u << 1;

    void reset() {
        m_haveMetadata = false;
        m_lastTS = 0;
        m_rate = 0;
        m_channels = 0;
        m_power = 0;
        m_noise = 0;
    }

    // Parses the header at the front of pkt. On success `bodyOff` is where the
    // Opus packet starts.
    //
    // Refuses a delta packet seen before any resynchronisation point rather
    // than guessing a timestamp, and a packet with a reserved flag bit rather
    // than ignoring it: both mean this reader and that writer disagree about
    // the format, and audio decoded on a guess is worse than audio dropped.
    bool decode(const std::uint8_t* pkt, std::size_t len,
                ubersdr::PCMv4Header& h, std::size_t& bodyOff, std::string& err) {
        if (len < 2) { err = "opus v4 header: packet too short"; return false; }

        const std::uint8_t flags = pkt[0];
        if (flags & ~static_cast<std::uint8_t>(kFlagQuality | kFlagMetadata)) {
            err = "opus v4 header: reserved flag bits set";
            return false;
        }
        std::size_t off = 1;
        const bool resync = (flags & kFlagMetadata) != 0;

        if (resync) {
            if (len < off + 8) { err = "opus v4 header: truncated timestamp"; return false; }
            std::uint64_t ts = 0;
            for (int i = 7; i >= 0; --i) ts = (ts << 8) | pkt[off + static_cast<std::size_t>(i)];
            m_lastTS = ts;
            off += 8;
        } else {
            if (!m_haveMetadata) {
                err = "opus v4 header: delta packet before any resynchronisation point";
                return false;
            }
            std::int64_t delta = 0;
            if (!ubersdr::readVarint(pkt, len, off, delta)) {
                err = "opus v4 header: malformed timestamp delta";
                return false;
            }
            m_lastTS = static_cast<std::uint64_t>(static_cast<std::int64_t>(m_lastTS) + delta);
        }
        h.timestampNanos = m_lastTS;

        if (resync) {
            std::uint64_t rate = 0;
            if (!ubersdr::readUvarint(pkt, len, off, rate)) {
                err = "opus v4 header: malformed sample rate";
                return false;
            }
            if (len < off + 1) { err = "opus v4 header: truncated channel count"; return false; }
            m_rate = static_cast<int>(rate);
            m_channels = static_cast<int>(pkt[off]);
            ++off;
            m_haveMetadata = true;
        }

        if (flags & kFlagQuality) {
            if (len < off + 4) { err = "opus v4 header: truncated signal quality"; return false; }
            m_power = static_cast<std::int16_t>(
                static_cast<std::uint16_t>(pkt[off]) | (static_cast<std::uint16_t>(pkt[off + 1]) << 8));
            m_noise = static_cast<std::int16_t>(
                static_cast<std::uint16_t>(pkt[off + 2]) | (static_cast<std::uint16_t>(pkt[off + 3]) << 8));
            off += 4;
        }

        if (m_rate <= 0 || m_channels <= 0) {
            err = "opus v4 header: implausible metadata";
            return false;
        }

        h.sampleRate = m_rate;
        h.channels = m_channels;
        h.basebandPower = ubersdr::pcmQualityToFloat(m_power);
        h.noise = ubersdr::pcmQualityToFloat(m_noise);
        h.sampleCount = 0;       // an Opus packet declares its own length
        h.profile = 0;
        h.shift = 0;
        h.escape = false;
        h.silent = false;

        bodyOff = off;
        return true;
    }

private:
    bool m_haveMetadata = false;
    std::uint64_t m_lastTS = 0;
    int m_rate = 0;
    int m_channels = 0;
    std::int16_t m_power = 0;
    std::int16_t m_noise = 0;
};

} // namespace ubersdr_ntp
