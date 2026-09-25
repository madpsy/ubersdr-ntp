// ubersdr-ntp-iqrecord: record IQ from an UberSDR receiver, lossless, with the
// receiver's own capture time for every packet.
//
//   ubersdr-ntp-iqrecord URL CARRIER_HZ SECONDS OUT_BASENAME
//
// Tunes on the carrier in IQ mode exactly as the daemon does for DCF77
// (mode=iq, ±6 kHz, PCM v4, min_margin=0), and writes:
//
//   OUT_BASENAME.wav        16-bit stereo I/Q at the stream's rate
//   OUT_BASENAME.times.csv  one row per packet: the WAV frame its first sample
//                           landed on, and radiod's GPS-synchronised timestamp
//                           of that sample, plus the packet's power and noise
//
// The CSV is what makes a recording usable for timing work: a WAV alone says
// nothing about when its first sample was taken. An undecodable packet
// is written as silence of the same length, as the daemon does. Samples are
// otherwise written back to back and never re-timed from the stamps, which
// jitter by milliseconds; a stamp of 0 means radiod had no capture time for
// that packet (a new channel, for a moment) and is written as 0.
//
// Not installed. Written to record MSF and Allouis for
// docs/lf-stations-msf-allouis.md, with DCF77 alongside as the reference.

#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXWebSocket.h>
#include <curl/curl.h>

#include "../third_party/pcm_v4.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

size_t curlWrite(char* p, size_t sz, size_t n, void* ud) {
    static_cast<std::string*>(ud)->append(p, sz * n);
    return sz * n;
}

std::string uuidV4() {
    std::ifstream f("/proc/sys/kernel/random/uuid");
    std::string s;
    std::getline(f, s);
    return s;
}

// POST /connection, which reserves the session the WebSocket then uses.
bool handshake(const std::string& url, const std::string& session, std::string& err) {
    CURL* c = curl_easy_init();
    if (!c) { err = "curl_easy_init"; return false; }
    const std::string body = "{\"user_session_id\":\"" + session + "\"}";
    std::string resp;
    struct curl_slist* h = curl_slist_append(nullptr, "Content-Type: application/json");
    h = curl_slist_append(h, "User-Agent: ubersdr-ntp-iqrecord");
    const std::string u = url + "/connection";
    curl_easy_setopt(c, CURLOPT_URL, u.c_str());
    curl_easy_setopt(c, CURLOPT_POSTFIELDS, body.c_str());
    curl_easy_setopt(c, CURLOPT_HTTPHEADER, h);
    curl_easy_setopt(c, CURLOPT_WRITEFUNCTION, curlWrite);
    curl_easy_setopt(c, CURLOPT_WRITEDATA, &resp);
    curl_easy_setopt(c, CURLOPT_TIMEOUT, 15L);
    const CURLcode r = curl_easy_perform(c);
    long code = 0;
    curl_easy_getinfo(c, CURLINFO_RESPONSE_CODE, &code);
    curl_slist_free_all(h);
    curl_easy_cleanup(c);
    if (r != CURLE_OK) { err = curl_easy_strerror(r); return false; }
    if (code != 200 || resp.find("\"allowed\":false") != std::string::npos) {
        err = "HTTP " + std::to_string(code) + ": " + resp;
        return false;
    }
    return true;
}

void put32(std::ofstream& f, uint32_t v) { f.write(reinterpret_cast<const char*>(&v), 4); }
void put16(std::ofstream& f, uint16_t v) { f.write(reinterpret_cast<const char*>(&v), 2); }

void writeWavHeader(std::ofstream& f, int rate, uint32_t frames) {
    const uint32_t data = frames * 4;
    f.seekp(0);
    f.write("RIFF", 4); put32(f, 36 + data); f.write("WAVE", 4);
    f.write("fmt ", 4); put32(f, 16); put16(f, 1); put16(f, 2);
    put32(f, static_cast<uint32_t>(rate)); put32(f, static_cast<uint32_t>(rate) * 4);
    put16(f, 4); put16(f, 16);
    f.write("data", 4); put32(f, data);
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 5) {
        std::fprintf(stderr, "usage: %s URL CARRIER_HZ SECONDS OUT_BASENAME\n", argv[0]);
        return 2;
    }
    std::string url = argv[1];
    while (!url.empty() && url.back() == '/') url.pop_back();
    const long carrier = std::atol(argv[2]);
    const double seconds = std::atof(argv[3]);
    const std::string base = argv[4];

    curl_global_init(CURL_GLOBAL_DEFAULT);
    ix::initNetSystem();

    const std::string session = uuidV4();
    std::string err;
    if (!handshake(url, session, err)) {
        std::fprintf(stderr, "POST /connection failed: %s\n", err.c_str());
        return 1;
    }

    std::string wsUrl = url;
    if (wsUrl.rfind("https://", 0) == 0) wsUrl = "wss://" + wsUrl.substr(8);
    else if (wsUrl.rfind("http://", 0) == 0) wsUrl = "ws://" + wsUrl.substr(7);
    wsUrl += "/ws?frequency=" + std::to_string(carrier) +
             "&mode=iq&bandwidthLow=-6000&bandwidthHigh=6000"
             "&format=pcm-zstd&min_margin=0&version=4&user_session_id=" + session;

    std::ofstream wav(base + ".wav", std::ios::binary);
    std::ofstream csv(base + ".times.csv");
    if (!wav || !csv) { std::fprintf(stderr, "cannot write %s.*\n", base.c_str()); return 1; }
    csv << "# ubersdr-ntp-iqrecord " << url << " carrier_hz=" << carrier
        << " mode=iq bandwidth=-6000..6000 min_margin=0\n"
        << "frame,timestamp_ns,frames,baseband_dbfs,noise_dbfs,host_unix_ns\n";

    std::mutex mu;
    ubersdr::PCMv4StreamDecoder dec;
    int rate = 0;
    uint64_t framesWritten = 0;
    uint64_t firstTs = 0;   // the first non-zero capture time
    long packets = 0, errors = 0;
    std::atomic<bool> open{false}, closed{false};
    std::string closeReason;
    const std::vector<int16_t> silence(48000 * 2, 0);

    auto writeSilence = [&](uint64_t frames) {
        while (frames > 0) {
            const uint64_t n = std::min<uint64_t>(frames, silence.size() / 2);
            wav.write(reinterpret_cast<const char*>(silence.data()), static_cast<std::streamsize>(n * 4));
            framesWritten += n;
            frames -= n;
        }
    };

    ix::WebSocket ws;
    ws.setUrl(wsUrl);
    ws.disableAutomaticReconnection();
    ix::SocketTLSOptions tls;
    tls.caFile = "SYSTEM";
    ws.setTLSOptions(tls);
    ws.setOnMessageCallback([&](const ix::WebSocketMessagePtr& m) {
        if (m->type == ix::WebSocketMessageType::Open) { open = true; return; }
        if (m->type == ix::WebSocketMessageType::Close) {
            closeReason = std::to_string(m->closeInfo.code) + " " + m->closeInfo.reason;
            closed = true; return;
        }
        if (m->type == ix::WebSocketMessageType::Error) {
            closeReason = m->errorInfo.reason; closed = true; return;
        }
        if (m->type != ix::WebSocketMessageType::Message || !m->binary) return;
        const auto* pkt = reinterpret_cast<const uint8_t*>(m->str.data());
        const size_t len = m->str.size();
        if (!ubersdr::PCMv4StreamDecoder::isV4Frame(pkt, len)) return;
        const auto host = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        std::lock_guard<std::mutex> lk(mu);
        ubersdr::PCMv4Header h;
        std::string e;
        const bool ok = dec.decode(pkt, len, h, e);
        if (h.channels != 2 || h.sampleRate <= 0 || h.sampleCount <= 0) {
            if (!ok && errors++ % 100 == 0) std::fprintf(stderr, "decode: %s\n", e.c_str());
            return;
        }
        if (rate == 0) { rate = h.sampleRate; writeWavHeader(wav, rate, 0); }
        if (h.sampleRate != rate) { std::fprintf(stderr, "rate changed %d -> %d\n", rate, h.sampleRate); return; }
        const int frames = h.sampleCount / 2;

        csv << framesWritten << ',' << h.timestampNanos << ',' << frames << ','
            << h.basebandPower << ',' << h.noise << ',' << host << '\n';
        if (ok) {
            wav.write(reinterpret_cast<const char*>(dec.samples()), static_cast<std::streamsize>(frames) * 4);
            framesWritten += static_cast<uint64_t>(frames);
        } else {
            if (errors++ % 100 == 0) std::fprintf(stderr, "decode: %s\n", e.c_str());
            writeSilence(static_cast<uint64_t>(frames));
        }
        if (firstTs == 0) firstTs = h.timestampNanos;
        ++packets;
        if (packets % 250 == 0) csv.flush();
    });
    ws.start();

    const auto t0 = std::chrono::steady_clock::now();
    auto lastPing = t0;
    for (;;) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (closed) break;
        const auto now = std::chrono::steady_clock::now();
        if (open && now - lastPing >= std::chrono::seconds(30)) {
            ws.send("{\"type\":\"ping\"}");   // the UberSDR session keepalive
            lastPing = now;
        }
        std::lock_guard<std::mutex> lk(mu);
        if (rate > 0 && static_cast<double>(framesWritten) >= seconds * rate) break;
        if (!open && now - t0 > std::chrono::seconds(30)) { closeReason = "did not open in 30 s"; closed = true; break; }
    }
    ws.stop();

    std::lock_guard<std::mutex> lk(mu);
    if (rate > 0) writeWavHeader(wav, rate, static_cast<uint32_t>(framesWritten));
    wav.close();
    csv.close();
    std::fprintf(stderr, "%s: %d Hz, %.1f s, %ld packets, %ld decode errors, first stamped sample %.3f UTC%s%s\n",
                 base.c_str(), rate, rate ? static_cast<double>(framesWritten) / rate : 0.0,
                 packets, errors, static_cast<double>(firstTs) * 1e-9,
                 closed ? ", closed: " : "", closed ? closeReason.c_str() : "");
    return rate > 0 ? 0 : 1;
}
