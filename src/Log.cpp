#include "Log.h"

#include <cerrno>
#include <cstdarg>
#include <cstring>
#include <ctime>
#include <sys/time.h>
#include <vector>

namespace ubersdr_ntp {

const char* levelName(LogLevel l) {
    switch (l) {
        case LogLevel::Trace: return "TRACE";
        case LogLevel::Debug: return "DEBUG";
        case LogLevel::Info:  return "INFO ";
        case LogLevel::Warn:  return "WARN ";
        case LogLevel::Error: return "ERROR";
    }
    return "?????";
}

bool parseLevel(const std::string& s, LogLevel& out) {
    if (s == "trace") { out = LogLevel::Trace; return true; }
    if (s == "debug") { out = LogLevel::Debug; return true; }
    if (s == "info")  { out = LogLevel::Info;  return true; }
    if (s == "warn" || s == "warning") { out = LogLevel::Warn; return true; }
    if (s == "error") { out = LogLevel::Error; return true; }
    return false;
}

Log& Log::instance() {
    static Log inst;
    return inst;
}

bool Log::setFile(const std::string& path, std::string& err) {
    if (path.empty()) {
        std::lock_guard<std::mutex> lk(m_mu);
        if (m_file) std::fclose(m_file);
        m_file = nullptr;
        m_path.clear();
        return true;
    }

    std::FILE* f = std::fopen(path.c_str(), "a");
    if (!f) {
        err = std::string("cannot open ") + path + ": " + std::strerror(errno);
        return false;
    }

    std::lock_guard<std::mutex> lk(m_mu);
    if (m_file) std::fclose(m_file);
    m_file = f;
    m_path = path;
    return true;
}

void Log::reopen() {
    std::lock_guard<std::mutex> lk(m_mu);
    if (m_path.empty()) return;
    std::FILE* f = std::fopen(m_path.c_str(), "a");
    if (!f) return;   // keep the old handle; a failed reopen must not silence us
    if (m_file) std::fclose(m_file);
    m_file = f;
}

// "2026-09-12 13:55:04.271Z" — a fixed-width, sortable, unambiguous stamp.
// Milliseconds because state changes land within a second of each other during
// an acquisition and their order is the interesting part.
static std::string stampNow() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    struct tm tmv;
    gmtime_r(&tv.tv_sec, &tmv);
    char buf[64];
    std::snprintf(buf, sizeof buf, "%04d-%02d-%02d %02d:%02d:%02d.%03dZ",
                  tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
                  tmv.tm_hour, tmv.tm_min, tmv.tm_sec,
                  static_cast<int>(tv.tv_usec / 1000));
    return buf;
}

void Log::write(LogLevel l, const std::string& tag, const std::string& message) {
    if (l < m_level) return;

    // Tag padded to a fixed width so the message column lines up down the file.
    // Source names are user-chosen and can be long; a long one pushes its own
    // line out rather than widening every other line.
    char head[128];
    std::snprintf(head, sizeof head, "%s %s [%-12s] ",
                  stampNow().c_str(), levelName(l), tag.c_str());

    std::lock_guard<std::mutex> lk(m_mu);
    if (m_stderr) {
        std::fputs(head, stderr);
        std::fputs(message.c_str(), stderr);
        std::fputc('\n', stderr);
        std::fflush(stderr);
    }
    if (m_file) {
        std::fputs(head, m_file);
        std::fputs(message.c_str(), m_file);
        std::fputc('\n', m_file);
        std::fflush(m_file);
    }
}

void Log::block(LogLevel l, const std::string& tag, const std::string& title,
                const std::string& body) {
    if (l < m_level) return;

    char head[128];
    std::snprintf(head, sizeof head, "%s %s [%-12s] ",
                  stampNow().c_str(), levelName(l), tag.c_str());

    std::string out = head;
    out += title;
    out += '\n';
    // Indent continuation lines past the header so the block reads as a unit
    // and a `grep -v '^ '` strips it back to one-line records.
    std::size_t pos = 0;
    while (pos < body.size()) {
        std::size_t nl = body.find('\n', pos);
        if (nl == std::string::npos) nl = body.size();
        out += "    ";
        out.append(body, pos, nl - pos);
        out += '\n';
        pos = nl + 1;
    }

    std::lock_guard<std::mutex> lk(m_mu);
    if (m_stderr) { std::fputs(out.c_str(), stderr); std::fflush(stderr); }
    if (m_file)   { std::fputs(out.c_str(), m_file); std::fflush(m_file); }
}

void logf(LogLevel l, const char* tag, const char* fmt, ...) {
    Log& log = Log::instance();
    if (!log.enabled(l)) return;

    // Two-pass so a long line (a status row, a server error message) is never
    // silently truncated.
    va_list ap;
    va_start(ap, fmt);
    va_list ap2;
    va_copy(ap2, ap);
    const int n = std::vsnprintf(nullptr, 0, fmt, ap);
    va_end(ap);
    if (n < 0) { va_end(ap2); return; }
    std::vector<char> buf(static_cast<std::size_t>(n) + 1);
    std::vsnprintf(buf.data(), buf.size(), fmt, ap2);
    va_end(ap2);

    log.write(l, tag, std::string(buf.data(), static_cast<std::size_t>(n)));
}

} // namespace ubersdr_ntp
