#pragma once

// Structured logging for a background daemon.
//
// Two destinations, both optional and independent: stderr (so `systemd` picks
// it up, and so a foreground run says something) and a file. The file is the
// one the user reads to see what each source is doing, so it is opened in
// append mode and flushed per line — a status block nobody can read until the
// process exits is worth nothing, and 30-second blocks are nowhere near
// frequent enough for buffering to matter.
//
// Every line carries a UTC timestamp, a level, and a short subsystem tag.
// Source lines carry the source name as the tag, so `grep wwv10` pulls one
// receiver's whole history out of a multi-source log.

#include <cstdio>
#include <mutex>
#include <string>

namespace ubersdr_ntp {

enum class LogLevel { Trace = 0, Debug, Info, Warn, Error };

const char* levelName(LogLevel l);
bool parseLevel(const std::string& s, LogLevel& out);

class Log {
public:
    static Log& instance();

    // Opens (or reopens) the log file. An empty path closes it. Returns false
    // and leaves the previous file alone if the new one cannot be opened —
    // losing the log we already have to a typo in a new path helps nobody.
    bool setFile(const std::string& path, std::string& err);

    void setLevel(LogLevel l) { m_level = l; }
    LogLevel level() const { return m_level; }

    // Whether to also write to stderr. Off makes sense only when a file is
    // set; with neither, the daemon is silent and unsupportable.
    void setStderr(bool on) { m_stderr = on; }

    bool enabled(LogLevel l) const { return l >= m_level; }

    void write(LogLevel l, const std::string& tag, const std::string& message);

    // A pre-formatted multi-line block (the periodic status report). Written
    // with the timestamp/level/tag prefix on the first line only, and the rest
    // indented, so a block stays visually one unit in a file that otherwise
    // has one record per line.
    void block(LogLevel l, const std::string& tag, const std::string& title,
               const std::string& body);

    // Reopen the current file. For SIGHUP, so logrotate can move it.
    void reopen();

private:
    Log() = default;

    std::mutex m_mu;
    std::FILE* m_file = nullptr;
    std::string m_path;
    LogLevel m_level = LogLevel::Info;
    bool m_stderr = true;
};

// printf-style front ends. Variadic rather than iostream because every call
// site here is a fixed format string with a few numbers in it, and the format
// string is what makes a log line greppable.
void logf(LogLevel l, const char* tag, const char* fmt, ...)
    __attribute__((format(printf, 3, 4)));

#define LOG_TRACE(tag, ...) ::ubersdr_ntp::logf(::ubersdr_ntp::LogLevel::Trace, tag, __VA_ARGS__)
#define LOG_DEBUG(tag, ...) ::ubersdr_ntp::logf(::ubersdr_ntp::LogLevel::Debug, tag, __VA_ARGS__)
#define LOG_INFO(tag, ...)  ::ubersdr_ntp::logf(::ubersdr_ntp::LogLevel::Info,  tag, __VA_ARGS__)
#define LOG_WARN(tag, ...)  ::ubersdr_ntp::logf(::ubersdr_ntp::LogLevel::Warn,  tag, __VA_ARGS__)
#define LOG_ERROR(tag, ...) ::ubersdr_ntp::logf(::ubersdr_ntp::LogLevel::Error, tag, __VA_ARGS__)

} // namespace ubersdr_ntp
