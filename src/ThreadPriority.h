// Scheduling for the threads that only report: the status page, its event
// streams, and MQTT. They yield to everything that keeps time -- the decoders
// at normal priority, and the NTP replies above both (NtpServer::enterRealtime).
#pragma once

#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

namespace ubersdr_ntp {

// A reporting thread's nice value. Raising it needs no privilege, so this holds
// even where real-time is refused and the replies run at normal priority too.
inline constexpr int kReportingNice = 10;

// Sets the calling thread's nice value. On Linux nice is per thread, and a
// thread started from this one inherits it, so a server's accept thread
// covers every connection it spawns. Best effort: a refusal changes nothing
// that matters to the time.
inline void setThreadNice(int nice) {
    ::setpriority(PRIO_PROCESS, static_cast<id_t>(::syscall(SYS_gettid)), nice);
}

} // namespace ubersdr_ntp
