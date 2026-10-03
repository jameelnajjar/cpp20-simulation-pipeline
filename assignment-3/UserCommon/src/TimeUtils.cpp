// TimeUtils.cpp - implementation of UserCommon/TimeUtils.h.

#include <UserCommon/TimeUtils.h>

#include <atomic>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace user_common_213309941_213727837 {

namespace {

// Formats "now" as UTC using the supplied strftime pattern.
// gmtime_r is used rather than std::gmtime because the Simulator formats timestamps
// from several worker threads at once and std::gmtime returns a shared static buffer.
[[nodiscard]] std::string formatUtcNow(const char* pattern) {
    const auto now = std::chrono::system_clock::now();
    const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
    std::tm tm_utc{};
#if defined(_WIN32)
    gmtime_s(&tm_utc, &seconds); // Windows CRT equivalent of POSIX gmtime_r
#else
    ::gmtime_r(&seconds, &tm_utc); // POSIX thread-safe UTC breakdown
#endif
    std::ostringstream out;
    out << std::put_time(&tm_utc, pattern);
    return out.str();
}

// Process-wide counter appended by uniqueStamp() to guarantee distinct names
// even for two calls inside the same second from different threads.
std::atomic<unsigned long long> g_unique_counter{0};

} // namespace

// ISO-8601 UTC instant for the `generated_at_utc` YAML field.
std::string utcTimestamp() { return formatUtcNow("%Y-%m-%dT%H:%M:%SZ"); }

// Filename-safe UTC stamp (no colons, which are illegal on some filesystems).
std::string utcStampForFilename() { return formatUtcNow("%Y%m%d_%H%M%S"); }

// Filename-safe UTC stamp plus a zero-padded atomic counter.
std::string uniqueStamp() {
    const unsigned long long ordinal = g_unique_counter.fetch_add(1, std::memory_order_relaxed);
    std::ostringstream out;
    out << utcStampForFilename() << '_' << std::setw(6) << std::setfill('0') << ordinal;
    return out.str();
}

// Keeps [A-Za-z0-9._-] and turns everything else into '_'.
std::string sanitizeForFilename(std::string_view text) {
    std::string result;
    result.reserve(text.size());
    for (const char ch : text) {
        const bool keep = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
                          (ch >= '0' && ch <= '9') || ch == '.' || ch == '_' || ch == '-';
        result += keep ? ch : '_';
    }
    return result;
}

} // namespace user_common_213309941_213727837
