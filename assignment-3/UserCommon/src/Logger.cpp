// Logger.cpp - implementation of UserCommon/Logger.h.

#include <UserCommon/Logger.h>
#include <UserCommon/TimeUtils.h>

#include <iostream>
#include <system_error>
#include <utility>

namespace user_common_213309941_213727837 {

namespace {

// Maps a LogLevel enumerator onto the literal printed in the log line.
[[nodiscard]] std::string_view levelText(LogLevel level) {
    switch (level) {
        case LogLevel::Info:    return "INFO";
        case LogLevel::Warning: return "WARNING";
        case LogLevel::Error:   return "ERROR";
    }
    return "INFO";
}

} // namespace

// Stderr-only logger: no file is opened, echoing stays on.
Logger::Logger() = default;

// File-backed logger: creates the parent directory, opens in append mode, silences echo.
Logger::Logger(std::filesystem::path path) : path_(std::move(path)), echo_to_stderr_(false) {
    std::error_code ec;
    if (path_.has_parent_path()) {
        std::filesystem::create_directories(path_.parent_path(), ec);
    }
    stream_.open(path_, std::ios::out | std::ios::app);
    if (!stream_.is_open()) {
        // Degrade instead of throwing: a missing log must not abort a mission.
        path_.clear();
        echo_to_stderr_ = true;
    }
}

// Flushes whatever is buffered before the stream object dies.
Logger::~Logger() {
    const std::lock_guard<std::mutex> lock{mutex_};
    if (stream_.is_open()) {
        stream_.flush();
        stream_.close();
    }
}

// Toggles console mirroring.
void Logger::setEchoToStderr(bool echo) {
    const std::lock_guard<std::mutex> lock{mutex_};
    echo_to_stderr_ = echo;
}

// Convenience wrapper over write() with an empty code field.
void Logger::info(std::string_view message) { write(LogLevel::Info, {}, message); }

// Convenience wrapper over write() with an empty code field.
void Logger::warning(std::string_view message) { write(LogLevel::Warning, {}, message); }

// Convenience wrapper over write() that also records that an error happened.
void Logger::error(std::string_view code, std::string_view message) {
    write(LogLevel::Error, code, message);
}

// Reports whether error() was ever called on this logger.
bool Logger::hasErrors() const {
    const std::lock_guard<std::mutex> lock{mutex_};
    return has_errors_;
}

// Backing file path (empty for stderr-only loggers).
const std::filesystem::path& Logger::path() const { return path_; }

// The single place that formats and emits a line; holds the mutex for the whole operation
// so a line from one mission thread can never be spliced into a line from another.
void Logger::write(LogLevel level, std::string_view code, std::string_view message) {
    std::string line;
    line.reserve(message.size() + code.size() + 48);
    line += utcTimestamp();
    line += ' ';
    line += levelText(level);
    if (!code.empty()) {
        line += " [";
        line += code;
        line += ']';
    }
    line += ' ';
    line += message;
    line += '\n';

    const std::lock_guard<std::mutex> lock{mutex_};
    if (level == LogLevel::Error) { has_errors_ = true; }
    if (stream_.is_open()) {
        stream_ << line;
        stream_.flush(); // flush per line: a crash must not lose the last error
    }
    if (echo_to_stderr_) {
        std::cerr << line;
    }
}

} // namespace user_common_213309941_213727837
