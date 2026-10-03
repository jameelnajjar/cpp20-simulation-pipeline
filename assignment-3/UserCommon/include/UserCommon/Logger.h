#pragma once

// Logger.h - a small thread-safe append-only text logger.
// Lives in UserCommon because all three projects need to write log files:
// the Simulator writes simulation-level errors, the MissionControl writes mission
// error logs and (with -verbose) step traces, and the Algorithm may log its own errors.
// It is deliberately NOT a singleton: the Simulator runs many missions in parallel and
// each mission owns its own Logger, so no cross-thread ordering problems can appear.

#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <string_view>

namespace user_common_213309941_213727837 {

// Severity tag written at the start of every line. Plain scoped enum, not derived.
enum class LogLevel {
    Info,    // progress / verbose trace
    Warning, // recoverable problem, execution continues
    Error,   // problem that produced an ErrorRef or aborted a mission
};

class Logger {
public:
    // Constructs a logger that only echoes to std::cerr (no file backing).
    Logger();

    // Constructs a logger backed by `path`; parent directories are created on demand.
    // If the file cannot be opened the logger degrades to stderr-only instead of throwing,
    // because a failing log must never take down a simulation run.
    explicit Logger(std::filesystem::path path);

    // Flushes and closes the backing stream.
    ~Logger();

    // A Logger owns a std::mutex, which is neither copyable nor movable.
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    Logger(Logger&&) = delete;
    Logger& operator=(Logger&&) = delete;
    // Switches stderr echoing on/off. Off is the default for file-backed loggers so that
    // dozens of parallel missions do not interleave noise on the console.
    void setEchoToStderr(bool echo);

    // Writes "<utc> INFO <message>".
    void info(std::string_view message);

    // Writes "<utc> WARNING <message>".
    void warning(std::string_view message);

    // Writes "<utc> ERROR [<code>] <message>"; `code` matches common::types::ErrorRef::code.
    void error(std::string_view code, std::string_view message);

    // True when at least one error() call has been made; lets a caller decide whether an
    // otherwise-empty log file is worth keeping.
    [[nodiscard]] bool hasErrors() const;

    // Absolute path of the backing file, or an empty path for stderr-only loggers.
    [[nodiscard]] const std::filesystem::path& path() const;

private:
    // Single funnel used by info()/warning()/error(); takes the mutex and flushes.
    void write(LogLevel level, std::string_view code, std::string_view message);

    mutable std::mutex mutex_;      // serialises writes from concurrent mission threads
    std::filesystem::path path_;    // backing file, empty when stderr-only
    std::ofstream stream_;          // append-mode handle to path_
    bool echo_to_stderr_ = true;    // mirror every line to std::cerr
    bool has_errors_ = false;       // set by the first error() call
};

} // namespace user_common_213309941_213727837
