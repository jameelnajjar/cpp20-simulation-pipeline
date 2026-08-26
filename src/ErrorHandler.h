#pragma once

#include <filesystem>
#include <fstream>
#include <string>

namespace drone_mapper {

// Singleton error logger. Immediately flushes each entry to the log file.
class ErrorHandler {
public:
    static ErrorHandler& instance();

    void setLogFile(const std::filesystem::path& path);

    void logError(const std::string& code, const std::string& message);
    void logWarning(const std::string& message);

    ErrorHandler(const ErrorHandler&) = delete;
    ErrorHandler& operator=(const ErrorHandler&) = delete;

private:
    ErrorHandler() = default;
    std::ofstream log_file_;
};

} // namespace drone_mapper
