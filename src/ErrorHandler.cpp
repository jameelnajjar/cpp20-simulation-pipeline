#include "ErrorHandler.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace drone_mapper {

ErrorHandler& ErrorHandler::instance() {
    static ErrorHandler inst;
    return inst;
}

void ErrorHandler::setLogFile(const std::filesystem::path& path) {
    std::filesystem::create_directories(path.parent_path());
    log_file_.open(path, std::ios::app);
}

namespace {
std::string timestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&t), "[%Y-%m-%d %H:%M:%S]");
    return oss.str();
}
} // namespace

void ErrorHandler::logError(const std::string& code, const std::string& message) {
    const std::string line = timestamp() + " ERROR [" + code + "] " + message + "\n";
    if (log_file_.is_open()) {
        log_file_ << line;
        log_file_.flush();
    }
    std::cerr << line;
}

void ErrorHandler::logWarning(const std::string& message) {
    const std::string line = timestamp() + " WARN " + message + "\n";
    if (log_file_.is_open()) {
        log_file_ << line;
        log_file_.flush();
    }
    std::cerr << line;
}

} // namespace drone_mapper
