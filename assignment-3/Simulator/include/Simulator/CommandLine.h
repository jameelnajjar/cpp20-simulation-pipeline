#pragma once

// CommandLine.h - argv parser for the two simulator modes.

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace simulator {

enum class RunMode { Comparative, Competitive };

struct CommandLine {
    RunMode mode = RunMode::Comparative;
    std::filesystem::path simulation{};
    std::filesystem::path mission_control_folder{};
    std::filesystem::path algorithm{};
    std::filesystem::path mission_control{};
    std::filesystem::path algorithms_folder{};
    int num_threads = 1;   // missing or 1 => main thread only
    bool verbose = false;

    // Returns nullopt and prints usage on any error (unsupported / missing / unreadable).
    [[nodiscard]] static std::optional<CommandLine> parse(int argc, char** argv);
};

// Lists regular files with a `.so` extension in `folder` (used to discover plugins).
[[nodiscard]] std::vector<std::filesystem::path> listSharedObjects(
    const std::filesystem::path& folder);

} // namespace simulator
