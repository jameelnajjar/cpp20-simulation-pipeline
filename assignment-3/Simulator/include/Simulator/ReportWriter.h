#pragma once

// ReportWriter.h - YAML reports required by assignment 2 (per plugin) and assignment 3
// (comparative / competitive summaries).

#include <Simulator/SimulationTypes.h>

#include <filesystem>
#include <string>
#include <vector>

namespace simulator {

struct PluginBatchResult {
    std::string plugin_filename{};
    types::SimulationManagerReport report{};
    bool loaded = true;
    std::string error{};
};

void writeAssignment2Report(const types::SimulationManagerReport& report,
                            const std::filesystem::path& file,
                            const std::string& plugin_name);

void writeComparativeReport(const std::filesystem::path& composition_file,
                            const std::filesystem::path& mission_control_folder,
                            const std::vector<PluginBatchResult>& batches,
                            const std::filesystem::path& file);

void writeCompetitiveReport(const std::filesystem::path& composition_file,
                            const std::filesystem::path& mission_control_file,
                            const std::vector<PluginBatchResult>& batches,
                            const std::filesystem::path& file);

} // namespace simulator
