#pragma once

// YamlParser.h - composition / config YAML readers. Simulator-owned because the
// Simulator is the process that receives the composition file on the command line.

#include <Simulator/SimulationTypes.h>

#include <filesystem>

namespace simulator {

class YamlParser {
public:
    [[nodiscard]] static common::types::DroneConfigData parseDroneConfig(
        const std::filesystem::path& path);
    [[nodiscard]] static common::types::LidarConfigData parseLidarConfig(
        const std::filesystem::path& path);
    [[nodiscard]] static common::types::MissionConfigData parseMissionConfig(
        const std::filesystem::path& path);
    [[nodiscard]] static types::SimulationConfigData parseSimulationConfig(
        const std::filesystem::path& path);
    [[nodiscard]] static types::SimulationCompositionData parseComposition(
        const std::filesystem::path& path);
};

} // namespace simulator
