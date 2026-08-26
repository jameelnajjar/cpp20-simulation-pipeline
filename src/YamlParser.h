#pragma once

#include <drone_mapper/Types.h>

#include <filesystem>

namespace drone_mapper {

class YamlParser {
public:
    [[nodiscard]] static types::DroneConfigData parseDroneConfig(const std::filesystem::path& path);
    [[nodiscard]] static types::LidarConfigData parseLidarConfig(const std::filesystem::path& path);
    [[nodiscard]] static types::MissionConfigData parseMissionConfig(const std::filesystem::path& path);
    [[nodiscard]] static types::SimulationConfigData parseSimulationConfig(const std::filesystem::path& path);
    [[nodiscard]] static types::SimulationCompositionData parseComposition(const std::filesystem::path& path);
};

} // namespace drone_mapper
