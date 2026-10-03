#pragma once

// SimulationRunFactoryImpl.h - wires mocks + plugin factories into one ISimulationRun.
// Implements course-owned simulator::ISimulationRunFactory. Factories are captured at
// construction so create() is thread-safe (only an atomic counter is mutated).

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>
#include <Simulator/ISimulationRunFactory.h>

#include <atomic>
#include <string>

namespace simulator {

class SimulationRunFactoryImpl final : public ISimulationRunFactory {
public:
    SimulationRunFactoryImpl(common::MappingAlgorithmFactory algorithm_factory,
                             common::MissionControlFactory mission_control_factory,
                             std::string algorithm_label,
                             std::string mission_control_label,
                             bool verbose);

    [[nodiscard]] std::unique_ptr<ISimulationRun> create(
        const types::SimulationConfigData& simulation_config,
        const common::types::MissionConfigData& mission_config,
        const common::types::DroneConfigData& drone_config,
        const common::types::LidarConfigData& lidar_config,
        const std::filesystem::path& output_path) override;

private:
    common::MappingAlgorithmFactory algorithm_factory_;
    common::MissionControlFactory mission_control_factory_;
    std::string algorithm_label_;
    std::string mission_control_label_;
    bool verbose_ = false;
    std::atomic<int> run_counter_{0}; // unique output-map filenames across threads
};

} // namespace simulator
