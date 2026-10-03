// SimulationManager.cpp - cartesian product over the composition (assignment 2 manager).

#include <Simulator/SimulationManager.h>
#include <UserCommon/TimeUtils.h>

#include <stdexcept>

namespace simulator {

namespace ucm = user_common_213309941_213727837;

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory)
    : run_factory_(std::move(run_factory)) {
    if (!run_factory_) { throw std::invalid_argument("SimulationManager requires a run factory."); }
}

types::SimulationManagerReport SimulationManager::run(
    const types::SimulationCompositionData& composition, const std::filesystem::path& output_path) {
    // Cartesian product: simulations × missions × drones × lidars, one ISimulationRun each.
    std::vector<types::SimulationResult> runs;
    for (const auto& [simulation, missions] : composition.simulation_mission_groups) {
        for (const auto& mission : missions) {
            for (const auto& drone : composition.drone_configs) {
                for (const auto& lidar : composition.lidar_configs) {
                    try {
                        auto run = run_factory_->create(simulation, mission, drone, lidar, output_path);
                        runs.push_back(run->run());
                    } catch (const std::exception& e) {
                        types::SimulationResult error_result;
                        error_result.simulation_config = simulation;
                        error_result.mission_config = mission;
                        error_result.mission_score = -1.0;
                        common::types::MissionRunResult mr;
                        mr.status = common::types::MissionRunStatus::Error;
                        mr.errors.push_back({"RUN_CREATE_ERROR", e.what()});
                        error_result.mission_results.push_back(mr);
                        runs.push_back(error_result);
                    }
                }
            }
        }
    }
    return types::SimulationManagerReport{
        composition.composition_file, ucm::utcTimestamp(), "output_map_accuracy", {0.0, 100.0}, -1,
        std::move(runs)};
}

} // namespace simulator
