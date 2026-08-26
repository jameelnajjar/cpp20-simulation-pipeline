#include <drone_mapper/SimulationManager.h>

#include "ErrorHandler.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace drone_mapper {

namespace {
std::string currentUtcTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << std::put_time(std::gmtime(&t), "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}
} // namespace

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory)
    : run_factory_(std::move(run_factory)) {
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
}

types::SimulationManagerReport SimulationManager::run(const types::SimulationCompositionData& composition,
                                                       const std::filesystem::path& output_path) {
    std::vector<types::SimulationResult> runs;

    for (const auto& [simulation, missions] : composition.simulation_mission_groups) {
        for (const types::MissionConfigData& mission : missions) {
            for (const types::DroneConfigData& drone : composition.drones) {
                for (const types::LidarConfigData& lidar : composition.lidars) {
                    try {
                        std::unique_ptr<ISimulationRun> run =
                            run_factory_->create(simulation, mission, drone, lidar, output_path);
                        runs.push_back(run->run());
                    } catch (const std::exception& e) {
                        ErrorHandler::instance().logError("RUN_CREATE_ERROR", e.what());
                        types::SimulationResult error_result;
                        error_result.simulation_config = simulation;
                        error_result.mission_config = mission;
                        error_result.mission_score = -1.0;
                        types::MissionRunResult mr;
                        mr.status = types::MissionRunStatus::Error;
                        mr.errors.push_back({"RUN_CREATE_ERROR", e.what()});
                        error_result.mission_results.push_back(mr);
                        runs.push_back(error_result);
                    }
                }
            }
        }
    }

    return types::SimulationManagerReport{
        currentUtcTimestamp(),
        "output_map_accuracy",
        {0.0, 100.0},
        -1,
        std::move(runs),
    };
}

} // namespace drone_mapper
