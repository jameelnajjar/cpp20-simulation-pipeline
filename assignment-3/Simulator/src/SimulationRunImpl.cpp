// SimulationRunImpl.cpp - runs one mission and scores the output map.

#include <Simulator/SimulationRunImpl.h>
#include <Simulator/MapsComparison.h>
#include <UserCommon/GeometryUtils.h>

#include <stdexcept>

namespace simulator {

namespace ucm = user_common_213309941_213727837;

SimulationRunImpl::SimulationRunImpl(std::unique_ptr<common::IMutableMap3D> hidden_map,
                                     std::unique_ptr<common::IMutableMap3D> output_map,
                                     std::unique_ptr<common::IGPS> gps,
                                     std::unique_ptr<common::IDroneMovement> movement,
                                     std::unique_ptr<common::ILidar> lidar,
                                     std::unique_ptr<common::IMappingAlgorithm> mapping_algorithm,
                                     std::unique_ptr<common::IMissionControl> mission_control,
                                     types::SimulationConfigData simulation_config,
                                     common::types::MissionConfigData mission_config,
                                     std::filesystem::path output_map_file)
    : hidden_map_(std::move(hidden_map)),
      output_map_(std::move(output_map)),
      gps_(std::move(gps)),
      movement_(std::move(movement)),
      lidar_(std::move(lidar)),
      mapping_algorithm_(std::move(mapping_algorithm)),
      mission_control_(std::move(mission_control)),
      simulation_config_(std::move(simulation_config)),
      mission_config_(std::move(mission_config)),
      output_map_file_(std::move(output_map_file)) {
    if (!hidden_map_ || !output_map_ || !gps_ || !movement_ || !lidar_ || !mapping_algorithm_ ||
        !mission_control_) {
        throw std::invalid_argument("SimulationRunImpl requires all injected dependencies.");
    }
}

types::SimulationResult SimulationRunImpl::run() {
    types::SimulationResult result;
    result.simulation_config = simulation_config_;
    result.mission_config = mission_config_;
    result.output_map_file = output_map_file_;
    result.output_map_config = output_map_->getMapConfig();

    const double factor = mission_config_.output_mapping_resolution_factor;
    const double base_res = ucm::cmOf(simulation_config_.map_resolution);
    const double gps_res = ucm::cmOf(mission_config_.gps_resolution);
    if (factor <= 0.0) {
        result.resolution_request_status = types::ResolutionRequestStatus::Ignored;
    } else if ((gps_res * factor) < base_res - 1e-9) {
        result.resolution_request_status = types::ResolutionRequestStatus::IgnoredTooSmall;
    } else {
        result.resolution_request_status = types::ResolutionRequestStatus::Accepted;
    }

    try {
        const common::types::MissionRunResult mission_result = mission_control_->runMission();
        result.mission_results.push_back(mission_result);
        if (mission_result.status == common::types::MissionRunStatus::Error) {
            result.mission_score = -1.0;
            return result;
        }
        const std::vector<const common::IMap3D*> targets{output_map_.get()};
        const auto scores = MapsComparison::compare(*hidden_map_, targets);
        result.mission_score = scores.empty() ? -1.0 : scores[0];
    } catch (const std::exception& e) {
        common::types::MissionRunResult error_result;
        error_result.status = common::types::MissionRunStatus::Error;
        error_result.errors.push_back({"SIMULATION_RUN_ERROR", e.what()});
        result.mission_results.push_back(error_result);
        result.mission_score = -1.0;
    }
    return result;
}

} // namespace simulator
