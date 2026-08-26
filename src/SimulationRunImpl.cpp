#include <drone_mapper/SimulationRunImpl.h>
#include <drone_mapper/MapsComparison.h>

#include <stdexcept>

namespace drone_mapper {

SimulationRunImpl::SimulationRunImpl(std::unique_ptr<IMutableMap3D> hidden_map,
                                     std::unique_ptr<IMutableMap3D> output_map,
                                     std::unique_ptr<IGPS> gps,
                                     std::unique_ptr<IDroneMovement> movement,
                                     std::unique_ptr<ILidar> lidar,
                                     std::unique_ptr<IMappingAlgorithm> mapping_algorithm,
                                     std::unique_ptr<IDroneControl> drone_control,
                                     std::unique_ptr<IMissionControl> mission_control,
                                     types::SimulationConfigData simulation_config,
                                     types::MissionConfigData mission_config,
                                     std::filesystem::path output_map_file)
    : hidden_map_(std::move(hidden_map)),
      output_map_(std::move(output_map)),
      gps_(std::move(gps)),
      movement_(std::move(movement)),
      lidar_(std::move(lidar)),
      mapping_algorithm_(std::move(mapping_algorithm)),
      drone_control_(std::move(drone_control)),
      mission_control_(std::move(mission_control)),
      simulation_config_(std::move(simulation_config)),
      mission_config_(std::move(mission_config)),
      output_map_file_(std::move(output_map_file)) {
    if (!hidden_map_ || !output_map_ || !gps_ || !movement_ || !lidar_ ||
        !mapping_algorithm_ || !drone_control_ || !mission_control_) {
        throw std::invalid_argument("SimulationRunImpl requires all injected dependencies.");
    }
}

types::SimulationResult SimulationRunImpl::run() {
    types::SimulationResult result;
    result.simulation_config = simulation_config_;
    result.mission_config = mission_config_;
    result.output_map_file = output_map_file_;
    result.output_map_config = output_map_->getMapConfig();

    // Determine resolution request status
    const double factor = mission_config_.output_mapping_resolution_factor;
    const double base_res = simulation_config_.map_resolution.numerical_value_in(cm);
    const double gps_res  = mission_config_.gps_resolution.numerical_value_in(cm);
    if (factor <= 0.0) {
        result.resolution_request_status = types::ResolutionRequestStatus::Ignored;
    } else if ((gps_res * factor) < base_res - 1e-9) {
        result.resolution_request_status = types::ResolutionRequestStatus::IgnoredTooSmall;
    } else {
        result.resolution_request_status = types::ResolutionRequestStatus::Accepted;
    }

    try {
        const types::MissionRunResult mission_result = mission_control_->runMission();
        result.mission_results.push_back(mission_result);

        if (mission_result.status == types::MissionRunStatus::Error) {
            result.mission_score = -1.0;
            return result;
        }

        // Compare output map against hidden map
        std::vector<IMap3D*> targets = {output_map_.get()};
        const std::vector<double> scores = MapsComparison::compare(*hidden_map_, targets);
        result.mission_score = scores.empty() ? -1.0 : scores[0];

    } catch (const std::exception& e) {
        const types::ErrorRef err{"SIMULATION_RUN_ERROR", e.what()};
        types::MissionRunResult error_result;
        error_result.status = types::MissionRunStatus::Error;
        error_result.errors.push_back(err);
        result.mission_results.push_back(error_result);
        result.mission_score = -1.0;
    }

    return result;
}

} // namespace drone_mapper
