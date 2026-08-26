#include <drone_mapper/MissionControlImpl.h>

#include "ErrorHandler.h"

#include <filesystem>

namespace drone_mapper {

MissionControlImpl::MissionControlImpl(types::MissionConfigData mission,
                                       types::DroneConfigData drone,
                                       const IMap3D& hidden_map,
                                       IMutableMap3D& output_map,
                                       IDroneControl& drone_control,
                                       std::filesystem::path output_map_file)
    : mission_(std::move(mission)),
      drone_(std::move(drone)),
      hidden_map_(hidden_map),
      output_map_(output_map),
      drone_control_(drone_control),
      output_map_file_(std::move(output_map_file)) {}

types::MissionRunResult MissionControlImpl::runMission() {
    types::MissionRunResult result;
    result.status = types::MissionRunStatus::Completed;
    result.steps = 0;

    for (std::size_t step = 0; step < mission_.max_steps; ++step) {
        types::DroneStepResult step_result = drone_control_.step();
        ++result.steps;

        if (step_result.status == types::DroneStepStatus::Completed) {
            result.status = types::MissionRunStatus::Completed;
            break;
        }

        if (step_result.status == types::DroneStepStatus::Error) {
            const types::ErrorRef err{"DRONE_STEP_ERROR", step_result.message};
            result.errors.push_back(err);
            ErrorHandler::instance().logError(err.code, err.message);
            result.status = types::MissionRunStatus::Error;
            break;
        }

        if (step == mission_.max_steps - 1) {
            result.status = types::MissionRunStatus::MaxSteps;
        }
    }

    // Save output map regardless of status
    try {
        std::filesystem::create_directories(output_map_file_.parent_path());
        output_map_.save(output_map_file_);
    } catch (const std::exception& e) {
        const types::ErrorRef err{"MAP_SAVE_ERROR", e.what()};
        result.errors.push_back(err);
        ErrorHandler::instance().logError(err.code, err.message);
        result.status = types::MissionRunStatus::Error;
    }

    return result;
}

} // namespace drone_mapper
