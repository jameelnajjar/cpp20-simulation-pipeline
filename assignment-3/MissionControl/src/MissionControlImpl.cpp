// MissionControlImpl.cpp - mission loop + map save + optional verbose log.
// REGISTER_MISSION_CONTROL publishes this class when the .so is dlopen'ed.

#include <MissionControl/MissionControlImpl.h>
#include <Common/MissionControlRegistration.h>
#include <UserCommon/TimeUtils.h>

#include <filesystem>
#include <stdexcept>
#include <utility>

namespace mission_control_213309941_213727837 {

namespace ucm = user_common_213309941_213727837;

MissionControlImpl::MissionControlImpl(common::MissionControlDependencies dependencies)
    : mission_(dependencies.mission_config),
      drone_(dependencies.drone_config),
      output_map_(&dependencies.output_map),
      output_map_file_(std::move(dependencies.output_map_file)),
      verbose_(dependencies.verbose),
      logger_(output_map_file_.parent_path() /
              ("error_log_" + ucm::sanitizeForFilename(output_map_file_.stem().string()) + ".txt")) {
    if (verbose_) {
        verbose_logger_ = std::make_unique<ucm::Logger>(
            output_map_file_.parent_path() /
            ("verbose_" + ucm::sanitizeForFilename(output_map_file_.stem().string()) + ".txt"));
        verbose_logger_->info("MissionControlImpl starting");
    }

    DroneControlDependencies dc;
    dc.drone_config = drone_;
    dc.mission_config = mission_;
    dc.lidar_config = dependencies.lidar.config();
    dc.lidar = &dependencies.lidar;
    dc.gps = &dependencies.gps;
    dc.movement = &dependencies.movement;
    dc.output_map = output_map_;
    dc.algorithm = &dependencies.mapping_algorithm;
    dc.logger = &logger_;
    drone_control_ = std::make_unique<DroneControlImpl>(std::move(dc));
}

common::types::MissionRunResult MissionControlImpl::runMission() {
    common::types::MissionRunResult result;
    result.status = common::types::MissionRunStatus::Completed;

    for (std::size_t step = 0; step < mission_.max_steps; ++step) {
        common::types::DroneStepResult step_result;
        try {
            step_result = drone_control_->step();
        } catch (const std::exception& e) {
            // Wall-collision and other mandatory throws: log, mark Error, still save the map.
            result.errors.push_back({"DRONE_STEP_EXCEPTION", e.what()});
            logger_.error("DRONE_STEP_EXCEPTION", e.what());
            result.status = common::types::MissionRunStatus::Error;
            break;
        }
        ++result.steps;
        if (verbose_logger_) {
            verbose_logger_->info("step " + std::to_string(result.steps));
        }
        if (step_result.status == common::types::DroneStepStatus::Completed) {
            if (result.status != common::types::MissionRunStatus::Error) {
                result.status = common::types::MissionRunStatus::Completed;
            }
            break;
        }
        if (step_result.status == common::types::DroneStepStatus::Error) {
            // Common-issues bonus: "Log the error, continue."
            result.errors.push_back({"DRONE_STEP_ERROR", step_result.message});
            logger_.error("DRONE_STEP_ERROR", step_result.message);
            result.status = common::types::MissionRunStatus::Error;
            continue;
        }
        if (step == mission_.max_steps - 1 &&
            result.status != common::types::MissionRunStatus::Error) {
            result.status = common::types::MissionRunStatus::MaxSteps;
        }
    }

    for (const auto& err : drone_control_->errors()) { result.errors.push_back(err); }

    try {
        std::filesystem::create_directories(output_map_file_.parent_path());
        output_map_->save(output_map_file_);
    } catch (const std::exception& e) {
        result.errors.push_back({"MAP_SAVE_ERROR", e.what()});
        logger_.error("MAP_SAVE_ERROR", e.what());
        result.status = common::types::MissionRunStatus::Error;
    }
    return result;
}

} // namespace mission_control_213309941_213727837

using MissionControlImpl_213309941_213727837 =
    mission_control_213309941_213727837::MissionControlImpl;

REGISTER_MISSION_CONTROL(MissionControlImpl_213309941_213727837);
