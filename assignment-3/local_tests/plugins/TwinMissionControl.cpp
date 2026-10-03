// Twin of our submission MissionControl: same DroneControl, different .so name.
// Used to check comparative same_results grouping.

#include <MissionControl/DroneControlImpl.h>
#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>
#include <Common/MissionControlRegistration.h>
#include <UserCommon/Logger.h>
#include <UserCommon/TimeUtils.h>

#include <filesystem>
#include <memory>
#include <stdexcept>
#include <utility>

namespace local_tests {

namespace ucm = user_common_213309941_213727837;
using mission_control_213309941_213727837::DroneControlDependencies;
using mission_control_213309941_213727837::DroneControlImpl;

class TwinMissionControl final : public common::IMissionControl {
public:
    explicit TwinMissionControl(common::MissionControlDependencies dependencies)
        : mission_(dependencies.mission_config),
          drone_(dependencies.drone_config),
          output_map_(&dependencies.output_map),
          output_map_file_(std::move(dependencies.output_map_file)),
          logger_(output_map_file_.parent_path() /
                  ("error_log_" + ucm::sanitizeForFilename(output_map_file_.stem().string()) + ".txt")) {
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

    common::types::MissionRunResult runMission() override {
        common::types::MissionRunResult result;
        result.status = common::types::MissionRunStatus::Completed;
        for (std::size_t step = 0; step < mission_.max_steps; ++step) {
            common::types::DroneStepResult step_result;
            try {
                step_result = drone_control_->step();
            } catch (const std::exception& e) {
                result.errors.push_back({"DRONE_STEP_EXCEPTION", e.what()});
                result.status = common::types::MissionRunStatus::Error;
                break;
            }
            ++result.steps;
            if (step_result.status == common::types::DroneStepStatus::Completed) {
                result.status = common::types::MissionRunStatus::Completed;
                break;
            }
            if (step_result.status == common::types::DroneStepStatus::Error) {
                result.errors.push_back({"DRONE_STEP_ERROR", step_result.message});
                result.status = common::types::MissionRunStatus::Error;
                continue;
            }
            if (step == mission_.max_steps - 1) {
                result.status = common::types::MissionRunStatus::MaxSteps;
            }
        }
        for (const auto& err : drone_control_->errors()) { result.errors.push_back(err); }
        try {
            std::filesystem::create_directories(output_map_file_.parent_path());
            output_map_->save(output_map_file_);
        } catch (const std::exception& e) {
            result.errors.push_back({"MAP_SAVE_ERROR", e.what()});
            result.status = common::types::MissionRunStatus::Error;
        }
        return result;
    }

private:
    common::types::MissionConfigData mission_{};
    common::types::DroneConfigData drone_{};
    common::IMutableMap3D* output_map_ = nullptr;
    std::filesystem::path output_map_file_{};
    ucm::Logger logger_;
    std::unique_ptr<DroneControlImpl> drone_control_;
};

} // namespace local_tests

using TwinMissionControl_local = local_tests::TwinMissionControl;
REGISTER_MISSION_CONTROL(TwinMissionControl_local);
