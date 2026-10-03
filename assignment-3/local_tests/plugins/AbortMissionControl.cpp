#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>
#include <Common/MissionControlRegistration.h>
#include <Common/IMutableMap3D.h>

#include <utility>

namespace local_tests {

class AbortMissionControl final : public common::IMissionControl {
public:
    explicit AbortMissionControl(common::MissionControlDependencies dependencies)
        : output_map_(&dependencies.output_map),
          output_map_file_(std::move(dependencies.output_map_file)) {}

    common::types::MissionRunResult runMission() override {
        common::types::MissionRunResult result;
        result.status = common::types::MissionRunStatus::Error;
        result.errors.push_back({"ABORT_MC", "test mission control aborts immediately"});
        try {
            output_map_->save(output_map_file_);
        } catch (...) {
        }
        return result;
    }

private:
    common::IMutableMap3D* output_map_ = nullptr;
    std::filesystem::path output_map_file_{};
};

} // namespace local_tests

using AbortMissionControl_local = local_tests::AbortMissionControl;
REGISTER_MISSION_CONTROL(AbortMissionControl_local);
