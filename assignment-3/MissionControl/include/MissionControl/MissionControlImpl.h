#pragma once

// MissionControlImpl.h - concrete common::IMissionControl, compiled into the MC .so.
// Creates its own DroneControl (MissionControlDependencies comment: "mission will
// create its own drone controller").

#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>
#include <MissionControl/DroneControlImpl.h>
#include <UserCommon/Logger.h>

#include <filesystem>
#include <memory>

namespace mission_control_213309941_213727837 {

class MissionControlImpl final : public common::IMissionControl {
public:
    explicit MissionControlImpl(common::MissionControlDependencies dependencies);

    [[nodiscard]] common::types::MissionRunResult runMission() override;

private:
    common::types::MissionConfigData mission_{};
    common::types::DroneConfigData drone_{};
    common::IMutableMap3D* output_map_ = nullptr;
    std::filesystem::path output_map_file_{};
    bool verbose_ = false;

    user_common_213309941_213727837::Logger logger_;
    std::unique_ptr<user_common_213309941_213727837::Logger> verbose_logger_;
    std::unique_ptr<DroneControlImpl> drone_control_;
};

} // namespace mission_control_213309941_213727837
