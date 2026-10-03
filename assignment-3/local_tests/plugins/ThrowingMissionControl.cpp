#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>
#include <Common/MissionControlRegistration.h>

#include <stdexcept>

namespace local_tests {

class ThrowingMissionControl final : public common::IMissionControl {
public:
    explicit ThrowingMissionControl(common::MissionControlDependencies) {}

    common::types::MissionRunResult runMission() override {
        throw std::runtime_error("MC_THROW: test mission control throws from runMission");
    }
};

} // namespace local_tests

using ThrowingMissionControl_local = local_tests::ThrowingMissionControl;
REGISTER_MISSION_CONTROL(ThrowingMissionControl_local);
