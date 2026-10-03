// MissionControlRegistration.cpp - definition of the course-owned registration
// constructor. Compiled ONLY into the Simulator executable (assignment rule).

#include <Common/MissionControlRegistration.h>
#include <Simulator/Registrar.h>

namespace common {

MissionControlRegistration::MissionControlRegistration(MissionControlFactory factory) {
    simulator::Registrar::instance().addMissionControl(std::move(factory));
}

} // namespace common
