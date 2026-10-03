#pragma once

// Shared boilerplate for local test algorithms. Not part of the course zip layout.

#include <Common/IMappingAlgorithm.h>
#include <Common/MappingAlgorithmRegistration.h>
#include <Common/types/DroneTypes.h>
#include <UserCommon/GeometryUtils.h>

#include <optional>

namespace local_tests {

namespace ucm = user_common_213309941_213727837;

using common::types::AlgorithmStatus;
using common::types::MappingStepCommand;
using common::types::MovementCommand;
using common::types::MovementCommandType;
using common::types::RotationDirection;

inline MappingStepCommand workingMove(MovementCommandType type, double distance_cm,
                                      double angle_deg = 0.0) {
    return MappingStepCommand{
        MovementCommand{type, RotationDirection::Left, ucm::yawDeg(angle_deg),
                        ucm::lengthCm(distance_cm)},
        std::nullopt, AlgorithmStatus::Working};
}

inline MappingStepCommand workingScan() {
    return MappingStepCommand{std::nullopt,
                              common::Orientation{ucm::yawDeg(0.0), ucm::pitchDeg(0.0)},
                              AlgorithmStatus::Working};
}

inline MappingStepCommand finished() {
    return MappingStepCommand{std::nullopt, std::nullopt, AlgorithmStatus::Finished};
}

inline MappingStepCommand noop() {
    return MappingStepCommand{std::nullopt, std::nullopt, AlgorithmStatus::Working};
}

} // namespace local_tests
