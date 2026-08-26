#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/ScanResultToVoxels.h>

#include <cmath>

namespace drone_mapper {

namespace {
    double toCm(PhysicalLength v)   { return v.numerical_value_in(cm); }
    double toDeg(HorizontalAngle v) { return v.numerical_value_in(deg); }
} // namespace

DroneControlImpl::DroneControlImpl(types::DroneConfigData drone,
                                   types::MissionConfigData mission,
                                   ILidar& lidar,
                                   IGPS& gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

bool DroneControlImpl::isInMissionBounds(const Position3D& pos) const {
    return output_map_.isInBounds(pos);
}

types::MovementResult DroneControlImpl::executeMovement(const types::MovementCommand& cmd) {
    switch (cmd.type) {
        case types::MovementCommandType::Hover:
            return types::MovementResult{true, {}};

        case types::MovementCommandType::Rotate: {
            const double req = std::abs(toDeg(cmd.angle));
            const double max = std::abs(toDeg(drone_.max_rotate));
            if (req > max + 0.001) {
                return types::MovementResult{false, "ROTATION_EXCEEDS_MAX"};
            }
            return movement_.rotate(cmd.rotation, cmd.angle);
        }

        case types::MovementCommandType::Advance: {
            const double req = std::abs(toCm(cmd.distance));
            const double max = std::abs(toCm(drone_.max_advance));
            if (req > max + 0.001) {
                return types::MovementResult{false, "ADVANCE_EXCEEDS_MAX"};
            }
            return movement_.advance(cmd.distance);
        }

        case types::MovementCommandType::Elevate: {
            const double req = std::abs(toCm(cmd.distance));
            const double max = std::abs(toCm(drone_.max_elevate));
            if (req > max + 0.001) {
                return types::MovementResult{false, "ELEVATE_EXCEEDS_MAX"};
            }
            return movement_.elevate(cmd.distance);
        }
    }
    return types::MovementResult{true, {}};
}

types::DroneStepResult DroneControlImpl::step() {
    const types::LidarScanResult* scan_ptr = latest_scan_.has_value() ? &latest_scan_.value() : nullptr;
    const types::DroneState current_state = state();

    const types::MappingStepCommand cmd = mapping_algorithm_.nextStep(current_state, scan_ptr);

    if (cmd.status == types::AlgorithmStatus::Finished ||
        cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
        return types::DroneStepResult{types::DroneStepStatus::Completed, "Mission mapping complete"};
    }

    // Execute movement if requested
    if (cmd.movement.has_value()) {
        types::MovementResult result = executeMovement(cmd.movement.value());
        if (!result.success) {
            // Log and continue - non-fatal movement failure
            latest_scan_.reset();
            ++step_index_;
            // Don't hard-fail on movement errors - algorithm will adapt
        }
    }

    // Execute scan if requested
    if (cmd.scan_orientation.has_value()) {
        latest_scan_ = lidar_.scan(cmd.scan_orientation.value());

        // Apply scan results to the output map
        const types::DroneState post_state = state();
        ScanResultToVoxels::applyToMap(output_map_,
                                        post_state.position,
                                        post_state.heading,
                                        latest_scan_.value(),
                                        lidar_.config());
    } else {
        latest_scan_.reset();
    }

    ++step_index_;
    return types::DroneStepResult{types::DroneStepStatus::Continue, {}};
}

types::DroneState DroneControlImpl::state() const {
    return types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

} // namespace drone_mapper
