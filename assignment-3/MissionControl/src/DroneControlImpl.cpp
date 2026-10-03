// DroneControlImpl.cpp - drives the drone and applies the Common-issues table.

#include <MissionControl/DroneControlImpl.h>
#include <MissionControl/ScanResultToVoxels.h>
#include <UserCommon/GeometryUtils.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace mission_control_213309941_213727837 {

namespace ucm = user_common_213309941_213727837;

using common::types::DroneStepResult;
using common::types::DroneStepStatus;
using common::types::MovementCommand;
using common::types::MovementCommandType;
using common::types::RotationDirection;

DroneControlImpl::DroneControlImpl(DroneControlDependencies dependencies)
    : deps_(std::move(dependencies)) {
    if (!deps_.lidar || !deps_.gps || !deps_.movement || !deps_.output_map || !deps_.algorithm) {
        throw std::invalid_argument("DroneControlImpl: missing injected collaborator");
    }
}

const std::vector<common::types::ErrorRef>& DroneControlImpl::errors() const { return errors_; }
std::size_t DroneControlImpl::splitCommandCount() const { return split_command_count_; }

void DroneControlImpl::recordError(const char* code, const std::string& message) {
    errors_.push_back({code, message});
    if (deps_.logger) { deps_.logger->error(code, message); }
}

bool DroneControlImpl::insideMissionBounds(const common::Position3D& position) const {
    return ucm::isInsideBounds(deps_.mission_config.mission_bounds, position);
}

bool DroneControlImpl::commandValuesAreValid(const MovementCommand& command) const {
    if (!std::isfinite(ucm::degOf(command.angle)) || !std::isfinite(ucm::cmOf(command.distance))) {
        return false;
    }
    if (ucm::degOf(command.angle) < 0.0) { return false; }
    return true;
}

common::Position3D DroneControlImpl::predictPosition(const MovementCommand& command) const {
    const common::Position3D here = dead_reckoned_position_;
    if (command.type == MovementCommandType::Elevate) {
        return ucm::positionCm(ucm::cmOf(here.x), ucm::cmOf(here.y),
                               ucm::cmOf(here.z) + ucm::cmOf(command.distance));
    }
    if (command.type == MovementCommandType::Advance) {
        const ucm::UnitVector3 dir =
            ucm::directionOf(common::Orientation{dead_reckoned_heading_.horizontal, {}});
        const double d = ucm::cmOf(command.distance);
        return ucm::positionCm(ucm::cmOf(here.x) + dir.x * d, ucm::cmOf(here.y) + dir.y * d,
                               ucm::cmOf(here.z));
    }
    return here;
}

std::optional<MovementCommand> DroneControlImpl::amendForBounds(const MovementCommand& command) const {
    if (command.type != MovementCommandType::Advance &&
        command.type != MovementCommandType::Elevate) {
        return command;
    }

    const auto& bounds = deps_.mission_config.mission_bounds;
    const double x = ucm::cmOf(dead_reckoned_position_.x);
    const double y = ucm::cmOf(dead_reckoned_position_.y);
    const double z = ucm::cmOf(dead_reckoned_position_.z);
    constexpr double kMinLegalCm = 0.01;

    if (command.type == MovementCommandType::Elevate) {
        const double requested = ucm::cmOf(command.distance);
        const double z_min = ucm::cmOf(bounds.min_height);
        const double z_max = ucm::cmOf(bounds.max_height);
        double allowed = requested;
        if (requested > 0.0) { allowed = std::min(requested, z_max - z); }
        else if (requested < 0.0) { allowed = std::max(requested, z_min - z); }
        if (std::abs(allowed) < kMinLegalCm) { return std::nullopt; }
        MovementCommand amended = command;
        amended.distance = ucm::lengthCm(allowed);
        return amended;
    }

    const ucm::UnitVector3 dir =
        ucm::directionOf(common::Orientation{dead_reckoned_heading_.horizontal, {}});
    const double requested = ucm::cmOf(command.distance);
    if (std::abs(requested) < kMinLegalCm) { return command; }
    const double sign = requested < 0.0 ? -1.0 : 1.0;
    const double vx = dir.x * sign;
    const double vy = dir.y * sign;

    auto axis_limit = [](double pos, double vel, double lo, double hi) {
        if (vel > 1e-12) { return (hi - pos) / vel; }
        if (vel < -1e-12) { return (lo - pos) / vel; }
        return std::numeric_limits<double>::infinity();
    };
    const double t = std::min(axis_limit(x, vx, ucm::cmOf(bounds.min_x), ucm::cmOf(bounds.max_x)),
                              axis_limit(y, vy, ucm::cmOf(bounds.min_y), ucm::cmOf(bounds.max_y)));
    if (!(t > kMinLegalCm)) { return std::nullopt; }
    MovementCommand amended = command;
    amended.distance = ucm::lengthCm(sign * std::min(std::abs(requested), t));
    return amended;
}

std::deque<MovementCommand> DroneControlImpl::splitCommand(const MovementCommand& command) const {
    std::deque<MovementCommand> pieces;
    const double max_rotate = std::abs(ucm::degOf(deps_.drone_config.max_rotate));
    const double max_advance = std::abs(ucm::cmOf(deps_.drone_config.max_advance));
    const double max_elevate = std::abs(ucm::cmOf(deps_.drone_config.max_elevate));

    auto push_chunks = [&](MovementCommand piece, double remaining, double max_abs) {
        if (!(max_abs > 0.0)) { return; }
        const double sign = remaining < 0.0 ? -1.0 : 1.0;
        double left = std::abs(remaining);
        while (left > 1e-6) {
            if (pieces.size() >= kMaxSplitPieces) { return; }
            const double chunk = std::min(left, max_abs);
            if (piece.type == MovementCommandType::Rotate) {
                piece.angle = ucm::yawDeg(chunk);
                piece.distance = ucm::lengthCm(0.0);
            } else {
                piece.distance = ucm::lengthCm(sign * chunk);
            }
            pieces.push_back(piece);
            left -= chunk;
        }
    };

    switch (command.type) {
        case MovementCommandType::Hover:
            pieces.push_back(command);
            break;
        case MovementCommandType::Rotate:
            if (ucm::degOf(command.angle) <= max_rotate + 1e-6) { pieces.push_back(command); }
            else { push_chunks(command, ucm::degOf(command.angle), max_rotate); }
            break;
        case MovementCommandType::Advance:
            if (std::abs(ucm::cmOf(command.distance)) <= max_advance + 1e-6) { pieces.push_back(command); }
            else { push_chunks(command, ucm::cmOf(command.distance), max_advance); }
            break;
        case MovementCommandType::Elevate:
            if (std::abs(ucm::cmOf(command.distance)) <= max_elevate + 1e-6) { pieces.push_back(command); }
            else { push_chunks(command, ucm::cmOf(command.distance), max_elevate); }
            break;
    }
    return pieces;
}

std::deque<DroneControlImpl::PendingAction> DroneControlImpl::planActions(
    const common::types::MappingStepCommand& command) const {
    std::deque<PendingAction> actions;
    if (!command.movement.has_value() && !command.scan_orientation.has_value()) { return actions; }

    if (!command.movement.has_value()) {
        actions.push_back(PendingAction{std::nullopt, command.scan_orientation});
        return actions;
    }

    const MovementCommand& raw = *command.movement;
    if (!commandValuesAreValid(raw)) { return actions; }

    const auto pieces = splitCommand(raw);
    if (pieces.empty()) { return actions; }
    for (std::size_t i = 0; i < pieces.size(); ++i) {
        PendingAction action;
        action.movement = pieces[i];
        if (i + 1 == pieces.size()) { action.scan = command.scan_orientation; }
        actions.push_back(action);
    }
    return actions;
}

void DroneControlImpl::applyToDeadReckoning(const MovementCommand& command) {
    dead_reckoned_position_ = predictPosition(command);
    if (command.type == MovementCommandType::Rotate) {
        const common::HorizontalAngle signed_angle =
            (command.rotation == RotationDirection::Left) ? command.angle : -command.angle;
        dead_reckoned_heading_.horizontal += signed_angle;
    }
}

common::types::MovementResult DroneControlImpl::driveMovement(const MovementCommand& command) {
    common::types::MovementResult result{true, {}};
    for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
        switch (command.type) {
            case MovementCommandType::Hover: result = {true, {}}; break;
            case MovementCommandType::Rotate:
                result = deps_.movement->rotate(command.rotation, command.angle); break;
            case MovementCommandType::Advance:
                result = deps_.movement->advance(command.distance); break;
            case MovementCommandType::Elevate:
                result = deps_.movement->elevate(command.distance); break;
        }
        if (result) {
            movement_failure_strikes_ = 0;
            return result;
        }
        ++movement_failure_strikes_;
    }
    throw std::runtime_error("MOVEMENT_DRIVER_FAILED: " + result.message);
}

void DroneControlImpl::performScan(const common::Orientation& relative_direction) {
    common::types::LidarScanResult scan;
    for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
        scan = deps_.lidar->scan(relative_direction);
        if (!scan.empty()) {
            empty_scan_strikes_ = 0;
            latest_scan_ = scan;
            ScanResultToVoxels::applyToMap(*deps_.output_map, dead_reckoned_position_,
                                           dead_reckoned_heading_, scan, deps_.lidar->config());
            return;
        }
        ++empty_scan_strikes_;
    }
    throw std::runtime_error("EMPTY_LIDAR_SCAN");
}

bool DroneControlImpl::gpsAgreesWithDeadReckoning(const common::Position3D& gps_pos,
                                                  const common::Orientation& gps_heading) const {
    const double gps_res = std::max(1.0, ucm::cmOf(deps_.mission_config.gps_resolution));
    if (ucm::distanceCm(gps_pos, dead_reckoned_position_) > gps_res * 2.0 + 1.0) {
        return false;
    }
    if (std::abs(ucm::angleDiffDeg(ucm::degOf(gps_heading.horizontal),
                                   ucm::degOf(dead_reckoned_heading_.horizontal))) > 20.0) {
        return false;
    }
    if (std::abs(ucm::degOf(gps_heading.altitude) -
                 ucm::degOf(dead_reckoned_heading_.altitude)) > 20.0) {
        return false;
    }
    return true;
}

bool DroneControlImpl::refreshPose() {
    for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
        const common::Position3D gps_pos = deps_.gps->position();
        const common::Orientation gps_hdg = deps_.gps->heading();
        if (!ucm::isFinitePosition(gps_pos) || !ucm::isFiniteOrientation(gps_hdg)) { continue; }
        const bool gps_oob = !insideMissionBounds(gps_pos);
        const bool dead_oob = !insideMissionBounds(dead_reckoned_position_);
        if (gps_oob && dead_oob) {
            throw std::runtime_error("GPS_AND_DEAD_RECKONING_OUT_OF_BOUNDS");
        }
        if (gps_oob) { return true; } // row 11: ignore GPS, keep dead reckoning
        if (!gpsAgreesWithDeadReckoning(gps_pos, gps_hdg)) {
            continue; // row 12: re-read N times
        }
        dead_reckoned_position_ = gps_pos;
        dead_reckoned_heading_ = gps_hdg;
        return true;
    }
    return false;
}

DroneStepResult DroneControlImpl::executeAction(const PendingAction& action) {
    if (action.movement.has_value()) {
        auto amended = amendForBounds(*action.movement);
        if (!amended.has_value()) {
            recordError("ILLEGAL_MOVEMENT_IGNORED", "movement would leave mission bounds");
        } else {
            (void)driveMovement(*amended);
            applyToDeadReckoning(*amended);
            if (!refreshPose()) {
                return {DroneStepStatus::Error, "GPS_REFRESH_FAILED"};
            }
        }
    }
    if (action.scan.has_value()) { performScan(*action.scan); }
    else { latest_scan_.reset(); }
    ++step_index_;
    return {DroneStepStatus::Continue, {}};
}

DroneStepResult DroneControlImpl::step() {
    if (!pose_initialised_) {
        dead_reckoned_position_ = deps_.gps->position();
        dead_reckoned_heading_ = deps_.gps->heading();
        pose_initialised_ = true;
        if (!insideMissionBounds(dead_reckoned_position_)) {
            throw std::runtime_error("INITIAL_GPS_OUT_OF_BOUNDS");
        }
    }

    if (!pending_actions_.empty()) {
        const PendingAction leftover = pending_actions_.front();
        pending_actions_.pop_front();
        return executeAction(leftover);
    }

    const common::types::LidarScanResult* scan_ptr =
        latest_scan_.has_value() ? &latest_scan_.value() : nullptr;
    const auto command = deps_.algorithm->nextStep(state(), scan_ptr);

    if (command.status == common::types::AlgorithmStatus::Finished ||
        command.status == common::types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
        return {DroneStepStatus::Completed, "Mission mapping complete"};
    }

    auto actions = planActions(command);
    if (actions.empty()) {
        if (!command.movement.has_value() && !command.scan_orientation.has_value()) {
            if (++noop_strikes_ >= kMaxRetries) { throw std::runtime_error("REPEATED_NOOP_COMMAND"); }
            recordError("NOOP_COMMAND", "empty movement and scan");
            ++step_index_;
            return {DroneStepStatus::Continue, "noop"};
        }
        if (++invalid_command_strikes_ >= kMaxRetries) {
            throw std::runtime_error("REPEATED_INVALID_COMMAND");
        }
        recordError("INVALID_COMMAND", "algorithm command rejected");
        ++step_index_;
        return {DroneStepStatus::Continue, "invalid"};
    }
    noop_strikes_ = 0;
    invalid_command_strikes_ = 0;
    if (actions.size() > 1) { ++split_command_count_; }

    const PendingAction first = actions.front();
    actions.pop_front();
    for (auto& extra : actions) { pending_actions_.push_back(std::move(extra)); }
    return executeAction(first);
}

common::types::DroneState DroneControlImpl::state() const {
    return {dead_reckoned_position_, dead_reckoned_heading_, step_index_};
}

} // namespace mission_control_213309941_213727837
