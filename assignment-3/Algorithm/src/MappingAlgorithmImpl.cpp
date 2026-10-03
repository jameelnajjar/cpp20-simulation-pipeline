// MappingAlgorithmImpl.cpp - implementation of our mapping algorithm plus the
// registration statement that publishes it to the Simulator when this .so is dlopen'ed.

#include <Algorithm/MappingAlgorithmImpl.h> //

#include <Common/MappingAlgorithmRegistration.h> // course-owned: REGISTER_MAPPING_ALGORITHM macro

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <numbers>
#include <optional>
#include <queue>
#include <utility>

namespace algorithm_213309941_213727837 {

using common::Orientation;
using common::Position3D;
using common::types::AlgorithmStatus;
using common::types::MappingStepCommand;
using common::types::MovementCommand;
using common::types::MovementCommandType;
using common::types::RotationDirection;
using common::types::VoxelOccupancy;

namespace {

// The 26 lattice offsets around a cell (all face, edge and corner neighbours).
// Used to decide whether a cell still borders unexplored space.
[[nodiscard]] const std::array<ucm::VoxelKey, 26>& neighbourOffsets26() {
    static const std::array<ucm::VoxelKey, 26> offsets = [] {
        std::array<ucm::VoxelKey, 26> built{};
        std::size_t index = 0;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) { continue; }
                    built[index++] = ucm::VoxelKey{dx, dy, dz};
                }
            }
        }
        return built;
    }();
    return offsets;
} 

// The 6 face-adjacent lattice offsets, used as the BFS expansion set.
// Face adjacency (rather than 26-adjacency) keeps every path segment axis-aligned,
// which maps cleanly onto the drone's advance/elevate primitives.
[[nodiscard]] const std::array<ucm::VoxelKey, 6>& neighbourOffsets6() {
    static const std::array<ucm::VoxelKey, 6> offsets{
        ucm::VoxelKey{1, 0, 0},  ucm::VoxelKey{-1, 0, 0}, ucm::VoxelKey{0, 1, 0},
        ucm::VoxelKey{0, -1, 0}, ucm::VoxelKey{0, 0, 1},  ucm::VoxelKey{0, 0, -1},
    };
    return offsets;
} 

// Adds two lattice keys componentwise.
[[nodiscard]] ucm::VoxelKey addKeys(const ucm::VoxelKey& lhs, const ucm::VoxelKey& rhs) {
    return ucm::VoxelKey{lhs.ix + rhs.ix, lhs.iy + rhs.iy, lhs.iz + rhs.iz};
}

// Upper bound on how many cells a single BFS may expand, so a huge mission volume
// cannot make one nextStep() call take unbounded time.
constexpr std::size_t kMaxBfsExpansions = 200000;

} // namespace

// Delegates config/map storage to the course-owned base and leaves everything else to
// initialise(), which cannot run here because it needs the map's runtime geometry.
MappingAlgorithmImpl::MappingAlgorithmImpl(common::MappingAlgorithmDependencies dependencies)
    : common::IMappingAlgorithm(std::move(dependencies)) {}

// Reads the output map's geometry once and derives every derived constant from it.
void MappingAlgorithmImpl::initialise() {
    const common::types::MapConfig config = output_map_.getMapConfig();

    resolution_cm_ = ucm::cmOf(config.resolution);
    if (!(resolution_cm_ > 0.0)) {
        // Degenerate map: fall back to the GPS resolution so the algorithm still runs.
        resolution_cm_ = std::max(1.0, ucm::cmOf(mission_config_.gps_resolution));
    }

    origin_ = common::Position3D{
        config.boundaries.min_x,
        config.boundaries.min_y,
        config.boundaries.min_height,
    };

    const double span_x = ucm::cmOf(config.boundaries.max_x) - ucm::cmOf(config.boundaries.min_x);
    const double span_y = ucm::cmOf(config.boundaries.max_y) - ucm::cmOf(config.boundaries.min_y);
    const double span_z =
        ucm::cmOf(config.boundaries.max_height) - ucm::cmOf(config.boundaries.min_height);

    cells_x_ = std::max(1, static_cast<int>(std::floor(span_x / resolution_cm_)));
    cells_y_ = std::max(1, static_cast<int>(std::floor(span_y / resolution_cm_)));
    cells_z_ = std::max(1, static_cast<int>(std::floor(span_z / resolution_cm_)));

    // drone_config_.radius is half of the "sphere diameter the drone can pass through".
    // Half a voxel is added so a drone hugging a voxel border still clears the wall.
    clearance_cm_ = ucm::cmOf(drone_config_.radius) + 0.5 * resolution_cm_;
    clearance_cells_ = std::max(0, static_cast<int>(std::floor(clearance_cm_ / resolution_cm_)));

    scan_pattern_ = buildScanPattern();
    phase_ = Phase::Planning;
}

// Lattice cell -> centre of that cell in mission-frame world coordinates.
Position3D MappingAlgorithmImpl::keyToPosition(const VoxelKey& key) const {
    return Position3D{
        origin_.x + ucm::xCm((key.ix + 0.5) * resolution_cm_),
        origin_.y + ucm::yCm((key.iy + 0.5) * resolution_cm_),
        origin_.z + ucm::zCm((key.iz + 0.5) * resolution_cm_),
    };
}

// World position -> lattice cell containing it (floor division against the bounds minimum).
MappingAlgorithmImpl::VoxelKey MappingAlgorithmImpl::positionToKey(
    const Position3D& position) const {
    return VoxelKey{
        static_cast<int>(std::floor((ucm::cmOf(position.x) - ucm::cmOf(origin_.x)) / resolution_cm_)),
        static_cast<int>(std::floor((ucm::cmOf(position.y) - ucm::cmOf(origin_.y)) / resolution_cm_)),
        static_cast<int>(std::floor((ucm::cmOf(position.z) - ucm::cmOf(origin_.z)) / resolution_cm_)),
    };
}

// Pure index range check against the lattice extents computed in initialise().
bool MappingAlgorithmImpl::isInBounds(const VoxelKey& key) const {
    return key.ix >= 0 && key.ix < cells_x_ && key.iy >= 0 && key.iy < cells_y_ && key.iz >= 0 &&
           key.iz < cells_z_;
}

// Single point of contact with the course-owned IMap3D read interface.
VoxelOccupancy MappingAlgorithmImpl::occupancyAt(const VoxelKey& key) const {
    return output_map_.atVoxel(keyToPosition(key));
}

// Rejects a cell whose clearance cube contains anything that might be solid.
// PotentiallyOccupied counts as solid: it means "a beam stopped before z_min here".
bool MappingAlgorithmImpl::hasClearance(const VoxelKey& key) const {
    for (int dx = -clearance_cells_; dx <= clearance_cells_; ++dx) {
        for (int dy = -clearance_cells_; dy <= clearance_cells_; ++dy) {
            for (int dz = -clearance_cells_; dz <= clearance_cells_; ++dz) {
                const VoxelKey probe{key.ix + dx, key.iy + dy, key.iz + dz};
                if (!isInBounds(probe)) { return false; }
                const VoxelOccupancy occupancy = occupancyAt(probe);
                if (occupancy == VoxelOccupancy::Occupied ||
                    occupancy == VoxelOccupancy::PotentiallyOccupied) {
                    return false;
                }
            }
        }
    }
    return true;
}

// A cell is flyable only when we have positively observed it as Empty and it clears the hull.
bool MappingAlgorithmImpl::isNavigable(const VoxelKey& key) const {
    if (!isInBounds(key)) { return false; }
    if (blocked_cells_.count(key) != 0) { return false; }
    if (occupancyAt(key) != VoxelOccupancy::Empty) { return false; }
    return hasClearance(key);
}

// Frontier test: does this cell touch anything we have not observed yet?
bool MappingAlgorithmImpl::hasFrontierNeighbour(const VoxelKey& key) const {
    for (const VoxelKey& offset : neighbourOffsets26()) {
        const VoxelKey probe = addKeys(key, offset);
        if (!isInBounds(probe)) { continue; }
        if (occupancyAt(probe) == VoxelOccupancy::Unmapped) { return true; }
    }
    return false;
}

// Uniform-cost (unit-weight) BFS: the first frontier cell popped is a nearest one.
// Returns the waypoint chain excluding the start cell.
std::vector<MappingAlgorithmImpl::VoxelKey> MappingAlgorithmImpl::findPathToFrontier(
    const VoxelKey& start) const {
    std::map<VoxelKey, VoxelKey> parent; // child -> parent, also doubles as the visited set
    std::queue<VoxelKey> pending;

    parent.emplace(start, start);
    pending.push(start);
    std::size_t expansions = 0;

    while (!pending.empty() && expansions < kMaxBfsExpansions) {
        const VoxelKey current = pending.front();
        pending.pop();
        ++expansions;

        // Do not re-target a cell we already scanned from; its frontier is already harvested.
        const bool worth_visiting =
            !(current == start) && scanned_cells_.count(current) == 0 && hasFrontierNeighbour(current);
        if (worth_visiting) {
            std::vector<VoxelKey> path;
            for (VoxelKey step = current; !(step == start); step = parent.at(step)) {
                path.push_back(step);
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const VoxelKey& offset : neighbourOffsets6()) {
            const VoxelKey next = addKeys(current, offset);
            if (parent.count(next) != 0) { continue; }
            if (!isNavigable(next)) { continue; }
            parent.emplace(next, current);
            pending.push(next);
        }
    }
    return {};
}

// Samples the segment every half voxel and demands that each sample sits in a navigable cell.
bool MappingAlgorithmImpl::segmentIsClear(const Position3D& from, const Position3D& to) const {
    const double length = ucm::distanceCm(from, to);
    const int samples = std::max(1, static_cast<int>(std::ceil(length / (resolution_cm_ * 0.5))));
    for (int index = 1; index <= samples; ++index) {
        const double t = static_cast<double>(index) / static_cast<double>(samples);
        const Position3D sample = ucm::positionCm(
            ucm::cmOf(from.x) + t * (ucm::cmOf(to.x) - ucm::cmOf(from.x)),
            ucm::cmOf(from.y) + t * (ucm::cmOf(to.y) - ucm::cmOf(from.y)),
            ucm::cmOf(from.z) + t * (ucm::cmOf(to.z) - ucm::cmOf(from.z)));
        if (!isNavigable(positionToKey(sample))) { return false; }
    }
    return true;
}

// Walks the remaining path backwards and returns the last index still in line of sight,
// turning a staircase of single-cell hops into one long straight flight.
std::size_t MappingAlgorithmImpl::furthestVisibleWaypoint(const Position3D& from) const {
    std::size_t best = path_index_;
    for (std::size_t candidate = path_.size(); candidate > path_index_; --candidate) {
        const std::size_t index = candidate - 1;
        if (segmentIsClear(from, keyToPosition(path_[index]))) {
            best = index;
            break;
        }
    }
    return best;
}

// Angular radius of the LiDAR footprint: the outermost beam ring sits (fov_circles-1)*d
// away from the centre beam at the minimum operational distance z_min.
double MappingAlgorithmImpl::coneHalfAngleDeg() const {
    const double z_min = ucm::cmOf(lidar_config_.z_min);
    const double spacing = ucm::cmOf(lidar_config_.d);
    if (lidar_config_.fov_circles <= 1 || !(z_min > 0.0) || !(spacing > 0.0)) {
        return 10.0; // single-beam LiDAR: treat as a narrow pencil beam
    }
    const double radius = static_cast<double>(lidar_config_.fov_circles - 1) * spacing;
    const double half_angle = std::atan2(radius, z_min) * 180.0 / std::numbers::pi;
    return std::clamp(half_angle, 5.0, 45.0);
}

// Tiles the sphere with cones of radius coneHalfAngleDeg(): one ring of yaw directions per
// pitch level, with the yaw count shrinking as the ring approaches the poles.
std::vector<Orientation> MappingAlgorithmImpl::buildScanPattern() const {
    const double half_angle = coneHalfAngleDeg();
    const double step = std::max(8.0, 2.0 * half_angle);

    std::vector<Orientation> pattern;
    // Pitch levels from -84 to +84 degrees; straight up/down are added separately.
    for (double pitch = -84.0; pitch <= 84.0 + 1e-9; pitch += step) {
        const double shrink = std::max(0.15, std::cos(pitch * std::numbers::pi / 180.0));
        const int yaw_count = std::max(1, static_cast<int>(std::ceil(360.0 / (step / shrink))));
        for (int index = 0; index < yaw_count; ++index) {
            const double yaw = 360.0 * static_cast<double>(index) / static_cast<double>(yaw_count);
            pattern.push_back(Orientation{ucm::yawDeg(yaw), ucm::pitchDeg(pitch)});
        }
    }
    // Explicit zenith and nadir so ceilings and floors are always covered.
    pattern.push_back(Orientation{ucm::yawDeg(0.0), ucm::pitchDeg(89.0)});
    pattern.push_back(Orientation{ucm::yawDeg(0.0), ucm::pitchDeg(-89.0)});
    return pattern;
}

// Cheap information-gain estimate: march the cone's centre and four edge rays outwards and
// report whether any sample is still Unmapped. Marching stops at the first solid sample,
// because the LiDAR cannot see past an obstacle either.
bool MappingAlgorithmImpl::scanIsUseful(const Position3D& origin,
                                        const Orientation& absolute_direction) const {
    const double half_angle = coneHalfAngleDeg();
    const double z_max = ucm::cmOf(lidar_config_.z_max);
    const double stride = std::max(resolution_cm_, 1.0);

    const std::array<Orientation, 5> rays{
        absolute_direction,
        ucm::composeOrientation(absolute_direction, Orientation{ucm::yawDeg(half_angle), {}}),
        ucm::composeOrientation(absolute_direction, Orientation{ucm::yawDeg(-half_angle), {}}),
        ucm::composeOrientation(absolute_direction, Orientation{{}, ucm::pitchDeg(half_angle)}),
        ucm::composeOrientation(absolute_direction, Orientation{{}, ucm::pitchDeg(-half_angle)}),
    };

    for (const Orientation& ray : rays) {
        for (double distance = stride; distance <= z_max; distance += stride) {
            const Position3D sample = ucm::pointAlongBeam(origin, ray, ucm::lengthCm(distance));
            const VoxelKey key = positionToKey(sample);
            if (!isInBounds(key)) { break; }
            const VoxelOccupancy occupancy = occupancyAt(key);
            if (occupancy == VoxelOccupancy::Unmapped) { return true; }
            if (occupancy == VoxelOccupancy::Occupied) { break; }
        }
    }
    return false;
}

// Queues every scan direction that scanIsUseful() approves, expressed relative to the
// drone's current heading (the ILidar contract adds the heading back in).
void MappingAlgorithmImpl::queueScansAt(const common::types::DroneState& state) {
    pending_scans_.clear();
    for (const Orientation& relative : scan_pattern_) {
        const Orientation absolute = ucm::composeOrientation(state.heading, relative);
        if (scanIsUseful(state.position, absolute)) {
            pending_scans_.push_back(relative);
        }
    }
    scanned_cells_.insert(positionToKey(state.position));
}

// Turns a "go to this point" request into one legal primitive. Vertical error is cleared
// first because elevating needs no rotation; then the drone turns and advances.
// Every returned magnitude is clamped to the drone's configured maxima, so this algorithm
// never triggers the "movement bigger than max allowed" row of the issues table.
MovementCommand MappingAlgorithmImpl::buildMovement(const common::types::DroneState& state,
                                                    const Position3D& target) const {
    const double max_rotate = std::abs(ucm::degOf(drone_config_.max_rotate));
    const double max_advance = std::abs(ucm::cmOf(drone_config_.max_advance));
    const double max_elevate = std::abs(ucm::cmOf(drone_config_.max_elevate));

    const double dx = ucm::cmOf(target.x) - ucm::cmOf(state.position.x);
    const double dy = ucm::cmOf(target.y) - ucm::cmOf(state.position.y);
    const double dz = ucm::cmOf(target.z) - ucm::cmOf(state.position.z);
    const double planar = std::hypot(dx, dy);
    const double tolerance = resolution_cm_ * kArrivalFraction;

    if (std::abs(dz) > tolerance && max_elevate > 0.0) {
        return MovementCommand{MovementCommandType::Elevate, RotationDirection::Left,
                               ucm::yawDeg(0.0),
                               ucm::lengthCm(std::clamp(dz, -max_elevate, max_elevate))};
    }

    if (planar > tolerance) {
        const double desired_yaw = std::atan2(dy, dx) * 180.0 / std::numbers::pi;
        const double error = ucm::angleDiffDeg(ucm::degOf(state.heading.horizontal), desired_yaw);
        if (std::abs(error) > kHeadingToleranceDeg && max_rotate > 0.0) {
            const double magnitude = std::min(std::abs(error), max_rotate);
            const RotationDirection direction =
                (error >= 0.0) ? RotationDirection::Left : RotationDirection::Right;
            return MovementCommand{MovementCommandType::Rotate, direction, ucm::yawDeg(magnitude),
                                   ucm::lengthCm(0.0)};
        }
        if (max_advance > 0.0) {
            return MovementCommand{MovementCommandType::Advance, RotationDirection::Left,
                                   ucm::yawDeg(0.0),
                                   ucm::lengthCm(std::min(planar, max_advance))};
        }
    }

    if (std::abs(dz) > 0.0 && max_elevate > 0.0) {
        return MovementCommand{MovementCommandType::Elevate, RotationDirection::Left,
                               ucm::yawDeg(0.0),
                               ucm::lengthCm(std::clamp(dz, -max_elevate, max_elevate))};
    }

    return MovementCommand{MovementCommandType::Hover, RotationDirection::Left, ucm::yawDeg(0.0),
                           ucm::lengthCm(0.0)};
}

// Full sweep of the mission volume; only called once, when exploration has terminated.
bool MappingAlgorithmImpl::anyVoxelUnmapped() const {
    for (int ix = 0; ix < cells_x_; ++ix) {
        for (int iy = 0; iy < cells_y_; ++iy) {
            for (int iz = 0; iz < cells_z_; ++iz) {
                if (occupancyAt(VoxelKey{ix, iy, iz}) == VoxelOccupancy::Unmapped) { return true; }
            }
        }
    }
    return false;
}

// Builds a MappingStepCommand carrying only a scan request.
MappingStepCommand MappingAlgorithmImpl::scanCommand(const Orientation& relative_direction) {
    return MappingStepCommand{std::nullopt, relative_direction, AlgorithmStatus::Working};
}

// Builds a MappingStepCommand carrying only a movement request.
MappingStepCommand MappingAlgorithmImpl::moveCommand(const MovementCommand& movement) {
    return MappingStepCommand{movement, std::nullopt, AlgorithmStatus::Working};
}

// Builds the terminal MappingStepCommand that tells the DroneControl to stop.
MappingStepCommand MappingAlgorithmImpl::finishedCommand(AlgorithmStatus status) {
    return MappingStepCommand{std::nullopt, std::nullopt, status};
}

std::optional<MappingStepCommand> MappingAlgorithmImpl::handleScanning() {
    if (!pending_scans_.empty()) {
        const Orientation direction = pending_scans_.front();
        pending_scans_.pop_front();
        return scanCommand(direction);
    }
    phase_ = Phase::Planning;
    return std::nullopt;
}

std::optional<MappingStepCommand> MappingAlgorithmImpl::handlePlanning(
    const common::types::DroneState& state) {
    const VoxelKey current = positionToKey(state.position);

    if (scanned_cells_.count(current) == 0) {
        queueScansAt(state);
        if (!pending_scans_.empty()) {
            phase_ = Phase::Scanning;
            return std::nullopt;
        }
    }

    path_ = findPathToFrontier(current);
    if (path_.empty()) {
        phase_ = Phase::Done;
        return finishedCommand(anyVoxelUnmapped() ? AlgorithmStatus::FinishedWithUnmappableVoxels
                                                  : AlgorithmStatus::Finished);
    }
    path_index_ = 0;
    waypoint_attempts_ = 0;
    path_index_ = furthestVisibleWaypoint(state.position);
    if (path_index_ >= path_.size()) {
        path_.clear();
        phase_ = Phase::Planning;
        return std::nullopt;
    }
    waypoint_ = keyToPosition(path_[path_index_]);
    phase_ = Phase::Navigating;
    return std::nullopt;
}

std::optional<MappingStepCommand> MappingAlgorithmImpl::handleNavigating(
    const common::types::DroneState& state) {
    const double remaining = ucm::distanceCm(state.position, waypoint_);

    if (remaining <= resolution_cm_ * kArrivalFraction) {
        if (path_index_ + 1 >= path_.size()) {
            path_.clear();
            phase_ = Phase::Planning;
            return std::nullopt;
        }
        waypoint_attempts_ = 0;
        last_remaining_cm_ = -1.0;
        path_index_ = std::max(path_index_ + 1, furthestVisibleWaypoint(state.position));
        if (path_index_ >= path_.size()) {
            path_.clear();
            phase_ = Phase::Planning;
            return std::nullopt;
        }
        waypoint_ = keyToPosition(path_[path_index_]);
    }

    if (last_remaining_cm_ >= 0.0 && remaining < last_remaining_cm_ - 0.25) {
        waypoint_attempts_ = 0;
    }
    last_remaining_cm_ = remaining;

    if (waypoint_attempts_ >= kMaxWaypointAttempts) {
        if (path_index_ < path_.size()) { blocked_cells_.insert(path_[path_index_]); }
        path_.clear();
        waypoint_attempts_ = 0;
        last_remaining_cm_ = -1.0;
        phase_ = Phase::Planning;
        return std::nullopt;
    }

    const MovementCommand movement = buildMovement(state, waypoint_);
    if (movement.type == MovementCommandType::Hover) {
        if (path_index_ >= path_.size() || path_index_ + 1 >= path_.size()) {
            path_.clear();
            phase_ = Phase::Planning;
            return std::nullopt;
        }
        ++path_index_;
        waypoint_attempts_ = 0;
        last_remaining_cm_ = -1.0;
        waypoint_ = keyToPosition(path_[path_index_]);
        return std::nullopt;
    }
    ++waypoint_attempts_;
    return moveCommand(movement);
}

MappingStepCommand MappingAlgorithmImpl::nextStep(const common::types::DroneState& state,
                                                  const common::types::LidarScanResult* /*latest_scan*/) {
    if (phase_ == Phase::NeedsInit) { initialise(); }
    if (phase_ == Phase::Done) { return finishedCommand(AlgorithmStatus::Finished); }

    for (int transition = 0; transition < 8; ++transition) {
        switch (phase_) {
            case Phase::Scanning:
                if (auto command = handleScanning()) { return *command; }
                break;
            case Phase::Planning:
                if (auto command = handlePlanning(state)) { return *command; }
                break;
            case Phase::Navigating:
                if (auto command = handleNavigating(state)) { return *command; }
                break;
            case Phase::Done:
                return finishedCommand(AlgorithmStatus::Finished);
            case Phase::NeedsInit:
                phase_ = Phase::Planning;
                break;
        }
    }

    return moveCommand(MovementCommand{MovementCommandType::Hover, RotationDirection::Left,
                                       ucm::yawDeg(0.0), ucm::lengthCm(0.0)});
}

} // namespace algorithm_213309941_213727837

// The registration macro pastes the class name into an identifier, so it needs a name
// without "::". This global alias gives the macro a legal token while keeping the class
// itself inside the mandated unique namespace.
using MappingAlgorithmImpl_213309941_213727837 =
    algorithm_213309941_213727837::MappingAlgorithmImpl;

// Declares a namespace-scope object whose constructor runs during dlopen() and hands a
// factory lambda to common::MappingAlgorithmRegistration, which the Simulator implements.
// This is the whole "automatic registration" contract - no exported C symbols needed.
REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl_213309941_213727837); // Most important update to the pulled file from project 2 ( the instatnce made for the registration for the name of the Algo being used and the factories intializaton for it ! )
