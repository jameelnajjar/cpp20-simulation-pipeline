#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/IMap3D.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>

namespace drone_mapper {

namespace {
    constexpr double PI = std::numbers::pi;
    constexpr double DEG_TO_RAD = PI / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / PI;

    double toCmX(XLength v)         { return v.numerical_value_in(cm); }
    double toCmY(YLength v)         { return v.numerical_value_in(cm); }
    double toCmZ(ZLength v)         { return v.numerical_value_in(cm); }
    double toCm(PhysicalLength v)   { return v.numerical_value_in(cm); }
    double toDeg(HorizontalAngle v) { return v.numerical_value_in(deg); }

    double angleDiff(double from_deg, double to_deg) {
        double diff = to_deg - from_deg;
        while (diff > 180.0) diff -= 360.0;
        while (diff < -180.0) diff += 360.0;
        return diff;
    }
} // namespace

MappingAlgorithmImpl::VoxelKey MappingAlgorithmImpl::posToKey(const Position3D& pos) const {
    const double res = toCm(mission_config_.gps_resolution);
    if (res <= 0.0) return {0, 0, 0};
    const double x = toCmX(pos.x);
    const double y = toCmY(pos.y);
    const double z = toCmZ(pos.z);
    return {
        static_cast<int>(std::round(x / res)),
        static_cast<int>(std::round(y / res)),
        static_cast<int>(std::round(z / res)),
    };
}

Position3D MappingAlgorithmImpl::keyToPos(const VoxelKey& key) const {
    const double res = toCm(mission_config_.gps_resolution);
    return Position3D{
        (key.ix * res) * x_extent[cm],
        (key.iy * res) * y_extent[cm],
        (key.iz * res) * z_extent[cm],
    };
}

bool MappingAlgorithmImpl::isNavigable(const VoxelKey& key) const {
    const Position3D pos = keyToPos(key);
    if (!output_map_.isInBounds(pos)) return false;
    const auto occ = output_map_.atVoxel(pos);
    return occ != types::VoxelOccupancy::Occupied;
}

void MappingAlgorithmImpl::queueNeighbors(const VoxelKey& key) {
    const std::array<VoxelKey, 26> directions = {
        // Horizontal (XY) neighbors
        VoxelKey{key.ix+1, key.iy,   key.iz},
        VoxelKey{key.ix-1, key.iy,   key.iz},
        VoxelKey{key.ix,   key.iy+1, key.iz},
        VoxelKey{key.ix,   key.iy-1, key.iz},
        VoxelKey{key.ix+1, key.iy+1, key.iz},
        VoxelKey{key.ix+1, key.iy-1, key.iz},
        VoxelKey{key.ix-1, key.iy+1, key.iz},
        VoxelKey{key.ix-1, key.iy-1, key.iz},
        // Vertical neighbors
        VoxelKey{key.ix,   key.iy,   key.iz+1},
        VoxelKey{key.ix,   key.iy,   key.iz-1},
        // Diagonals up
        VoxelKey{key.ix+1, key.iy,   key.iz+1},
        VoxelKey{key.ix-1, key.iy,   key.iz+1},
        VoxelKey{key.ix,   key.iy+1, key.iz+1},
        VoxelKey{key.ix,   key.iy-1, key.iz+1},
        // Diagonals down
        VoxelKey{key.ix+1, key.iy,   key.iz-1},
        VoxelKey{key.ix-1, key.iy,   key.iz-1},
        VoxelKey{key.ix,   key.iy+1, key.iz-1},
        VoxelKey{key.ix,   key.iy-1, key.iz-1},
        // Far diagonals
        VoxelKey{key.ix+1, key.iy+1, key.iz+1},
        VoxelKey{key.ix-1, key.iy+1, key.iz+1},
        VoxelKey{key.ix+1, key.iy-1, key.iz+1},
        VoxelKey{key.ix-1, key.iy-1, key.iz+1},
        VoxelKey{key.ix+1, key.iy+1, key.iz-1},
        VoxelKey{key.ix-1, key.iy+1, key.iz-1},
        VoxelKey{key.ix+1, key.iy-1, key.iz-1},
        VoxelKey{key.ix-1, key.iy-1, key.iz-1},
    };

    for (const auto& neighbor : directions) {
        if (enqueued_.count(neighbor) > 0) continue;
        const Position3D pos = keyToPos(neighbor);
        if (!output_map_.isInBounds(pos)) continue;
        const auto occ = output_map_.atVoxel(pos);
        if (occ == types::VoxelOccupancy::Unmapped || occ == types::VoxelOccupancy::Empty) {
            frontier_.push_back(neighbor);
            enqueued_.insert(neighbor);
        }
    }
}

std::vector<MappingAlgorithmImpl::VoxelKey>
MappingAlgorithmImpl::bfsPath(const VoxelKey& from, const VoxelKey& to) const {
    if (from == to) return {from};

    std::map<VoxelKey, VoxelKey> parent;
    std::deque<VoxelKey> queue;
    queue.push_back(from);
    parent[from] = from;

    const int max_bfs = 2000;
    int explored = 0;

    while (!queue.empty() && explored < max_bfs) {
        VoxelKey cur = queue.front();
        queue.pop_front();
        ++explored;

        if (cur == to) {
            std::vector<VoxelKey> path;
            VoxelKey c = to;
            while (!(c == from)) {
                path.push_back(c);
                c = parent[c];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        const std::array<VoxelKey, 6> neighbors = {
            VoxelKey{cur.ix+1, cur.iy,   cur.iz},
            VoxelKey{cur.ix-1, cur.iy,   cur.iz},
            VoxelKey{cur.ix,   cur.iy+1, cur.iz},
            VoxelKey{cur.ix,   cur.iy-1, cur.iz},
            VoxelKey{cur.ix,   cur.iy,   cur.iz+1},
            VoxelKey{cur.ix,   cur.iy,   cur.iz-1},
        };

        for (const auto& nb : neighbors) {
            if (parent.count(nb) > 0) continue;
            if (!isNavigable(nb)) continue;
            parent[nb] = cur;
            queue.push_back(nb);
        }
    }
    return {}; // No path found
}

types::MovementCommand MappingAlgorithmImpl::buildMoveCommand(const types::DroneState& state,
                                                               const Position3D& next_pos) const {
    const double max_rotate = std::abs(toDeg(drone_config_.max_rotate));
    const double max_advance = toCm(drone_config_.max_advance);
    const double max_elevate = toCm(drone_config_.max_elevate);

    const double cx = toCmX(state.position.x);
    const double cy = toCmY(state.position.y);
    const double cz = toCmZ(state.position.z);
    const double tx = toCmX(next_pos.x);
    const double ty = toCmY(next_pos.y);
    const double tz = toCmZ(next_pos.z);
    const double current_heading = toDeg(state.heading.horizontal);

    const double dz = tz - cz;
    const double dxy = std::sqrt((tx - cx) * (tx - cx) + (ty - cy) * (ty - cy));

    // Prefer vertical movement first if large elevation change
    if (std::abs(dz) > 1.0 && std::abs(dz) > dxy * 0.5) {
        const double move_dz = std::clamp(dz, -max_elevate, max_elevate);
        return types::MovementCommand{
            types::MovementCommandType::Elevate,
            types::RotationDirection::Left,
            0.0 * horizontal_angle[deg],
            move_dz * cm,
        };
    }

    if (dxy > 1.0) {
        const double target_deg = std::atan2(ty - cy, tx - cx) * RAD_TO_DEG;
        const double diff = angleDiff(current_heading, target_deg);

        if (std::abs(diff) > 5.0) {
            const double rotate_amount = std::clamp(std::abs(diff), 0.0, max_rotate);
            const auto direction = (diff >= 0) ? types::RotationDirection::Left
                                               : types::RotationDirection::Right;
            return types::MovementCommand{
                types::MovementCommandType::Rotate,
                direction,
                rotate_amount * horizontal_angle[deg],
                0.0 * cm,
            };
        }

        const double advance_dist = std::clamp(dxy, 0.0, max_advance);
        return types::MovementCommand{
            types::MovementCommandType::Advance,
            types::RotationDirection::Left,
            0.0 * horizontal_angle[deg],
            advance_dist * cm,
        };
    }

    if (std::abs(dz) > 1.0) {
        const double move_dz = std::clamp(dz, -max_elevate, max_elevate);
        return types::MovementCommand{
            types::MovementCommandType::Elevate,
            types::RotationDirection::Left,
            0.0 * horizontal_angle[deg],
            move_dz * cm,
        };
    }

    return types::MovementCommand{types::MovementCommandType::Hover, {}, {}, {}};
}

types::MappingStepCommand MappingAlgorithmImpl::nextStep(const types::DroneState& state,
                                                          const types::LidarScanResult* /*latest_scan*/) {
    // Initialize on first call
    if (!initialized_) {
        initialized_ = true;
        phase_ = AlgoPhase::Scanning;
        // Seed 8 horizontal scan directions + 2 diagonal up/down
        for (int i = 0; i < 8; ++i) {
            const double angle_deg = i * 45.0;
            scan_queue_.push_back(Orientation{
                angle_deg * horizontal_angle[deg],
                0.0 * altitude_angle[deg],
            });
        }
        scan_queue_.push_back(Orientation{0.0 * horizontal_angle[deg], 45.0 * altitude_angle[deg]});
        scan_queue_.push_back(Orientation{0.0 * horizontal_angle[deg], -45.0 * altitude_angle[deg]});
    }

    // Phase: emit queued scans
    if (phase_ == AlgoPhase::Scanning && !scan_queue_.empty()) {
        const Orientation scan_dir = scan_queue_.back();
        scan_queue_.pop_back();
        const VoxelKey cur_key = posToKey(state.position);
        if (scan_queue_.empty()) {
            visited_.insert(cur_key);
            queueNeighbors(cur_key);
        }
        return types::MappingStepCommand{
            std::nullopt,
            scan_dir,
            types::AlgorithmStatus::Working,
        };
    }

    // Phase: navigate along planned path
    if (phase_ == AlgoPhase::Navigating && nav_idx_ < nav_path_.size()) {
        const VoxelKey& target_key = nav_path_[nav_idx_];
        const Position3D target_pos = keyToPos(target_key);

        const double res = toCm(mission_config_.gps_resolution);
        const double cx = toCmX(state.position.x);
        const double cy = toCmY(state.position.y);
        const double cz = toCmZ(state.position.z);
        const double tx = toCmX(target_pos.x);
        const double ty = toCmY(target_pos.y);
        const double tz = toCmZ(target_pos.z);
        const double dist = std::sqrt((tx-cx)*(tx-cx) + (ty-cy)*(ty-cy) + (tz-cz)*(tz-cz));

        ++waypoint_attempts_;
        const bool arrived = (dist < res * 0.6);
        const bool stuck = (waypoint_attempts_ > MAX_WAYPOINT_ATTEMPTS);
        if (arrived || stuck) {
            waypoint_attempts_ = 0;
            if (stuck) {
                // Give up on this waypoint - mark as visited so we don't retry
                visited_.insert(target_key);
            }
            ++nav_idx_;
            if (nav_idx_ >= nav_path_.size()) {
                if (arrived) {
                    // Actually arrived at exploration target - scan from here
                    phase_ = AlgoPhase::Scanning;
                    for (int i = 0; i < 8; ++i) {
                        const double angle_deg = i * 45.0;
                        scan_queue_.push_back(Orientation{
                            angle_deg * horizontal_angle[deg],
                            0.0 * altitude_angle[deg],
                        });
                    }
                    scan_queue_.push_back(Orientation{0.0 * horizontal_angle[deg], 45.0 * altitude_angle[deg]});
                    scan_queue_.push_back(Orientation{0.0 * horizontal_angle[deg], -45.0 * altitude_angle[deg]});
                } else {
                    // Was stuck - skip scanning and go straight back to planning
                    phase_ = AlgoPhase::Planning;
                }
                return nextStep(state, nullptr);
            }
        }

        // Still navigating: build movement command
        const types::MovementCommand cmd = buildMoveCommand(state, target_pos);
        if (cmd.type == types::MovementCommandType::Hover) {
            ++nav_idx_;
            return nextStep(state, nullptr);
        }
        return types::MappingStepCommand{cmd, std::nullopt, types::AlgorithmStatus::Working};
    }

    // Phase: find next target from frontier
    phase_ = AlgoPhase::Planning;

    while (!frontier_.empty()) {
        const VoxelKey target = frontier_.front();
        frontier_.pop_front();

        if (visited_.count(target) > 0) continue;
        if (!output_map_.isInBounds(keyToPos(target))) continue;

        const VoxelKey cur_key = posToKey(state.position);
        std::vector<VoxelKey> path = bfsPath(cur_key, target);

        if (!path.empty()) {
            nav_path_ = std::move(path);
            nav_idx_ = 0;
            waypoint_attempts_ = 0;
            phase_ = AlgoPhase::Navigating;
            return nextStep(state, nullptr);
        }
    }

    // No more reachable targets
    phase_ = AlgoPhase::Done;
    return types::MappingStepCommand{std::nullopt, std::nullopt, types::AlgorithmStatus::Finished};
}

} // namespace drone_mapper
