#pragma once

#include <drone_mapper/IMappingAlgorithm.h>

#include <deque>
#include <optional>
#include <set>
#include <vector>

namespace drone_mapper {

// BFS-based mapping algorithm that systematically explores the mission space.
class MappingAlgorithmImpl final : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;

    [[nodiscard]] types::MappingStepCommand nextStep(const types::DroneState& state,
                                                      const types::LidarScanResult* latest_scan) override;

private:
    struct VoxelKey {
        int ix, iy, iz;
        bool operator<(const VoxelKey& o) const {
            if (ix != o.ix) return ix < o.ix;
            if (iy != o.iy) return iy < o.iy;
            return iz < o.iz;
        }
        bool operator==(const VoxelKey& o) const {
            return ix == o.ix && iy == o.iy && iz == o.iz;
        }
    };

    enum class AlgoPhase {
        Initial,
        Scanning,
        Planning,
        Navigating,
        Done,
    };

    // Convert world position to discrete grid key (using gps_resolution).
    [[nodiscard]] VoxelKey posToKey(const Position3D& pos) const;
    // Convert discrete key back to world position (center of voxel).
    [[nodiscard]] Position3D keyToPos(const VoxelKey& key) const;

    // Queue all unmapped neighbors of the current key.
    void queueNeighbors(const VoxelKey& key);

    // BFS to find a path from current key to target key.
    [[nodiscard]] std::vector<VoxelKey> bfsPath(const VoxelKey& from, const VoxelKey& to) const;

    // Build movement command to go toward next_pos from state.
    [[nodiscard]] types::MovementCommand buildMoveCommand(const types::DroneState& state,
                                                           const Position3D& next_pos) const;

    // Return true if a voxel key is navigable (not occupied, in bounds).
    [[nodiscard]] bool isNavigable(const VoxelKey& key) const;

    AlgoPhase phase_ = AlgoPhase::Initial;
    std::deque<VoxelKey> frontier_{};      // BFS exploration queue
    std::set<VoxelKey> enqueued_{};        // keys already added to frontier
    std::set<VoxelKey> visited_{};         // keys we've physically visited

    std::vector<Orientation> scan_queue_{};  // remaining scan orientations at current position
    std::vector<VoxelKey> nav_path_{};       // remaining navigation waypoints
    std::size_t nav_idx_ = 0;
    int waypoint_attempts_ = 0;              // steps spent trying to reach current waypoint

    static constexpr int MAX_WAYPOINT_ATTEMPTS = 5;

    bool initialized_ = false;
};

} // namespace drone_mapper
