#pragma once 

// MappingAlgorithmImpl.h - our concrete drone mapping algorithm.
//
// It is compiled into Algorithm_213309941_213727837.so and is loaded at run time by the
// Simulator. The Simulator never links this file; the only contract between them is the
// course-owned common::IMappingAlgorithm interface plus the registration macro.
//
// Strategy: "safe frontier exploration".
//   1. From the current cell, emit only those LiDAR scan directions whose cone still
//      contains unmapped voxels (skipping useless scans is where most of the step budget
//      is saved, which matters for the competition ranking: score first, then fewer steps).
//   2. Breadth-first search over cells that are *known empty* with drone clearance, to find
//      the nearest cell that still has unmapped neighbours.
//   3. Fly there along a line-of-sight-smoothed path, then go back to step 1.
// Because step 2 only ever walks through voxels already observed as Empty, the drone can
// never be commanded into a wall - the mandatory "a valid algorithm does not create the
// error" row of the course's "Common issues and handling" table.

#include <Common/IMappingAlgorithm.h>       // course-owned: base class + MappingAlgorithmDependencies
#include <Common/IMap3D.h>                  // course-owned: read-only map view the algorithm inspects
#include <Common/types/DroneTypes.h>        // course-owned: MappingStepCommand, MovementCommand, DroneState
#include <Common/types/LidarTypes.h>        // course-owned: LidarScanResult / LidarConfigData

#include <UserCommon/GeometryUtils.h>       // our shared maths: VoxelKey, unit helpers, beam geometry

#include <cstddef>
#include <deque>
#include <unordered_set>
#include <vector>

namespace algorithm_213309941_213727837 {

// Shorthand for the shared helper namespace declared in UserCommon/GeometryUtils.h.
namespace ucm = user_common_213309941_213727837;

class MappingAlgorithmImpl final : public common::IMappingAlgorithm {
public:
    // Sole constructor. Takes the dependency bundle *by value* because that is exactly what
    // REGISTER_MAPPING_ALGORITHM's generated lambda does (`std::make_unique<T>(std::move(deps))`).
    // It forwards to the course-owned IMappingAlgorithm base, which stores mission/lidar/drone
    // configs and the const IMap3D& for us in its protected members.
    explicit MappingAlgorithmImpl(common::MappingAlgorithmDependencies dependencies);

    // The one virtual entry point, inherited from common::IMappingAlgorithm.
    // Returns the next movement and/or scan the drone should perform.
    // `latest_scan` may legitimately be nullptr (no scan was performed on the previous step).
    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& state,
        const common::types::LidarScanResult* latest_scan) override;

private:
    // Which part of the explore loop we are currently in.
    enum class Phase {
        NeedsInit,  // nothing computed yet; first nextStep() call bootstraps the algorithm
        Scanning,   // draining pending_scans_ at the current waypoint
        Planning,   // looking for the next frontier cell
        Navigating, // flying along path_
        Done,       // no reachable frontier remains
    };

    // Alias for the lattice coordinate defined in UserCommon/GeometryUtils.h.
    using VoxelKey = ucm::VoxelKey;

    // One-time setup performed on the first nextStep(): reads the output map geometry,
    // derives clearance and the scan pattern, and queues the first burst of scans.
    void initialise();

    // Converts a lattice index into the centre of the matching output-map voxel.
    [[nodiscard]] common::Position3D keyToPosition(const VoxelKey& key) const;

    // Converts a world position into the lattice index of the voxel containing it.
    [[nodiscard]] VoxelKey positionToKey(const common::Position3D& position) const;

    // True when the lattice cell lies inside the mission boundaries of the output map.
    [[nodiscard]] bool isInBounds(const VoxelKey& key) const;

    // Reads the occupancy the output map currently holds for a lattice cell.
    [[nodiscard]] common::types::VoxelOccupancy occupancyAt(const VoxelKey& key) const;

    // True when every cell within the drone's clearance radius is known not to be solid.
    // This is what keeps the drone away from walls it has already seen.
    [[nodiscard]] bool hasClearance(const VoxelKey& key) const;

    // True when the cell is in bounds, observed as Empty, and has clearance.
    // Deliberately refuses Unmapped cells: we never fly into territory we have not seen.
    [[nodiscard]] bool isNavigable(const VoxelKey& key) const;

    // True when at least one of the 26 neighbours of `key` is still Unmapped, i.e. standing
    // here (or nearby) could still teach us something.
    [[nodiscard]] bool hasFrontierNeighbour(const VoxelKey& key) const;

    // Breadth-first search over navigable cells starting at `start`.
    // Returns the path (excluding `start`) to the nearest cell with a frontier neighbour,
    // or an empty vector when no such cell is reachable.
    [[nodiscard]] std::vector<VoxelKey> findPathToFrontier(const VoxelKey& start) const;

    // True when the straight segment between two world positions only crosses navigable
    // cells; used to shortcut the lattice path into long straight flights.
    [[nodiscard]] bool segmentIsClear(const common::Position3D& from,
                                      const common::Position3D& to) const;

    // Index of the furthest path waypoint still reachable in a straight line from `from`.
    [[nodiscard]] std::size_t furthestVisibleWaypoint(const common::Position3D& from) const;

    // Half-angle of the LiDAR cone in degrees, derived from the lidar config
    // (outermost ring radius (fov_circles-1)*d seen from distance z_min).
    [[nodiscard]] double coneHalfAngleDeg() const;

    // Builds the fixed set of scan orientations that tiles the sphere with the LiDAR cone.
    [[nodiscard]] std::vector<common::Orientation> buildScanPattern() const;

    // True when firing the LiDAR in this direction can still reveal unmapped voxels.
    // Samples the centre ray plus four cone-edge rays between z_min and z_max.
    [[nodiscard]] bool scanIsUseful(const common::Position3D& origin,
                                    const common::Orientation& absolute_direction) const;

    // Fills pending_scans_ with the useful subset of the scan pattern for `state`.
    void queueScansAt(const common::types::DroneState& state);

    // Produces a single legal MovementCommand that makes progress from `state` towards
    // `target`. Always respects drone_config_'s max_rotate / max_advance / max_elevate.
    [[nodiscard]] common::types::MovementCommand buildMovement(
        const common::types::DroneState& state, const common::Position3D& target) const;

    // Scans the whole mission volume once and reports whether any voxel is still Unmapped;
    // decides between AlgorithmStatus::Finished and FinishedWithUnmappableVoxels.
    [[nodiscard]] bool anyVoxelUnmapped() const;

    // Convenience wrapper returning a "scan only" command.
    [[nodiscard]] static common::types::MappingStepCommand scanCommand(
        const common::Orientation& relative_direction);

    // Convenience wrapper returning a "move only" command.
    [[nodiscard]] static common::types::MappingStepCommand moveCommand(
        const common::types::MovementCommand& movement);

    // Convenience wrapper returning a terminal command carrying `status`.
    [[nodiscard]] static common::types::MappingStepCommand finishedCommand(
        common::types::AlgorithmStatus status);

    Phase phase_ = Phase::NeedsInit; // explore-loop state

    double resolution_cm_ = 0.0;   // output map voxel size; the planning lattice pitch
    double clearance_cm_ = 0.0;    // drone radius plus half a voxel of safety margin
    int clearance_cells_ = 0;      // clearance_cm_ expressed in lattice cells
    common::Position3D origin_{};  // world position of lattice cell (0,0,0) = mission bounds minimum
    int cells_x_ = 0;              // lattice extent along X
    int cells_y_ = 0;              // lattice extent along Y
    int cells_z_ = 0;              // lattice extent along Z

    std::vector<common::Orientation> scan_pattern_{}; // sphere-covering directions, built once
    std::deque<common::Orientation> pending_scans_{}; // still to emit at the current waypoint

    std::vector<VoxelKey> path_{};        // lattice waypoints produced by findPathToFrontier()
    std::size_t path_index_ = 0;          // next waypoint in path_
    common::Position3D waypoint_{};       // world position currently being flown to
    int waypoint_attempts_ = 0;           // consecutive commands that made no progress
    double last_remaining_cm_ = -1.0;     // previous distance to waypoint_, -1 when unknown

    std::unordered_set<VoxelKey> scanned_cells_{}; // cells we already scanned from
    std::unordered_set<VoxelKey> blocked_cells_{}; // frontier cells we failed to reach

    // Give up on a single waypoint after this many movement commands and re-plan.
    static constexpr int kMaxWaypointAttempts = 12;
    // Treat the drone as "arrived" once it is within this fraction of a voxel.
    static constexpr double kArrivalFraction = 0.55;
    // Do not bother rotating for heading errors smaller than this.
    static constexpr double kHeadingToleranceDeg = 3.0;
};

} // namespace algorithm_213309941_213727837
