// ScanResultToVoxels.cpp - implementation of MissionControl/ScanResultToVoxels.h.

#include <MissionControl/ScanResultToVoxels.h>

#include <UserCommon/GeometryUtils.h> // shared beam maths, so MockLidar and this agree exactly

#include <algorithm>
#include <cmath>

namespace mission_control_213309941_213727837 {

namespace ucm = user_common_213309941_213727837;

using common::Orientation;
using common::Position3D;
using common::types::VoxelOccupancy;

namespace {

// Ranks occupancy states so a write only ever upgrades a voxel's certainty.
// Occupied beats Empty beats PotentiallyOccupied beats Unmapped, which means a wall
// observed once is never repainted as free space by a later grazing beam.
[[nodiscard]] int certainty(VoxelOccupancy occupancy) {
    switch (occupancy) {
        case VoxelOccupancy::Occupied:            return 3;
        case VoxelOccupancy::Empty:               return 2;
        case VoxelOccupancy::PotentiallyOccupied: return 1;
        case VoxelOccupancy::Unmapped:
        case VoxelOccupancy::OutOfBounds:         return 0;
    }
    return 0;
}

// Writes `value` at `position` only when it is more certain than what is already there.
void writeIfStronger(common::IMutableMap3D& output_map, const Position3D& position,
                     VoxelOccupancy value) {
    if (!output_map.isInBounds(position)) { return; }
    if (certainty(value) > certainty(output_map.atVoxel(position))) {
        output_map.set(position, value);
    }
}

// Marks every sample along [start_cm, end_cm] of one beam with `value`.
// Sampling at 0.4 voxels guarantees no voxel is skipped even for a fully diagonal beam
// (the worst-case gap between consecutive samples stays below one voxel edge).
void markBeamSegment(common::IMutableMap3D& output_map, const Position3D& origin,
                     const Orientation& beam, double start_cm, double end_cm, double step_cm,
                     VoxelOccupancy value) {
    if (!(step_cm > 0.0) || end_cm < start_cm) { return; }
    for (double distance = start_cm; distance <= end_cm; distance += step_cm) {
        const Position3D point = ucm::pointAlongBeam(origin, beam, ucm::lengthCm(distance));
        if (!output_map.isInBounds(point)) { break; } // left the mission volume: stop this beam
        writeIfStronger(output_map, point, value);
    }
}

} // namespace

// Iterates the hits of one scan and paints the map. `drone_heading` converts the
// drone-relative beam angles carried by LidarHit into absolute map directions.
void ScanResultToVoxels::applyToMap(common::IMutableMap3D& output_map,
                                     const Position3D& scan_origin,
                                     const Orientation& drone_heading,
                                     const common::types::LidarScanResult& scan,
                                     const common::types::LidarConfigData& lidar_config) {
    if (!output_map.isInBounds(scan_origin)) { return; }

    const double resolution_cm = ucm::cmOf(output_map.getMapConfig().resolution);
    const double step_cm = 0.4 * resolution_cm;
    if (!(step_cm > 0.0)) { return; }

    const double z_min_cm = ucm::cmOf(lidar_config.z_min);
    const double z_max_cm = ucm::cmOf(lidar_config.z_max);

    // The drone itself occupies the origin voxel and it is certainly free space.
    writeIfStronger(output_map, scan_origin, VoxelOccupancy::Empty);

    for (const common::types::LidarHit& hit : scan) {
        const Orientation beam = ucm::composeOrientation(drone_heading, hit.angle);
        const double distance_cm = ucm::cmOf(hit.distance);

        if (ucm::isLidarMiss(hit.distance)) {
            // Nothing within range along this beam: the whole ray is free space.
            markBeamSegment(output_map, scan_origin, beam, 0.0, z_max_cm, step_cm,
                            VoxelOccupancy::Empty);
            continue;
        }

        if (distance_cm <= 0.0) {
            // Something is closer than the LiDAR's minimum range; we know it is there but
            // not where exactly, so the blind zone is flagged rather than mapped.
            markBeamSegment(output_map, scan_origin, beam, 0.0, z_min_cm, step_cm,
                            VoxelOccupancy::PotentiallyOccupied);
            continue;
        }

        // Free up to just before the hit, solid at the hit itself.
        markBeamSegment(output_map, scan_origin, beam, 0.0,
                        std::max(0.0, distance_cm - 0.5 * resolution_cm), step_cm,
                        VoxelOccupancy::Empty);
        writeIfStronger(output_map, ucm::pointAlongBeam(scan_origin, beam, hit.distance),
                        VoxelOccupancy::Occupied);
    }
}

} // namespace mission_control_213309941_213727837
