#pragma once

// ScanResultToVoxels.h - turns one LiDAR scan into voxel writes on the output map.
//
// It belongs to the MissionControl project because the MissionControl (through its own
// DroneControl) is the component that owns the mapping loop: the Simulator hands it an
// IMutableMap3D and it is the MissionControl's job to fill it in.

#include <Common/IMutableMap3D.h>       // course-owned: the writable map interface
#include <Common/types/LidarTypes.h>    // course-owned: LidarScanResult / LidarConfigData
#include <Common/Units.h>               // course-owned: Position3D / Orientation

namespace mission_control_213309941_213727837 {

// Stateless utility; grouped as a class with a static member purely for namespacing,
// exactly as in assignment 2. Not derived from anything.
class ScanResultToVoxels {
public:
    // Writes one scan into `output_map`.
    //
    // Beam semantics (unchanged from assignment 2's MockLidar contract):
    //   distance == 0                  -> an obstacle sits closer than z_min: the LiDAR cannot
    //                                     measure it, so the segment [0, z_min] is marked
    //                                     PotentiallyOccupied.
    //   distance == DBL_MAX            -> the beam hit nothing: [0, z_max] is Empty.
    //   0 < distance < DBL_MAX         -> [0, distance) is Empty and the end point is Occupied.
    //
    // Writes are "strongest wins" so that a later Empty observation cannot erase a wall.
    static void applyToMap(common::IMutableMap3D& output_map,
                           const common::Position3D& scan_origin,
                           const common::Orientation& drone_heading,
                           const common::types::LidarScanResult& scan,
                           const common::types::LidarConfigData& lidar_config);
};

} // namespace mission_control_213309941_213727837
