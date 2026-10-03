#pragma once

// MockMovement.h - simulated motors. Implements course-owned common::IDroneMovement.
// Holds the hidden map so it can detect wall collisions (mandatory Common-issues row).

#include <Common/IDroneMovement.h>
#include <Common/IMap3D.h>
#include <Simulator/MockGPS.h>

namespace simulator {

class MockMovement final : public common::IDroneMovement {
public:
    MockMovement(MockGPS& gps, const common::IMap3D& hidden_map,
                 common::PhysicalLength drone_radius);

    common::types::MovementResult rotate(common::types::RotationDirection direction,
                                         common::HorizontalAngle angle) override;
    common::types::MovementResult advance(common::PhysicalLength distance) override;
    common::types::MovementResult elevate(common::PhysicalLength distance) override;

private:
    // Samples the straight segment and throws std::runtime_error on Occupied voxels
    // within the drone hull. Mandatory: "Throw an exception" on a wall collision.
    void assertPathClear(const common::Position3D& from, const common::Position3D& to) const;

    MockGPS& gps_;                      // pose we mutate
    const common::IMap3D& hidden_map_;  // ground truth
    common::PhysicalLength drone_radius_{};
};

} // namespace simulator
