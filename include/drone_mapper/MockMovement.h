#pragma once

#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/MockGPS.h>

namespace drone_mapper {
// Optional implementation for the IDroneMovement interface using MockGPS.
class MockMovement final : public IDroneMovement {
public:
    explicit MockMovement(MockGPS& gps);

    types::MovementResult rotate(types::RotationDirection direction, HorizontalAngle angle) override;
    types::MovementResult advance(PhysicalLength distance) override;
    types::MovementResult elevate(PhysicalLength distance) override; // Can be negative!

private:
    MockGPS& gps_;
};

} // namespace drone_mapper
