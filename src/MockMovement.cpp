#include <drone_mapper/MockMovement.h>

#include <cmath>
#include <numbers>

namespace drone_mapper {

namespace {
    constexpr double PI = std::numbers::pi;
    double toRadians(double degrees) { return degrees * PI / 180.0; }
} // namespace

MockMovement::MockMovement(MockGPS& gps) : gps_(gps) {}

types::MovementResult MockMovement::rotate(types::RotationDirection direction, HorizontalAngle angle) {
    const Orientation current = gps_.heading();
    const HorizontalAngle signed_angle =
        (direction == types::RotationDirection::Left) ? angle : -angle;
    gps_.setHeading(Orientation{current.horizontal + signed_angle, current.altitude});
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::advance(PhysicalLength distance) {
    const Position3D current = gps_.position();
    const Orientation heading = gps_.heading();

    const double heading_rad = toRadians(heading.horizontal.numerical_value_in(deg));
    const double dist_cm = distance.numerical_value_in(cm);

    const double dx = dist_cm * std::cos(heading_rad);
    const double dy = dist_cm * std::sin(heading_rad);

    const Position3D target{
        current.x + dx * x_extent[cm],
        current.y + dy * y_extent[cm],
        current.z,
    };

    gps_.setPosition(target);
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    const Position3D current = gps_.position();
    const double dz_cm = distance.numerical_value_in(cm);

    const Position3D target{
        current.x,
        current.y,
        current.z + dz_cm * z_extent[cm],
    };

    gps_.setPosition(target);
    return types::MovementResult{true, {}};
}

} // namespace drone_mapper
