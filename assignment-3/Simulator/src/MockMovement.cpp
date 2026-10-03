// MockMovement.cpp - updates MockGPS. Throws on hidden-map wall collision (mandatory).

#include <Simulator/MockMovement.h>
#include <UserCommon/GeometryUtils.h>

#include <cmath>
#include <stdexcept>

namespace simulator {

namespace ucm = user_common_213309941_213727837;

MockMovement::MockMovement(MockGPS& gps, const common::IMap3D& hidden_map,
                           common::PhysicalLength drone_radius)
    : gps_(gps), hidden_map_(hidden_map), drone_radius_(drone_radius) {}

void MockMovement::assertPathClear(const common::Position3D& from,
                                   const common::Position3D& to) const {
    const double length = ucm::distanceCm(from, to);
    const double res = std::max(1.0, ucm::cmOf(hidden_map_.getMapConfig().resolution));
    const double radius = ucm::cmOf(drone_radius_);
    const int samples = std::max(1, static_cast<int>(std::ceil(length / (0.25 * res))));
    for (int i = 1; i <= samples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(samples);
        const common::Position3D p = ucm::positionCm(
            ucm::cmOf(from.x) + t * (ucm::cmOf(to.x) - ucm::cmOf(from.x)),
            ucm::cmOf(from.y) + t * (ucm::cmOf(to.y) - ucm::cmOf(from.y)),
            ucm::cmOf(from.z) + t * (ucm::cmOf(to.z) - ucm::cmOf(from.z)));
        // Probe the hull with a few offsets so a grazing collision is still seen.
        const double offs[] = {0.0, radius * 0.5, -radius * 0.5};
        for (double ox : offs) {
            for (double oy : offs) {
                const common::Position3D probe =
                    ucm::positionCm(ucm::cmOf(p.x) + ox, ucm::cmOf(p.y) + oy, ucm::cmOf(p.z));
                if (hidden_map_.atVoxel(probe) == common::types::VoxelOccupancy::Occupied) {
                    throw std::runtime_error("WALL_COLLISION: movement hit occupied voxel");
                }
            }
        }
    }
}

common::types::MovementResult MockMovement::rotate(common::types::RotationDirection direction,
                                                   common::HorizontalAngle angle) {
    const common::Orientation current = gps_.heading();
    const common::HorizontalAngle signed_angle =
        (direction == common::types::RotationDirection::Left) ? angle : -angle;
    gps_.setHeading(common::Orientation{current.horizontal + signed_angle, current.altitude});
    return common::types::MovementResult{true, {}};
}

common::types::MovementResult MockMovement::advance(common::PhysicalLength distance) {
    const common::Position3D current = gps_.position();
    const ucm::UnitVector3 dir =
        ucm::directionOf(common::Orientation{gps_.heading().horizontal, {}});
    const double dist = ucm::cmOf(distance);
    const common::Position3D target =
        ucm::positionCm(ucm::cmOf(current.x) + dir.x * dist,
                        ucm::cmOf(current.y) + dir.y * dist,
                        ucm::cmOf(current.z));
    assertPathClear(current, target);
    gps_.setPosition(target);
    return common::types::MovementResult{true, {}};
}

common::types::MovementResult MockMovement::elevate(common::PhysicalLength distance) {
    const common::Position3D current = gps_.position();
    const common::Position3D target = ucm::positionCm(
        ucm::cmOf(current.x), ucm::cmOf(current.y), ucm::cmOf(current.z) + ucm::cmOf(distance));
    assertPathClear(current, target);
    gps_.setPosition(target);
    return common::types::MovementResult{true, {}};
}

} // namespace simulator
