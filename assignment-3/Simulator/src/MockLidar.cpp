// MockLidar.cpp - concentric-ring LiDAR from assignment 2. Beam order is i/beam_count
// (NOT reversed) so LID04-style bugs stay out of our code.

#include <Simulator/MockLidar.h>
#include <UserCommon/GeometryUtils.h>

#include <mp-units/systems/si/math.h>

namespace simulator {

namespace ucm = user_common_213309941_213727837;

namespace {

// Ring i has 4^i beams (assignment 1/2 LiDAR model).
[[nodiscard]] std::size_t beamsOnCircle(std::size_t circle_index) {
    std::size_t count = 1;
    for (std::size_t i = 0; i < circle_index; ++i) { count *= 4; }
    return count;
}

// atan2(offset, z_min) as a yaw quantity; inherited from HW2 MockLidar.
[[nodiscard]] common::HorizontalAngle horizontalDelta(common::PhysicalLength offset,
                                                      common::PhysicalLength distance) {
    return common::HorizontalAngle{common::si::atan2(offset, distance)};
}

// atan2(offset, z_min) as a pitch quantity; inherited from HW2 MockLidar.
[[nodiscard]] common::AltitudeAngle altitudeDelta(common::PhysicalLength offset,
                                                  common::PhysicalLength distance) {
    return common::AltitudeAngle{common::si::atan2(offset, distance)};
}

} // namespace

MockLidar::MockLidar(common::types::LidarConfigData config, const common::IMap3D& map,
                     const common::IGPS& gps)
    : config_(config), map_(map), gps_(gps) {}

common::types::LidarConfigData MockLidar::config() const { return config_; }

common::types::LidarScanResult MockLidar::scan(common::Orientation scan_orientation) const {
    common::types::LidarScanResult results;
    if (config_.fov_circles == 0) { return results; }

    const common::Orientation heading = gps_.heading();
    const common::Orientation center_abs{
        scan_orientation.horizontal + heading.horizontal,
        scan_orientation.altitude + heading.altitude,
    };
    results.push_back(common::types::LidarHit{traceBeam(center_abs), scan_orientation});

    for (std::size_t circle = 1; circle < config_.fov_circles; ++circle) {
        const std::size_t beam_count = beamsOnCircle(circle);
        const common::PhysicalLength radius = static_cast<double>(circle) * config_.d;
        for (std::size_t i = 0; i < beam_count; ++i) {
            const auto theta =
                (360.0 * static_cast<double>(i) / static_cast<double>(beam_count)) * common::deg;
            const common::PhysicalLength h_off = radius * common::si::cos(theta);
            const common::PhysicalLength a_off = radius * common::si::sin(theta);
            const common::Orientation offset{
                horizontalDelta(h_off, config_.z_min),
                altitudeDelta(a_off, config_.z_min),
            };
            const common::Orientation relative{
                scan_orientation.horizontal + offset.horizontal,
                scan_orientation.altitude + offset.altitude,
            };
            const common::Orientation absolute{
                relative.horizontal + heading.horizontal,
                relative.altitude + heading.altitude,
            };
            results.push_back(common::types::LidarHit{traceBeam(absolute), relative});
        }
    }
    return results;
}

common::PhysicalLength MockLidar::traceBeam(const common::Orientation& beam) const {
    const common::Position3D origin = gps_.position();
    const double res_cm = ucm::cmOf(map_.getMapConfig().resolution);
    const double step_cm = (res_cm > 0.0) ? 0.1 * res_cm : 1.0;
    const double z_max = ucm::cmOf(config_.z_max);
    const double z_min = ucm::cmOf(config_.z_min);

    for (double distance = 0.0; distance <= z_max; distance += step_cm) {
        const common::Position3D sample = ucm::pointAlongBeam(origin, beam, ucm::lengthCm(distance));
        if (map_.atVoxel(sample) == common::types::VoxelOccupancy::Occupied) {
            if (distance < z_min) { return ucm::lengthCm(0.0); }
            return ucm::lengthCm(distance);
        }
    }
    return ucm::lengthCm(ucm::kLidarMissCm);
}

} // namespace simulator
