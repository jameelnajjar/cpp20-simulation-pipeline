#pragma once

// GeometryUtils.h - header-only geometry/unit helpers shared by all three projects
// (Algorithm, MissionControl, Simulator). Lives in UserCommon because more than one
// project needs the exact same maths; duplicating it would risk the three binaries
// disagreeing about where a LiDAR beam lands.

#include <Common/Units.h> // course-owned header: defines Position3D, Orientation, cm, deg, mp, si
#include <Common/types/MapTypes.h> // course-owned header: defines MapConfig / MappingBounds

#include <mp-units/systems/si/math.h> // si::sin / si::cos / si::atan2 (not in core mp-units/math.h)

#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>

namespace user_common_213309941_213727837 {

// Pulls the course-owned `common` namespace (Position3D, XLength, cm, deg, si, mp, ...)
// into this namespace so the helpers below read like plain maths.
using namespace common;

// ---------------------------------------------------------------------------
// Scalar extraction: mp-units quantity -> plain double in centimetres/degrees.
// Every quantity spec (isq::length, x_extent, y_extent, z_extent) needs its own
// overload because mp-units deliberately makes them distinct types.
// ---------------------------------------------------------------------------

// Converts a generic length quantity (isq::length[cm]) to a bare double in cm.
[[nodiscard]] inline double cmOf(PhysicalLength value) { return value.numerical_value_in(cm); }

// Converts an X-axis length (x_extent[cm], from Common/Units.h) to a bare double in cm.
[[nodiscard]] inline double cmOf(XLength value) { return value.numerical_value_in(cm); }

// Converts a Y-axis length (y_extent[cm], from Common/Units.h) to a bare double in cm.
[[nodiscard]] inline double cmOf(YLength value) { return value.numerical_value_in(cm); }

// Converts a Z-axis length (z_extent[cm], from Common/Units.h) to a bare double in cm.
[[nodiscard]] inline double cmOf(ZLength value) { return value.numerical_value_in(cm); }

// Converts a horizontal (yaw) angle quantity to a bare double in degrees.
[[nodiscard]] inline double degOf(HorizontalAngle value) { return value.numerical_value_in(deg); }

// Converts an altitude (pitch) angle quantity to a bare double in degrees.
[[nodiscard]] inline double degOf(AltitudeAngle value) { return value.numerical_value_in(deg); }

// ---------------------------------------------------------------------------
// Scalar injection: plain double -> mp-units quantity of the right spec.
// ---------------------------------------------------------------------------

// Builds a generic length quantity from centimetres (used for advance/elevate distances).
[[nodiscard]] inline PhysicalLength lengthCm(double value) { return value * cm; }

// Builds an X-axis coordinate from centimetres (x_extent spec, required by Position3D::x).
[[nodiscard]] inline XLength xCm(double value) { return value * x_extent[cm]; }

// Builds a Y-axis coordinate from centimetres (y_extent spec, required by Position3D::y).
[[nodiscard]] inline YLength yCm(double value) { return value * y_extent[cm]; }

// Builds a Z-axis coordinate from centimetres (z_extent spec, required by Position3D::z).
[[nodiscard]] inline ZLength zCm(double value) { return value * z_extent[cm]; }

// Builds a yaw angle from degrees (horizontal_angle spec, required by Orientation::horizontal).
[[nodiscard]] inline HorizontalAngle yawDeg(double value) { return value * horizontal_angle[deg]; }

// Builds a pitch angle from degrees (altitude_angle spec, required by Orientation::altitude).
[[nodiscard]] inline AltitudeAngle pitchDeg(double value) { return value * altitude_angle[deg]; }

// Assembles a common::Position3D (course-owned struct) from three plain cm values.
[[nodiscard]] inline Position3D positionCm(double x, double y, double z) {
    return Position3D{xCm(x), yCm(y), zCm(z)};
}

// ---------------------------------------------------------------------------
// Angle arithmetic.
// ---------------------------------------------------------------------------

// Wraps any degree value into the half-open range [0, 360).
[[nodiscard]] inline double normalizeDeg(double degrees) {
    double wrapped = std::fmod(degrees, 360.0);
    if (wrapped < 0.0) { wrapped += 360.0; }
    return wrapped;
}

// Signed shortest rotation (in degrees, range (-180, 180]) that takes `from` to `to`.
// Positive means "turn left" in this project's convention.
[[nodiscard]] inline double angleDiffDeg(double from_deg, double to_deg) {
    double diff = std::fmod(to_deg - from_deg, 360.0);
    if (diff > 180.0) { diff -= 360.0; }
    if (diff <= -180.0) { diff += 360.0; }
    return diff;
}

// ---------------------------------------------------------------------------
// Beam geometry - the single definition of "where does a beam point".
// MockLidar (Simulator) and ScanResultToVoxels (MissionControl) MUST agree on
// this, otherwise the mapped world would be rotated relative to the real one.
// ---------------------------------------------------------------------------

// Plain unit-vector triple in map coordinates; not an mp-units type on purpose,
// because it is dimensionless and only ever multiplied by a distance.
struct UnitVector3 {
    double x = 0.0; // eastward component  (cos(altitude) * cos(yaw))
    double y = 0.0; // northward component (cos(altitude) * sin(yaw))
    double z = 0.0; // upward component    (sin(altitude))
};

// Converts an absolute Orientation (course-owned struct) into a unit direction vector.
[[nodiscard]] inline UnitVector3 directionOf(const Orientation& orientation) {
    const auto cos_altitude = si::cos(orientation.altitude);
    const auto dx = cos_altitude * si::cos(orientation.horizontal);
    const auto dy = cos_altitude * si::sin(orientation.horizontal);
    const auto dz = si::sin(orientation.altitude);
    return UnitVector3{
        dx.numerical_value_in(mp::one),
        dy.numerical_value_in(mp::one),
        dz.numerical_value_in(mp::one),
    };
}

// Adds a drone heading to a beam orientation that is expressed relative to the drone.
[[nodiscard]] inline Orientation composeOrientation(const Orientation& heading,
                                                    const Orientation& relative) {
    return Orientation{heading.horizontal + relative.horizontal,
                       heading.altitude + relative.altitude};
}

// Returns the point reached by travelling `distance` from `origin` along `orientation`.
[[nodiscard]] inline Position3D pointAlongBeam(const Position3D& origin,
                                               const Orientation& orientation,
                                               PhysicalLength distance) {
    const UnitVector3 dir = directionOf(orientation);
    const double travelled = cmOf(distance);
    return Position3D{
        origin.x + xCm(dir.x * travelled),
        origin.y + yCm(dir.y * travelled),
        origin.z + zCm(dir.z * travelled),
    };
}

// Straight-line distance in cm between two Position3D values.
[[nodiscard]] inline double distanceCm(const Position3D& lhs, const Position3D& rhs) {
    const double dx = cmOf(lhs.x) - cmOf(rhs.x);
    const double dy = cmOf(lhs.y) - cmOf(rhs.y);
    const double dz = cmOf(lhs.z) - cmOf(rhs.z);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// True when every coordinate of a position is a finite number; guards against a
// faulty component handing us NaN/inf (see "Common issues" table, GPS rows).
[[nodiscard]] inline bool isFinitePosition(const Position3D& position) {
    return std::isfinite(cmOf(position.x)) && std::isfinite(cmOf(position.y)) &&
           std::isfinite(cmOf(position.z));
}

// True when both components of an orientation are finite numbers.
[[nodiscard]] inline bool isFiniteOrientation(const Orientation& orientation) {
    return std::isfinite(degOf(orientation.horizontal)) && std::isfinite(degOf(orientation.altitude));
}

// True when a position lies inside the given MappingBounds (course-owned struct), inclusive.
[[nodiscard]] inline bool isInsideBounds(const types::MappingBounds& bounds,
                                         const Position3D& position) {
    const double px = cmOf(position.x);
    const double py = cmOf(position.y);
    const double pz = cmOf(position.z);
    return px >= cmOf(bounds.min_x) && px <= cmOf(bounds.max_x) &&
           py >= cmOf(bounds.min_y) && py <= cmOf(bounds.max_y) &&
           pz >= cmOf(bounds.min_height) && pz <= cmOf(bounds.max_height);
}

// ---------------------------------------------------------------------------
// Discrete voxel addressing, shared by the algorithm's search grid and by any
// component that needs a hashable position.
// ---------------------------------------------------------------------------

// Integer lattice coordinate. Plain aggregate, not derived from anything.
struct VoxelKey {
    int ix = 0; // grid index along X
    int iy = 0; // grid index along Y
    int iz = 0; // grid index along Z

    // Value equality, needed by std::unordered_set<VoxelKey>.
    [[nodiscard]] friend bool operator==(const VoxelKey& lhs, const VoxelKey& rhs) {
        return lhs.ix == rhs.ix && lhs.iy == rhs.iy && lhs.iz == rhs.iz;
    }

    // Total order, needed by std::map<VoxelKey, ...> in the BFS parent table.
    [[nodiscard]] friend bool operator<(const VoxelKey& lhs, const VoxelKey& rhs) {
        if (lhs.ix != rhs.ix) { return lhs.ix < rhs.ix; }
        if (lhs.iy != rhs.iy) { return lhs.iy < rhs.iy; }
        return lhs.iz < rhs.iz;
    }
};

// Rounds a world position onto the lattice defined by `resolution_cm`.
[[nodiscard]] inline VoxelKey positionToKey(const Position3D& position, double resolution_cm) {
    if (!(resolution_cm > 0.0)) { return VoxelKey{}; }
    return VoxelKey{
        static_cast<int>(std::lround(cmOf(position.x) / resolution_cm)),
        static_cast<int>(std::lround(cmOf(position.y) / resolution_cm)),
        static_cast<int>(std::lround(cmOf(position.z) / resolution_cm)),
    };
}

// Inverse of positionToKey(): returns the world position of a lattice node.
[[nodiscard]] inline Position3D keyToPosition(const VoxelKey& key, double resolution_cm) {
    return positionCm(key.ix * resolution_cm, key.iy * resolution_cm, key.iz * resolution_cm);
}

// Sentinel distance used by ILidar implementations to mean "beam hit nothing".
// Matches the assignment-2 convention that MockLidar returns DBL_MAX cm on a miss.
inline constexpr double kLidarMissCm = std::numeric_limits<double>::max();

// True when a LiDAR hit distance is the "no obstacle found" sentinel.
[[nodiscard]] inline bool isLidarMiss(PhysicalLength distance) {
    return cmOf(distance) >= kLidarMissCm;
}

} // namespace user_common_213309941_213727837

// std::hash specialisation so VoxelKey can key an unordered_set/unordered_map.
// Specialising a std template for a user type is the standard-sanctioned way to do this.
template <>
struct std::hash<user_common_213309941_213727837::VoxelKey> {
    // Mixes the three indices with the 64-bit FNV-style constant used by boost::hash_combine.
    [[nodiscard]] std::size_t operator()(
        const user_common_213309941_213727837::VoxelKey& key) const noexcept {
        std::size_t seed = 0;
        const auto mix = [&seed](int value) {
            seed ^= std::hash<int>{}(value) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
        };
        mix(key.ix);
        mix(key.iy);
        mix(key.iz);
        return seed;
    }
};
