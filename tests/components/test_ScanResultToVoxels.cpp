#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/NpyArray.h>
#include <drone_mapper/ScanResultToVoxels.h>

#include <limits>

using namespace drone_mapper;

namespace {

types::MapConfig makeConfig(std::size_t n = 20, double res = 10.0) {
    types::MapConfig cfg;
    cfg.resolution = res * cm;
    cfg.offset     = Position3D{};
    const double size = static_cast<double>(n) * res;
    cfg.boundaries = types::MappingBounds{
        0.0 * x_extent[cm], size * x_extent[cm],
        0.0 * y_extent[cm], size * y_extent[cm],
        0.0 * z_extent[cm], size * z_extent[cm],
    };
    return cfg;
}

PhysicalLength missDistance() {
    return std::numeric_limits<double>::max() * cm;
}

} // namespace

// A miss beam marks voxels along the full z_max ray as Empty.
TEST(ScanResultToVoxels, MissMarkBeamEmpty) {
    const std::size_t N = 20;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl map(arr, makeConfig(N));

    // Origin at (50, 100, 100), beam east (angle 0), miss
    const Position3D origin{50.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    const Orientation heading{};
    const Orientation beam{};
    types::LidarConfigData cfg{5.0 * cm, 80.0 * cm, 2.5 * cm, 1};
    types::LidarScanResult scan = {{missDistance(), beam}};

    ScanResultToVoxels::applyToMap(map, origin, heading, scan, cfg);

    // Voxel 40 cm east of origin: (90, 100, 100)
    const Position3D check{90.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    ASSERT_TRUE(map.isInBounds(check));
    EXPECT_EQ(map.atVoxel(check), types::VoxelOccupancy::Empty);
}

// A normal hit marks the exact hit voxel as Occupied.
TEST(ScanResultToVoxels, HitMarkOccupied) {
    const std::size_t N = 20;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl map(arr, makeConfig(N));

    const Position3D origin{50.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    const Orientation heading{};
    const Orientation beam{};
    types::LidarConfigData cfg{5.0 * cm, 80.0 * cm, 2.5 * cm, 1};
    types::LidarScanResult scan = {{50.0 * cm, beam}};  // hit at 50 cm east

    ScanResultToVoxels::applyToMap(map, origin, heading, scan, cfg);

    // Hit position: (50+50, 100, 100) = (100, 100, 100)
    const Position3D hit_pos{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    ASSERT_TRUE(map.isInBounds(hit_pos));
    EXPECT_EQ(map.atVoxel(hit_pos), types::VoxelOccupancy::Occupied);
}

// The path traversed before a hit is marked Empty (not Unmapped, not Occupied).
TEST(ScanResultToVoxels, PathBeforeHitIsEmpty) {
    const std::size_t N = 20;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl map(arr, makeConfig(N));

    const Position3D origin{50.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    const Orientation heading{};
    const Orientation beam{};
    types::LidarConfigData cfg{5.0 * cm, 80.0 * cm, 2.5 * cm, 1};
    types::LidarScanResult scan = {{60.0 * cm, beam}};  // hit at 60 cm

    ScanResultToVoxels::applyToMap(map, origin, heading, scan, cfg);

    // Midpoint at 30 cm: (80, 100, 100) — well before the hit at 60 cm
    const Position3D mid{80.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    ASSERT_TRUE(map.isInBounds(mid));
    EXPECT_EQ(map.atVoxel(mid), types::VoxelOccupancy::Empty);
}

// Zero distance marks the z_min near segment as PotentiallyOccupied (not Occupied).
TEST(ScanResultToVoxels, ZeroDistanceMarksPotentiallyOccupied) {
    const std::size_t N = 20;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl map(arr, makeConfig(N));

    const Position3D origin{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    const Orientation heading{};
    const Orientation beam{};
    types::LidarConfigData cfg{20.0 * cm, 80.0 * cm, 2.5 * cm, 1};
    types::LidarScanResult scan = {{0.0 * cm, beam}};  // obstacle within z_min

    ScanResultToVoxels::applyToMap(map, origin, heading, scan, cfg);

    // Near point at 10 cm from origin (within z_min=20 cm): (110, 100, 100).
    // Must be PotentiallyOccupied, NOT Occupied (no false positive obstacles).
    const Position3D near{110.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    ASSERT_TRUE(map.isInBounds(near));
    EXPECT_EQ(map.atVoxel(near), types::VoxelOccupancy::PotentiallyOccupied);
    EXPECT_NE(map.atVoxel(near), types::VoxelOccupancy::Occupied);
}

// Zero distance must NOT mark the near segment as Occupied (no false positive).
TEST(ScanResultToVoxels, ZeroDistanceDoesNotMarkOccupied) {
    const std::size_t N = 20;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl map(arr, makeConfig(N));

    const Position3D origin{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    const Orientation heading{};
    const Orientation beam{};
    types::LidarConfigData cfg{20.0 * cm, 80.0 * cm, 2.5 * cm, 1};
    types::LidarScanResult scan = {{0.0 * cm, beam}};  // obstacle within z_min

    ScanResultToVoxels::applyToMap(map, origin, heading, scan, cfg);

    const Position3D near{110.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    ASSERT_TRUE(map.isInBounds(near));
    EXPECT_NE(map.atVoxel(near), types::VoxelOccupancy::Occupied);
}

// Occupied voxels are not downgraded to Empty by a subsequent miss beam.
// (Tests the priority system: Occupied=3 beats Empty=2.)
TEST(ScanResultToVoxels, OccupiedNotOverwrittenByEmpty) {
    const std::size_t N = 20;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl map(arr, makeConfig(N));

    const Position3D origin{50.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    const Orientation heading{};
    const Orientation beam{};
    types::LidarConfigData cfg{5.0 * cm, 80.0 * cm, 2.5 * cm, 1};

    // First: hit at 30 cm marks (80, 100, 100) as Occupied
    ScanResultToVoxels::applyToMap(map, origin, heading, {{30.0 * cm, beam}}, cfg);

    // Second: miss — attempts to mark the same region as Empty
    ScanResultToVoxels::applyToMap(map, origin, heading, {{missDistance(), beam}}, cfg);

    const Position3D occupied_pos{80.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    ASSERT_TRUE(map.isInBounds(occupied_pos));
    EXPECT_EQ(map.atVoxel(occupied_pos), types::VoxelOccupancy::Occupied);
}

// PotentiallyOccupied is upgraded to Occupied when a direct hit lands there.
// (Tests priority: Occupied=3 beats PotentiallyOccupied=1.)
TEST(ScanResultToVoxels, PotentiallyOccupiedUpgradedToOccupied) {
    const std::size_t N = 20;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl map(arr, makeConfig(N));

    const Position3D origin{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    const Orientation heading{};
    const Orientation beam{};
    types::LidarConfigData cfg{30.0 * cm, 80.0 * cm, 2.5 * cm, 1};

    // Zero-distance scan marks 0–30 cm as PotentiallyOccupied
    ScanResultToVoxels::applyToMap(map, origin, heading, {{0.0 * cm, beam}}, cfg);

    const Position3D near{115.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    ASSERT_TRUE(map.isInBounds(near));
    EXPECT_EQ(map.atVoxel(near), types::VoxelOccupancy::PotentiallyOccupied);

    // A hit at 15 cm should upgrade that voxel from PotentiallyOccupied to Occupied
    ScanResultToVoxels::applyToMap(map, origin, heading, {{15.0 * cm, beam}}, cfg);
    EXPECT_EQ(map.atVoxel(near), types::VoxelOccupancy::Occupied);
}

// A direct hit at a fresh Unmapped position marks it as Occupied.
TEST(ScanResultToVoxels, DirectHitProducesOccupied) {
    const std::size_t N = 20;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl map(arr, makeConfig(N));

    const Position3D origin{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    const Orientation heading{};
    const Orientation beam{};
    types::LidarConfigData cfg{5.0 * cm, 80.0 * cm, 2.5 * cm, 1};

    const Position3D near{115.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    ASSERT_TRUE(map.isInBounds(near));
    EXPECT_EQ(map.atVoxel(near), types::VoxelOccupancy::Unmapped);  // fresh map

    ScanResultToVoxels::applyToMap(map, origin, heading, {{15.0 * cm, beam}}, cfg);
    EXPECT_EQ(map.atVoxel(near), types::VoxelOccupancy::Occupied);
}

// Out-of-bounds scan origin does nothing: map remains fully Unmapped.
TEST(ScanResultToVoxels, OutOfBoundsOriginDoesNothing) {
    const std::size_t N = 5;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl map(arr, makeConfig(N));

    const Position3D outside{9999.0 * x_extent[cm], 9999.0 * y_extent[cm], 9999.0 * z_extent[cm]};
    types::LidarConfigData cfg{5.0 * cm, 50.0 * cm, 2.5 * cm, 1};
    types::LidarScanResult scan = {{10.0 * cm, Orientation{}}};

    EXPECT_NO_THROW(ScanResultToVoxels::applyToMap(map, outside, {}, scan, cfg));

    // Map interior must remain completely Unmapped (not Occupied or Empty)
    const Position3D p{25.0 * x_extent[cm], 25.0 * y_extent[cm], 25.0 * z_extent[cm]};
    EXPECT_EQ(map.atVoxel(p), types::VoxelOccupancy::Unmapped);
}

// Empty scan result leaves the map fully Unmapped.
TEST(ScanResultToVoxels, EmptyScanDoesNothing) {
    const std::size_t N = 5;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl map(arr, makeConfig(N));

    const Position3D origin{25.0 * x_extent[cm], 25.0 * y_extent[cm], 25.0 * z_extent[cm]};
    types::LidarConfigData cfg{5.0 * cm, 50.0 * cm, 2.5 * cm, 1};

    EXPECT_NO_THROW(ScanResultToVoxels::applyToMap(map, origin, {}, {}, cfg));
    EXPECT_EQ(map.atVoxel(origin), types::VoxelOccupancy::Unmapped);
}

// Drone heading is correctly added to the relative beam to form the absolute direction.
// Heading east (0 deg) and heading north (90 deg) produce hits at different positions.
TEST(ScanResultToVoxels, HeadingOffsetApplied) {
    const std::size_t N = 20;

    auto arr_a = std::make_shared<NpyArray>(N, N, N);
    auto arr_b = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl map_a(arr_a, makeConfig(N));
    Map3DImpl map_b(arr_b, makeConfig(N));

    const Position3D origin{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    const Orientation beam{};  // relative angle = 0
    types::LidarConfigData cfg{5.0 * cm, 60.0 * cm, 2.5 * cm, 1};
    types::LidarScanResult scan = {{40.0 * cm, beam}};

    // Heading east (0 deg): absolute beam = 0+0 = 0 → +x; hit at (140, 100, 100)
    ScanResultToVoxels::applyToMap(map_a, origin,
        {0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}, scan, cfg);

    // Heading north (90 deg): absolute beam = 90+0 = 90 → +y; hit at (100, 140, 100)
    ScanResultToVoxels::applyToMap(map_b, origin,
        {90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}, scan, cfg);

    const Position3D east_pos {140.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    const Position3D north_pos{100.0 * x_extent[cm], 140.0 * y_extent[cm], 100.0 * z_extent[cm]};

    ASSERT_TRUE(map_a.isInBounds(east_pos));
    EXPECT_EQ(map_a.atVoxel(east_pos),  types::VoxelOccupancy::Occupied);
    EXPECT_NE(map_a.atVoxel(north_pos), types::VoxelOccupancy::Occupied);

    ASSERT_TRUE(map_b.isInBounds(north_pos));
    EXPECT_EQ(map_b.atVoxel(north_pos), types::VoxelOccupancy::Occupied);
    EXPECT_NE(map_b.atVoxel(east_pos),  types::VoxelOccupancy::Occupied);
}

// A hit provided at exactly z_max distance is applied correctly.
// Catches bugs that truncate beam range before z_max (e.g., the "2/3 truncation" bug).
TEST(ScanResultToVoxels, HitAtFullZMaxApplied) {
    const std::size_t N = 20;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl map(arr, makeConfig(N));

    // Origin at (10, 100, 100), beam east, hit at z_max=80 cm → world (90, 100, 100)
    const Position3D origin{10.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    types::LidarConfigData cfg{5.0 * cm, 80.0 * cm, 2.5 * cm, 1};
    types::LidarScanResult scan = {{80.0 * cm, Orientation{}}};

    ScanResultToVoxels::applyToMap(map, origin, {}, scan, cfg);

    const Position3D hit_pos{90.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    ASSERT_TRUE(map.isInBounds(hit_pos));
    EXPECT_EQ(map.atVoxel(hit_pos), types::VoxelOccupancy::Occupied);
}

// Multiple beams with different directions produce independent hit marks.
TEST(ScanResultToVoxels, MultipleBeamsAppliedIndependently) {
    const std::size_t N = 20;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl map(arr, makeConfig(N));

    const Position3D origin{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    const Orientation heading{};
    types::LidarConfigData cfg{5.0 * cm, 60.0 * cm, 2.5 * cm, 1};

    const Orientation beam_east {0.0  * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    const Orientation beam_north{90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    types::LidarScanResult scan = {
        {40.0 * cm, beam_east},   // hit at (140, 100, 100)
        {40.0 * cm, beam_north},  // hit at (100, 140, 100)
    };

    ScanResultToVoxels::applyToMap(map, origin, heading, scan, cfg);

    const Position3D east_hit {140.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    const Position3D north_hit{100.0 * x_extent[cm], 140.0 * y_extent[cm], 100.0 * z_extent[cm]};

    ASSERT_TRUE(map.isInBounds(east_hit));
    ASSERT_TRUE(map.isInBounds(north_hit));
    EXPECT_EQ(map.atVoxel(east_hit),  types::VoxelOccupancy::Occupied);
    EXPECT_EQ(map.atVoxel(north_hit), types::VoxelOccupancy::Occupied);
}

// A hit position is NOT marked as Empty (i.e., the hit position gets Occupied, not Empty).
TEST(ScanResultToVoxels, HitPositionNotEmpty) {
    const std::size_t N = 20;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl map(arr, makeConfig(N));

    const Position3D origin{50.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    types::LidarConfigData cfg{5.0 * cm, 80.0 * cm, 2.5 * cm, 1};
    types::LidarScanResult scan = {{50.0 * cm, Orientation{}}};

    ScanResultToVoxels::applyToMap(map, origin, {}, scan, cfg);

    const Position3D hit_pos{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    ASSERT_TRUE(map.isInBounds(hit_pos));
    EXPECT_NE(map.atVoxel(hit_pos), types::VoxelOccupancy::Empty);
}
