#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/NpyArray.h>

#include <limits>

using namespace drone_mapper;

namespace {

types::MapConfig makeConfig(std::size_t n = 20, double res = 10.0) {
    types::MapConfig cfg;
    cfg.resolution = res * cm;
    cfg.offset = Position3D{};
    const double size = static_cast<double>(n) * res;
    cfg.boundaries = types::MappingBounds{
        0.0 * x_extent[cm], size * x_extent[cm],
        0.0 * y_extent[cm], size * y_extent[cm],
        0.0 * z_extent[cm], size * z_extent[cm],
    };
    return cfg;
}

types::LidarConfigData defaultLidar() {
    return types::LidarConfigData{20.0 * cm, 150.0 * cm, 2.5 * cm, 3};
}

} // namespace

// config() returns the config passed at construction.
TEST(MockLidar, ConfigRoundtrip) {
    const std::size_t N = 10;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl map(arr, makeConfig(N));
    MockGPS gps({}, {}, 10.0 * cm);
    const auto cfg = defaultLidar();
    MockLidar lidar(cfg, map, gps);
    const auto ret = lidar.config();
    EXPECT_EQ(ret.fov_circles, cfg.fov_circles);
    EXPECT_DOUBLE_EQ(ret.z_max.numerical_value_in(cm), cfg.z_max.numerical_value_in(cm));
}

// Scan in empty map returns miss distance for all beams.
TEST(MockLidar, ScanInEmptyMapReturnsMiss) {
    const std::size_t N = 20;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl map(arr, makeConfig(N));

    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    MockGPS gps(start, {}, 10.0 * cm);
    MockLidar lidar(defaultLidar(), map, gps);

    const Orientation scan_dir{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    const auto result = lidar.scan(scan_dir);

    EXPECT_FALSE(result.empty());
    for (const auto& hit : result) {
        EXPECT_DOUBLE_EQ(hit.distance.numerical_value_in(cm),
                         std::numeric_limits<double>::max());
    }
}

// Scan toward occupied voxel returns finite distance.
TEST(MockLidar, ScanHitsOccupiedVoxel) {
    const std::size_t N = 20;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    // Place obstacle at (15, 10, 10) in world = index (15,10,10) with res=10
    arr->set(15, 10, 10, true);
    Map3DImpl map(arr, makeConfig(N));

    // Drone at (100, 100, 100) cm
    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    MockGPS gps(start, {}, 10.0 * cm);

    types::LidarConfigData lidar_cfg{5.0 * cm, 200.0 * cm, 2.5 * cm, 1};
    MockLidar lidar(lidar_cfg, map, gps);

    // Scan East (angle 0) - should hit the block at x=150
    const Orientation scan_dir{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    const auto result = lidar.scan(scan_dir);

    ASSERT_GE(result.size(), 1u);
    const double dist_cm = result[0].distance.numerical_value_in(cm);
    EXPECT_LT(dist_cm, std::numeric_limits<double>::max());
    EXPECT_GT(dist_cm, 0.0);
}

// fov_circles=0 returns empty result.
TEST(MockLidar, ZeroFovCirclesReturnsEmpty) {
    const std::size_t N = 5;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl map(arr, makeConfig(N));
    MockGPS gps({}, {}, 10.0 * cm);
    MockLidar lidar({20.0 * cm, 120.0 * cm, 2.5 * cm, 0}, map, gps);
    const auto result = lidar.scan({});
    EXPECT_TRUE(result.empty());
}

// Scan results contain at least 1 + (4+16) beams for fov_circles=3.
TEST(MockLidar, ScanResultSizeMatchesFovCircles) {
    const std::size_t N = 20;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl map(arr, makeConfig(N));
    const Position3D center{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    MockGPS gps(center, {}, 10.0 * cm);
    types::LidarConfigData cfg{20.0 * cm, 150.0 * cm, 2.5 * cm, 3};
    MockLidar lidar(cfg, map, gps);
    const auto result = lidar.scan({});
    // 1 + 4 + 16 = 21 beams for 3 circles
    EXPECT_EQ(result.size(), 21u);
}

// fov_circles=1 returns exactly 1 beam (center beam only).
TEST(MockLidar, FovCircles1ReturnsOneBeam) {
    const std::size_t N = 10;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl map(arr, makeConfig(N));
    const Position3D center{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    MockGPS gps(center, {}, 10.0 * cm);
    MockLidar lidar({5.0 * cm, 80.0 * cm, 2.5 * cm, 1}, map, gps);
    EXPECT_EQ(lidar.scan({}).size(), 1u);
}

// fov_circles=2 returns 1 + 4 = 5 beams.
TEST(MockLidar, FovCircles2ReturnsFiveBeams) {
    const std::size_t N = 20;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl map(arr, makeConfig(N));
    const Position3D center{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    MockGPS gps(center, {}, 10.0 * cm);
    MockLidar lidar({5.0 * cm, 80.0 * cm, 2.5 * cm, 2}, map, gps);
    // Circle 0 (center) = 1, circle 1 = 4 beams → total 5
    EXPECT_EQ(lidar.scan({}).size(), 5u);
}

// fov_circles=4 returns 1 + 4 + 16 + 64 = 85 beams.
TEST(MockLidar, FovCircles4Returns85Beams) {
    const std::size_t N = 20;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl map(arr, makeConfig(N));
    const Position3D center{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    MockGPS gps(center, {}, 10.0 * cm);
    MockLidar lidar({5.0 * cm, 80.0 * cm, 2.5 * cm, 4}, map, gps);
    // 1 + 4 + 16 + 64 = 85 beams
    EXPECT_EQ(lidar.scan({}).size(), 85u);
}

// Obstacle closer than z_min returns 0 distance (blocked zone).
TEST(MockLidar, ObstacleBelowZMinReturnsZeroDistance) {
    const std::size_t N = 20;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    // Drone at x=100cm, obstacle at index 11 (110-120cm) → first hit at distance=10cm
    arr->set(11, 10, 10, true);
    Map3DImpl map(arr, makeConfig(N));

    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    MockGPS gps(start, {}, 10.0 * cm);
    // z_min=40cm; hit at 10cm < 40cm → should return 0
    types::LidarConfigData cfg{40.0 * cm, 200.0 * cm, 2.5 * cm, 1};
    MockLidar lidar(cfg, map, gps);

    const auto result = lidar.scan({});
    ASSERT_GE(result.size(), 1u);
    EXPECT_DOUBLE_EQ(result[0].distance.numerical_value_in(cm), 0.0);
}

// Obstacle exactly at z_min boundary returns finite distance (not 0).
TEST(MockLidar, ObstacleAtExactlyZMinReturnsFiniteDistance) {
    const std::size_t N = 20;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    // Drone at x=100cm, obstacle at index 14 (140-150cm) → first hit at distance=40cm
    arr->set(14, 10, 10, true);
    Map3DImpl map(arr, makeConfig(N));

    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    MockGPS gps(start, {}, 10.0 * cm);
    // z_min=40cm; hit at 40cm: condition is (40 < 40) = false → returns distance
    types::LidarConfigData cfg{40.0 * cm, 200.0 * cm, 2.5 * cm, 1};
    MockLidar lidar(cfg, map, gps);

    const auto result = lidar.scan({});
    ASSERT_GE(result.size(), 1u);
    const double dist = result[0].distance.numerical_value_in(cm);
    EXPECT_GT(dist, 0.0);
    EXPECT_LT(dist, std::numeric_limits<double>::max());
    EXPECT_NEAR(dist, 40.0, 2.0);  // within 2 step-sizes of 40cm
}

// Obstacle at exactly z_max distance is detected (boundary-inclusive loop).
TEST(MockLidar, ObstacleAtZMaxBoundaryIsDetected) {
    const std::size_t N = 20;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    // Drone at x=100cm, obstacle at index 18 (180-190cm) → first hit at distance=80cm
    arr->set(18, 10, 10, true);
    Map3DImpl map(arr, makeConfig(N));

    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    MockGPS gps(start, {}, 10.0 * cm);
    // z_max=80cm; the loop uses `distance <= z_max`, so 80 <= 80 samples at ix=18
    types::LidarConfigData cfg{5.0 * cm, 80.0 * cm, 2.5 * cm, 1};
    MockLidar lidar(cfg, map, gps);

    const auto result = lidar.scan({});
    ASSERT_GE(result.size(), 1u);
    // Must be a finite hit, not a miss
    EXPECT_LT(result[0].distance.numerical_value_in(cm), std::numeric_limits<double>::max());
}

// Obstacle at 77% of z_max is detected — catches the "2/3 truncation" bug.
// If a bug limits beams to 2/3*z_max (60cm) instead of 90cm, this 70cm hit is missed.
TEST(MockLidar, ObstacleAtSeventyPercentZMaxIsDetected) {
    const std::size_t N = 20;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    // Drone at x=100cm, obstacle at index 17 (170-180cm) → first hit at distance=70cm
    // z_max=90cm → 70 <= 90 ✓; 2/3-bug z_max=60cm → 70 > 60 ✗
    arr->set(17, 10, 10, true);
    Map3DImpl map(arr, makeConfig(N));

    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    MockGPS gps(start, {}, 10.0 * cm);
    types::LidarConfigData cfg{5.0 * cm, 90.0 * cm, 2.5 * cm, 1};
    MockLidar lidar(cfg, map, gps);

    const auto result = lidar.scan({});
    ASSERT_GE(result.size(), 1u);
    const double dist = result[0].distance.numerical_value_in(cm);
    EXPECT_LT(dist, std::numeric_limits<double>::max());
    EXPECT_NEAR(dist, 70.0, 2.0);
}

// Sensor heading rotates the absolute beam direction.
// Obstacle east of the drone should be missed when heading is north (90 deg).
TEST(MockLidar, HeadingRotatesAbsoluteBeamDirection) {
    const std::size_t N = 20;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    // Obstacle east of drone at index (15, 10, 10)
    arr->set(15, 10, 10, true);
    Map3DImpl map(arr, makeConfig(N));

    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    // Heading 90 deg → sensor faces north (+y direction)
    MockGPS gps(start, Orientation{90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}, 10.0 * cm);
    types::LidarConfigData cfg{5.0 * cm, 100.0 * cm, 2.5 * cm, 1};
    MockLidar lidar(cfg, map, gps);

    // scan({0,0}): absolute beam = heading(90) + scan(0) = 90 deg → travels +y, not +x
    // The obstacle is to the east (+x), so this should miss
    const auto result = lidar.scan({});
    ASSERT_GE(result.size(), 1u);
    EXPECT_DOUBLE_EQ(result[0].distance.numerical_value_in(cm),
                     std::numeric_limits<double>::max());
}

// When heading is 0 (east), the same obstacle IS detected.
TEST(MockLidar, ZeroHeadingDetectsEastObstacle) {
    const std::size_t N = 20;
    auto arr = std::make_shared<NpyArray>(N, N, N);
    arr->set(15, 10, 10, true);
    Map3DImpl map(arr, makeConfig(N));

    const Position3D start{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    MockGPS gps(start, Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]}, 10.0 * cm);
    types::LidarConfigData cfg{5.0 * cm, 100.0 * cm, 2.5 * cm, 1};
    MockLidar lidar(cfg, map, gps);

    const auto result = lidar.scan({});
    ASSERT_GE(result.size(), 1u);
    EXPECT_LT(result[0].distance.numerical_value_in(cm), std::numeric_limits<double>::max());
}
