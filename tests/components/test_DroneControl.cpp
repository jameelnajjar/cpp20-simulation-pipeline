#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <TinyNPY.h>

using namespace drone_mapper;
using ::testing::Return;

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

types::MissionConfigData defaultMission(std::size_t n = 20, double res = 10.0) {
    types::MissionConfigData m;
    m.max_steps = 100;
    m.gps_resolution = res * cm;
    m.output_mapping_resolution_factor = 1.0;
    const double size = static_cast<double>(n) * res;
    m.mission_bounds = types::MappingBounds{
        0.0 * x_extent[cm], size * x_extent[cm],
        0.0 * y_extent[cm], size * y_extent[cm],
        0.0 * z_extent[cm], size * z_extent[cm],
    };
    return m;
}

// Minimal stub mapping algorithm that always returns Finished immediately.
class FinishingAlgorithm : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;
    types::MappingStepCommand nextStep(const types::DroneState&, const types::LidarScanResult*) override {
        return types::MappingStepCommand{std::nullopt, std::nullopt, types::AlgorithmStatus::Finished};
    }
};

// Stub algorithm that returns one scan then finishes.
class ScanThenFinishAlgorithm : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;
    types::MappingStepCommand nextStep(const types::DroneState&, const types::LidarScanResult*) override {
        if (calls_++ == 0) {
            return types::MappingStepCommand{
                std::nullopt,
                Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                types::AlgorithmStatus::Working,
            };
        }
        return types::MappingStepCommand{std::nullopt, std::nullopt, types::AlgorithmStatus::Finished};
    }
    int calls_ = 0;
};

} // namespace

// state() returns GPS position and heading.
TEST(DroneControl, StateReflectsGPS) {
    const std::size_t N = 10;
    auto hidden_arr = std::make_shared<TinyNPY::Array>(N, N, N);
    auto output_arr = std::make_shared<TinyNPY::Array>(N, N, N);
    Map3DImpl hidden_map(hidden_arr, makeConfig(N));
    Map3DImpl output_map(output_arr, makeConfig(N));

    const Position3D start{50.0 * x_extent[cm], 60.0 * y_extent[cm], 30.0 * z_extent[cm]};
    MockGPS gps(start, {}, 10.0 * cm);
    MockMovement movement(gps);

    types::LidarConfigData lidar_cfg{20.0 * cm, 150.0 * cm, 2.5 * cm, 1};
    MockLidar lidar_impl(lidar_cfg, hidden_map, gps);
    FinishingAlgorithm algo(defaultMission(N), lidar_cfg,
                             types::DroneConfigData{5.0 * cm, 90.0 * horizontal_angle[deg], 30.0 * cm, 20.0 * cm},
                             output_map);

    DroneControlImpl ctrl(
        types::DroneConfigData{5.0 * cm, 90.0 * horizontal_angle[deg], 30.0 * cm, 20.0 * cm},
        defaultMission(N),
        lidar_impl, gps, movement, output_map, algo);

    const auto state = ctrl.state();
    EXPECT_NEAR(state.position.x.numerical_value_in(cm), 50.0, 0.01);
    EXPECT_NEAR(state.position.y.numerical_value_in(cm), 60.0, 0.01);
    EXPECT_NEAR(state.position.z.numerical_value_in(cm), 30.0, 0.01);
}

// When algorithm says Finished, step() returns Completed.
TEST(DroneControl, StepReturnsCompletedWhenAlgorithmFinishes) {
    const std::size_t N = 10;
    auto hidden_arr = std::make_shared<TinyNPY::Array>(N, N, N);
    auto output_arr = std::make_shared<TinyNPY::Array>(N, N, N);
    Map3DImpl hidden_map(hidden_arr, makeConfig(N));
    Map3DImpl output_map(output_arr, makeConfig(N));

    MockGPS gps(Position3D{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]}, {}, 10.0 * cm);
    MockMovement movement(gps);

    types::LidarConfigData lidar_cfg{20.0 * cm, 150.0 * cm, 2.5 * cm, 1};
    MockLidar lidar_impl(lidar_cfg, hidden_map, gps);

    const auto drone_cfg = types::DroneConfigData{5.0 * cm, 90.0 * horizontal_angle[deg], 30.0 * cm, 20.0 * cm};
    FinishingAlgorithm algo(defaultMission(N), lidar_cfg, drone_cfg, output_map);

    DroneControlImpl ctrl(drone_cfg, defaultMission(N), lidar_impl, gps, movement, output_map, algo);

    const auto result = ctrl.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Completed);
}

// When algorithm requests a scan, step() applies it to map.
TEST(DroneControl, ScanAppliedToMap) {
    const std::size_t N = 20;
    auto hidden_arr = std::make_shared<TinyNPY::Array>(N, N, N);
    auto output_arr = std::make_shared<TinyNPY::Array>(N, N, N);
    Map3DImpl hidden_map(hidden_arr, makeConfig(N));
    Map3DImpl output_map(output_arr, makeConfig(N));

    const Position3D center{100.0 * x_extent[cm], 100.0 * y_extent[cm], 100.0 * z_extent[cm]};
    MockGPS gps(center, {}, 10.0 * cm);
    MockMovement movement(gps);

    types::LidarConfigData lidar_cfg{20.0 * cm, 150.0 * cm, 2.5 * cm, 1};
    MockLidar lidar_impl(lidar_cfg, hidden_map, gps);

    const auto drone_cfg = types::DroneConfigData{5.0 * cm, 90.0 * horizontal_angle[deg], 30.0 * cm, 20.0 * cm};
    ScanThenFinishAlgorithm algo(defaultMission(N), lidar_cfg, drone_cfg, output_map);

    DroneControlImpl ctrl(drone_cfg, defaultMission(N), lidar_impl, gps, movement, output_map, algo);

    // First step should scan; second should finish
    (void)ctrl.step();
    (void)ctrl.step();

    bool has_non_unmapped = false;
    for (double x = 10; x < 190 && !has_non_unmapped; x += 10) {
        for (double y = 10; y < 190 && !has_non_unmapped; y += 10) {
            for (double z = 10; z < 190 && !has_non_unmapped; z += 10) {
                const Position3D p{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
                if (output_map.isInBounds(p)) {
                    const auto v = output_map.atVoxel(p);
                    if (v != types::VoxelOccupancy::Unmapped) has_non_unmapped = true;
                }
            }
        }
    }
    EXPECT_TRUE(has_non_unmapped);
}
