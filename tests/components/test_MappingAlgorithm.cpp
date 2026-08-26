#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/NpyArray.h>

using namespace drone_mapper;

namespace {

types::MapConfig makeConfig(std::size_t n = 10, double res = 10.0) {
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

types::MissionConfigData defaultMission(std::size_t n = 10, double res = 10.0) {
    types::MissionConfigData m;
    m.max_steps = 500;
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

types::LidarConfigData defaultLidar() {
    return types::LidarConfigData{20.0 * cm, 150.0 * cm, 2.5 * cm, 3};
}

types::DroneConfigData defaultDrone() {
    return types::DroneConfigData{15.0 * cm, 90.0 * horizontal_angle[deg], 50.0 * cm, 40.0 * cm};
}

} // namespace

// First nextStep call returns a scan command (initial scanning phase).
TEST(MappingAlgorithm, FirstCallReturnsScan) {
    auto arr = std::make_shared<NpyArray>(10, 10, 10);
    Map3DImpl map(arr, makeConfig());

    MappingAlgorithmImpl algo(defaultMission(), defaultLidar(), defaultDrone(), map);

    const types::DroneState state{
        Position3D{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
        Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
        0
    };

    const auto cmd = algo.nextStep(state, nullptr);
    EXPECT_NE(cmd.status, types::AlgorithmStatus::Finished);
    // First calls should include a scan orientation or movement
    EXPECT_TRUE(cmd.scan_orientation.has_value() || cmd.movement.has_value());
}

// Algorithm eventually returns Finished for a trivially small map.
TEST(MappingAlgorithm, EventuallyFinishes) {
    auto arr = std::make_shared<NpyArray>(3, 3, 3);
    Map3DImpl map(arr, makeConfig(3));

    MappingAlgorithmImpl algo(defaultMission(3), defaultLidar(), defaultDrone(), map);

    const types::DroneState state{
        Position3D{15.0 * x_extent[cm], 15.0 * y_extent[cm], 15.0 * z_extent[cm]},
        Orientation{},
        0
    };

    bool finished = false;
    for (int i = 0; i < 200 && !finished; ++i) {
        const auto cmd = algo.nextStep(state, nullptr);
        if (cmd.status == types::AlgorithmStatus::Finished ||
            cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
            finished = true;
        }
    }
    EXPECT_TRUE(finished);
}

// Movement commands respect drone config limits.
TEST(MappingAlgorithm, MovementCommandRespectsDroneLimits) {
    auto arr = std::make_shared<NpyArray>(10, 10, 10);
    Map3DImpl map(arr, makeConfig());

    const types::DroneConfigData drone{15.0 * cm, 90.0 * horizontal_angle[deg], 50.0 * cm, 40.0 * cm};
    MappingAlgorithmImpl algo(defaultMission(), defaultLidar(), drone, map);

    const types::DroneState state{
        Position3D{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]},
        Orientation{},
        0
    };

    for (int i = 0; i < 50; ++i) {
        const auto cmd = algo.nextStep(state, nullptr);
        if (cmd.movement.has_value()) {
            const auto& mv = cmd.movement.value();
            if (mv.type == types::MovementCommandType::Advance) {
                EXPECT_LE(std::abs(mv.distance.numerical_value_in(cm)), 50.0 + 0.001);
            }
            if (mv.type == types::MovementCommandType::Elevate) {
                EXPECT_LE(std::abs(mv.distance.numerical_value_in(cm)), 40.0 + 0.001);
            }
            if (mv.type == types::MovementCommandType::Rotate) {
                EXPECT_LE(std::abs(mv.angle.numerical_value_in(deg)), 90.0 + 0.001);
            }
        }
        if (cmd.status == types::AlgorithmStatus::Finished) break;
    }
}

// After a scan, algorithm uses scan data to queue neighbors (test that further calls don't crash).
TEST(MappingAlgorithm, HandlesNullScan) {
    auto arr = std::make_shared<NpyArray>(5, 5, 5);
    Map3DImpl map(arr, makeConfig(5));
    MappingAlgorithmImpl algo(defaultMission(5), defaultLidar(), defaultDrone(), map);

    const types::DroneState state{
        Position3D{25.0 * x_extent[cm], 25.0 * y_extent[cm], 25.0 * z_extent[cm]},
        Orientation{},
        0
    };
    EXPECT_NO_THROW({
        for (int i = 0; i < 20; ++i) {
            (void)algo.nextStep(state, nullptr);
        }
    });
}
