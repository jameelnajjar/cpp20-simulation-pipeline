#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <TinyNPY.h>
#include <drone_mapper/SimulationRunImpl.h>

#include <filesystem>
#include <fstream>
#include <memory>

using namespace drone_mapper;

namespace {

types::MapConfig makeConfig(std::size_t n = 5, double res = 10.0) {
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

types::MissionConfigData makeMission() {
    types::MissionConfigData m;
    m.max_steps = 3;
    m.gps_resolution = 10.0 * cm;
    m.output_mapping_resolution_factor = 1.0;
    m.mission_bounds = makeConfig().boundaries;
    return m;
}

class ImmediateCompleteDroneCtrl : public IDroneControl {
public:
    types::DroneStepResult step() override { return {types::DroneStepStatus::Completed, {}}; }
    types::DroneState state() const override { return {}; }
};

class ImmediateCompleteMissionCtrl : public IMissionControl {
public:
    types::MissionRunResult runMission() override {
        if (!output_path_.empty()) {
            std::ofstream f(output_path_);
        }
        return types::MissionRunResult{types::MissionRunStatus::Completed, 1, {}};
    }
    std::filesystem::path output_path_;
};

class AlwaysWorkingAlgo : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;
    types::MappingStepCommand nextStep(const types::DroneState&, const types::LidarScanResult*) override {
        return types::MappingStepCommand{std::nullopt, std::nullopt, types::AlgorithmStatus::Finished};
    }
};

} // namespace

// run() returns a SimulationResult with the correct configs.
TEST(SimulationRun, RunReturnsCorrectConfigs) {
    const std::size_t N = 5;
    auto h_arr = std::make_shared<TinyNPY::Array>(N, N, N);
    auto o_arr = std::make_shared<TinyNPY::Array>(N, N, N);

    auto hidden = std::make_unique<Map3DImpl>(h_arr, makeConfig(N));
    auto output = std::make_unique<Map3DImpl>(o_arr, makeConfig(N));

    const Position3D center{25.0 * x_extent[cm], 25.0 * y_extent[cm], 25.0 * z_extent[cm]};
    auto gps = std::make_unique<MockGPS>(center, Orientation{}, 10.0 * cm);
    auto movement = std::make_unique<MockMovement>(*gps);

    types::LidarConfigData lidar_cfg{20.0 * cm, 150.0 * cm, 2.5 * cm, 1};
    auto lidar = std::make_unique<MockLidar>(lidar_cfg, *hidden, *gps);

    const auto drone_cfg = types::DroneConfigData{5.0 * cm, 90.0 * horizontal_angle[deg], 30.0 * cm, 20.0 * cm};
    auto algo = std::make_unique<AlwaysWorkingAlgo>(makeMission(), lidar_cfg, drone_cfg, *output);
    auto drone_ctrl = std::make_unique<ImmediateCompleteDroneCtrl>();

    auto mission_ctrl = std::make_unique<ImmediateCompleteMissionCtrl>();
    const auto tmp = std::filesystem::temp_directory_path() / "sim_run_test.npy";
    mission_ctrl->output_path_ = tmp;

    const types::SimulationConfigData sim_cfg;
    const types::MissionConfigData mission_cfg = makeMission();

    SimulationRunImpl run(
        std::move(hidden), std::move(output),
        std::move(gps), std::move(movement),
        std::move(lidar), std::move(algo),
        std::move(drone_ctrl), std::move(mission_ctrl),
        sim_cfg, mission_cfg, tmp);

    const auto result = run.run();
    EXPECT_EQ(result.mission_results.size(), 1u);
    // Score should be ≥ 0 for a valid run
    EXPECT_GE(result.mission_score, 0.0);

    std::filesystem::remove(tmp);
}

// Constructor throws on null dependency.
TEST(SimulationRun, ConstructorThrowsOnNull) {
    EXPECT_THROW({
        SimulationRunImpl run(
            nullptr, nullptr, nullptr, nullptr, nullptr,
            nullptr, nullptr, nullptr,
            types::SimulationConfigData{}, types::MissionConfigData{},
            std::filesystem::path{});
    }, std::invalid_argument);
}

// MockGPS is properly mutable via setPosition/setHeading.
TEST(SimulationRun, MockGPSMutability) {
    MockGPS gps(Position3D{}, Orientation{}, 10.0 * cm);
    const Position3D new_pos{50.0 * x_extent[cm], 60.0 * y_extent[cm], 70.0 * z_extent[cm]};
    gps.setPosition(new_pos);
    EXPECT_NEAR(gps.position().x.numerical_value_in(cm), 50.0, 0.01);
    EXPECT_NEAR(gps.position().y.numerical_value_in(cm), 60.0, 0.01);

    const Orientation new_heading{45.0 * horizontal_angle[deg], 10.0 * altitude_angle[deg]};
    gps.setHeading(new_heading);
    EXPECT_NEAR(gps.heading().horizontal.numerical_value_in(deg), 45.0, 0.01);
}

// MockMovement - rotate updates heading.
TEST(SimulationRun, MockMovementRotate) {
    MockGPS gps(Position3D{25.0 * x_extent[cm], 25.0 * y_extent[cm], 25.0 * z_extent[cm]},
                Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                10.0 * cm);
    MockMovement mv(gps);

    const auto result = mv.rotate(types::RotationDirection::Left, 45.0 * horizontal_angle[deg]);
    EXPECT_TRUE(result.success);
    EXPECT_NEAR(gps.heading().horizontal.numerical_value_in(deg), 45.0, 0.01);
}
