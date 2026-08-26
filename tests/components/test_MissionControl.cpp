#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/NpyArray.h>

#include <filesystem>
#include <fstream>

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

types::MissionConfigData defaultMission() {
    types::MissionConfigData m;
    m.max_steps = 5;
    m.gps_resolution = 10.0 * cm;
    m.output_mapping_resolution_factor = 1.0;
    return m;
}

// Stub drone control that immediately completes.
class InstantCompleteDroneControl : public IDroneControl {
public:
    types::DroneStepResult step() override {
        return types::DroneStepResult{types::DroneStepStatus::Completed, "done"};
    }
    types::DroneState state() const override { return {}; }
};

// Stub drone control that always continues.
class AlwaysContinueDroneControl : public IDroneControl {
public:
    types::DroneStepResult step() override {
        ++steps;
        return types::DroneStepResult{types::DroneStepStatus::Continue, {}};
    }
    types::DroneState state() const override { return {}; }
    int steps = 0;
};

// Stub drone control that always errors.
class ErrorDroneControl : public IDroneControl {
public:
    types::DroneStepResult step() override {
        return types::DroneStepResult{types::DroneStepStatus::Error, "TEST_ERROR"};
    }
    types::DroneState state() const override { return {}; }
};

} // namespace

// Mission completes when drone says Completed on first step.
TEST(MissionControl, CompletesWhenDroneCompletes) {
    const std::size_t N = 5;
    auto hidden_arr = std::make_shared<NpyArray>(N, N, N);
    auto output_arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl hidden(hidden_arr, makeConfig(N));
    Map3DImpl output(output_arr, makeConfig(N));

    InstantCompleteDroneControl drone_ctrl;
    const auto tmp = std::filesystem::temp_directory_path() / "test_output_complete.npy";

    MissionControlImpl mc(defaultMission(),
                          types::DroneConfigData{5.0 * cm, 90.0 * horizontal_angle[deg], 30.0 * cm, 20.0 * cm},
                          hidden, output, drone_ctrl, tmp);

    const auto result = mc.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 1u);
}

// Mission hits max_steps if drone keeps continuing.
TEST(MissionControl, StopsAtMaxSteps) {
    const std::size_t N = 5;
    auto hidden_arr = std::make_shared<NpyArray>(N, N, N);
    auto output_arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl hidden(hidden_arr, makeConfig(N));
    Map3DImpl output(output_arr, makeConfig(N));

    AlwaysContinueDroneControl drone_ctrl;
    const auto tmp = std::filesystem::temp_directory_path() / "test_output_maxsteps.npy";

    auto mission = defaultMission();
    mission.max_steps = 3;

    MissionControlImpl mc(mission,
                          types::DroneConfigData{5.0 * cm, 90.0 * horizontal_angle[deg], 30.0 * cm, 20.0 * cm},
                          hidden, output, drone_ctrl, tmp);

    const auto result = mc.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(drone_ctrl.steps, 3);
}

// Mission returns Error when drone errors.
TEST(MissionControl, ReturnsErrorOnDroneError) {
    const std::size_t N = 5;
    auto hidden_arr = std::make_shared<NpyArray>(N, N, N);
    auto output_arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl hidden(hidden_arr, makeConfig(N));
    Map3DImpl output(output_arr, makeConfig(N));

    ErrorDroneControl drone_ctrl;
    const auto tmp = std::filesystem::temp_directory_path() / "test_output_error.npy";

    MissionControlImpl mc(defaultMission(),
                          types::DroneConfigData{5.0 * cm, 90.0 * horizontal_angle[deg], 30.0 * cm, 20.0 * cm},
                          hidden, output, drone_ctrl, tmp);

    const auto result = mc.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::Error);
    EXPECT_FALSE(result.errors.empty());
}

// Output map file is created after runMission.
TEST(MissionControl, OutputMapFileIsCreated) {
    const std::size_t N = 5;
    auto hidden_arr = std::make_shared<NpyArray>(N, N, N);
    auto output_arr = std::make_shared<NpyArray>(N, N, N);
    Map3DImpl hidden(hidden_arr, makeConfig(N));
    Map3DImpl output(output_arr, makeConfig(N));

    InstantCompleteDroneControl drone_ctrl;
    const auto tmp = std::filesystem::temp_directory_path() / "test_output_created.npy";
    std::filesystem::remove(tmp);

    MissionControlImpl mc(defaultMission(),
                          types::DroneConfigData{5.0 * cm, 90.0 * horizontal_angle[deg], 30.0 * cm, 20.0 * cm},
                          hidden, output, drone_ctrl, tmp);

    (void)mc.runMission();
    EXPECT_TRUE(std::filesystem::exists(tmp));
    std::filesystem::remove(tmp);
}
