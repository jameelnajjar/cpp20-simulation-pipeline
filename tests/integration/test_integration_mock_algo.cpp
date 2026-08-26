#include <gtest/gtest.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/MapsComparison.h>
#include <TinyNPY.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/ISimulationRunFactory.h>
#include <drone_mapper/SimulationRunImpl.h>

#include <filesystem>
#include <memory>

using namespace drone_mapper;

namespace {

types::MapConfig makeConfig(std::size_t n = 8, double res = 10.0) {
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

// Mock algorithm: performs 4 scans from initial position then finishes.
class QuickScanAlgorithm : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;

    types::MappingStepCommand nextStep(const types::DroneState&, const types::LidarScanResult*) override {
        static const double angles[] = {0.0, 90.0, 180.0, 270.0};
        if (scan_count_ < 4) {
            const double angle_deg = angles[scan_count_++];
            return types::MappingStepCommand{
                std::nullopt,
                Orientation{angle_deg * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                types::AlgorithmStatus::Working,
            };
        }
        return types::MappingStepCommand{std::nullopt, std::nullopt, types::AlgorithmStatus::Finished};
    }
private:
    int scan_count_ = 0;
};

// Factory that creates runs with the QuickScanAlgorithm.
class QuickScanFactory : public ISimulationRunFactory {
public:
    std::unique_ptr<ISimulationRun> create(const types::SimulationConfigData& sim,
                                            const types::MissionConfigData& mission,
                                            const types::DroneConfigData& drone,
                                            const types::LidarConfigData& lidar,
                                            const std::filesystem::path& output_path) override {
        const std::size_t N = 8;
        const double RES = 10.0;
        const auto cfg = makeConfig(N, RES);

        auto h_arr = std::make_shared<TinyNPY::Array>(N, N, N);
        h_arr->set(5, 4, 4, true); // one obstacle
        auto o_arr = std::make_shared<TinyNPY::Array>(N, N, N);

        auto hidden = std::make_unique<Map3DImpl>(h_arr, cfg);
        auto output = std::make_unique<Map3DImpl>(o_arr, cfg);

        const Position3D start{40.0 * x_extent[cm], 40.0 * y_extent[cm], 40.0 * z_extent[cm]};
        auto gps = std::make_unique<MockGPS>(start, Orientation{}, RES * cm);
        auto movement = std::make_unique<MockMovement>(*gps);
        auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden, *gps);
        auto algo = std::make_unique<QuickScanAlgorithm>(mission, lidar, drone, *output);
        auto drone_ctrl = std::make_unique<DroneControlImpl>(drone, mission, *lidar_impl, *gps, *movement, *output, *algo);

        const auto out_file = output_path / "output_results" / "mock_algo_output.npy";
        std::filesystem::create_directories(out_file.parent_path());
        auto mc = std::make_unique<MissionControlImpl>(mission, drone, *hidden, *output, *drone_ctrl, out_file);

        return std::make_unique<SimulationRunImpl>(
            std::move(hidden), std::move(output),
            std::move(gps), std::move(movement), std::move(lidar_impl),
            std::move(algo), std::move(drone_ctrl), std::move(mc),
            sim, mission, out_file);
    }
};

} // namespace

// Integration with mock algorithm: full pipeline runs without errors.
TEST(Integration, MockAlgorithmCompletesSuccessfully) {
    const std::filesystem::path tmp = std::filesystem::temp_directory_path() / "hw2_mock_algo";
    std::filesystem::create_directories(tmp);

    types::SimulationCompositionData comp;
    comp.simulation_mission_groups.emplace_back(
        types::SimulationConfigData{},
        std::vector{types::MissionConfigData{100, 10.0 * cm, 1.0, makeConfig().boundaries}});
    comp.drones.push_back({5.0 * cm, 90.0 * horizontal_angle[deg], 30.0 * cm, 20.0 * cm});
    comp.lidars.push_back({10.0 * cm, 60.0 * cm, 5.0 * cm, 2});

    SimulationManager manager{std::make_unique<QuickScanFactory>()};
    const auto report = manager.run(comp, tmp);

    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_GE(report.runs[0].mission_score, 0.0);
    EXPECT_LE(report.runs[0].mission_score, 100.0);
    EXPECT_FALSE(report.runs[0].mission_results.empty());
    EXPECT_EQ(report.runs[0].mission_results[0].status, types::MissionRunStatus::Completed);

    std::filesystem::remove_all(tmp);
}

// Integration: multiple simulations each produce independent results.
TEST(Integration, MultipleSimulationsProduceIndependentResults) {
    const std::filesystem::path tmp = std::filesystem::temp_directory_path() / "hw2_multi_mock";
    std::filesystem::create_directories(tmp);

    types::SimulationCompositionData comp;
    for (int i = 0; i < 3; ++i) {
        comp.simulation_mission_groups.emplace_back(
            types::SimulationConfigData{},
            std::vector{types::MissionConfigData{50, 10.0 * cm, 1.0, makeConfig().boundaries}});
    }
    comp.drones.push_back({5.0 * cm, 90.0 * horizontal_angle[deg], 30.0 * cm, 20.0 * cm});
    comp.lidars.push_back({10.0 * cm, 60.0 * cm, 5.0 * cm, 2});

    SimulationManager manager{std::make_unique<QuickScanFactory>()};
    const auto report = manager.run(comp, tmp);

    EXPECT_EQ(report.runs.size(), 3u);
    for (const auto& r : report.runs) {
        EXPECT_GE(r.mission_score, 0.0);
        EXPECT_LE(r.mission_score, 100.0);
    }

    std::filesystem::remove_all(tmp);
}
