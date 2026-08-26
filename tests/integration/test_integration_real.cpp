#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/MapsComparison.h>
#include <TinyNPY.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <drone_mapper/SimulationManager.h>

#include <filesystem>
#include <memory>

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

} // namespace

// Integration test 1: Full flow with real mapping algorithm.
TEST(Integration, FullFlowWithRealAlgorithm) {
    const std::size_t N = 10;
    const double RES = 10.0;
    const types::MapConfig cfg = makeConfig(N, RES);

    auto hidden_arr = std::make_shared<TinyNPY::Array>(N, N, N);
    for (std::size_t y = 0; y < N; ++y) {
        for (std::size_t z = 0; z < N; ++z) {
            hidden_arr->set(7, y, z, true); // wall at x=7
        }
    }
    auto hidden_map = std::make_shared<Map3DImpl>(hidden_arr, cfg);
    auto output_arr = std::make_shared<TinyNPY::Array>(N, N, N);
    auto output_map = std::make_shared<Map3DImpl>(output_arr, cfg);

    types::MissionConfigData mission;
    mission.max_steps = 200;
    mission.gps_resolution = RES * cm;
    mission.output_mapping_resolution_factor = 1.0;
    mission.mission_bounds = cfg.boundaries;

    const types::DroneConfigData drone{15.0 * cm, 90.0 * horizontal_angle[deg], 30.0 * cm, 20.0 * cm};
    const types::LidarConfigData lidar_cfg{10.0 * cm, 80.0 * cm, 5.0 * cm, 2};

    const Position3D start{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    auto gps = std::make_shared<MockGPS>(start, Orientation{}, RES * cm);
    auto movement = std::make_shared<MockMovement>(*gps);
    auto lidar = std::make_shared<MockLidar>(lidar_cfg, *hidden_map, *gps);
    auto algo = std::make_shared<MappingAlgorithmImpl>(mission, lidar_cfg, drone, *output_map);
    auto drone_ctrl = std::make_shared<DroneControlImpl>(drone, mission, *lidar, *gps, *movement, *output_map, *algo);

    const auto tmp = std::filesystem::temp_directory_path() / "integration_test1_output.npy";
    MissionControlImpl mc(mission, drone, *hidden_map, *output_map, *drone_ctrl, tmp);

    const auto result = mc.runMission();

    // Mission should complete without unrecoverable errors.
    EXPECT_NE(result.status, types::MissionRunStatus::Error);

    // Output map should have some non-Unmapped voxels after running.
    bool has_mapped = false;
    for (std::size_t x = 0; x < N && !has_mapped; ++x) {
        for (std::size_t y = 0; y < N && !has_mapped; ++y) {
            for (std::size_t z = 0; z < N && !has_mapped; ++z) {
                const Position3D p{
                    (x * RES + RES * 0.5) * x_extent[cm],
                    (y * RES + RES * 0.5) * y_extent[cm],
                    (z * RES + RES * 0.5) * z_extent[cm]
                };
                if (output_map->isInBounds(p)) {
                    const auto occ = output_map->atVoxel(p);
                    if (occ != types::VoxelOccupancy::Unmapped) has_mapped = true;
                }
            }
        }
    }
    EXPECT_TRUE(has_mapped);

    // Score vs hidden map should be non-negative.
    std::vector<IMap3D*> targets = {output_map.get()};
    const auto scores = MapsComparison::compare(*hidden_map, targets);
    EXPECT_GE(scores[0], 0.0);
    EXPECT_LE(scores[0], 100.0);

    std::filesystem::remove(tmp);
}

// Integration test 2: SimulationManager full cartesian product with real components.
TEST(Integration, SimulationManagerFullPipeline) {
    const std::filesystem::path tmp_dir = std::filesystem::temp_directory_path() / "hw2_integration";
    std::filesystem::create_directories(tmp_dir / "maps");

    // Build a 5x5x5 npy with one occupied voxel.
    TinyNPY::Array tiny_map(5, 5, 5);
    tiny_map.set(2, 2, 2, true);
    tiny_map.save(tmp_dir / "maps" / "tiny.npy");

    // Build composition
    types::SimulationConfigData sim_cfg;
    sim_cfg.map_filename = tmp_dir / "maps" / "tiny.npy";
    sim_cfg.map_resolution = 10.0 * cm;
    sim_cfg.initial_drone_position = Position3D{25.0 * x_extent[cm], 25.0 * y_extent[cm], 25.0 * z_extent[cm]};
    sim_cfg.initial_angle = 0.0 * horizontal_angle[deg];

    types::MissionConfigData mission;
    mission.max_steps = 50;
    mission.gps_resolution = 10.0 * cm;
    mission.output_mapping_resolution_factor = 1.0;
    mission.mission_bounds = types::MappingBounds{
        0.0 * x_extent[cm], 50.0 * x_extent[cm],
        0.0 * y_extent[cm], 50.0 * y_extent[cm],
        0.0 * z_extent[cm], 50.0 * z_extent[cm],
    };

    types::SimulationCompositionData comp;
    comp.composition_file = "test";
    comp.simulation_mission_groups.emplace_back(sim_cfg, std::vector{mission});
    comp.drones.push_back({15.0 * cm, 90.0 * horizontal_angle[deg], 30.0 * cm, 20.0 * cm});
    comp.lidars.push_back({10.0 * cm, 60.0 * cm, 5.0 * cm, 2});

    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager{std::move(factory)};

    const types::SimulationManagerReport report = manager.run(comp, tmp_dir);

    EXPECT_EQ(report.runs.size(), 1u);
    EXPECT_GE(report.runs[0].mission_score, 0.0);

    std::filesystem::remove_all(tmp_dir);
}
