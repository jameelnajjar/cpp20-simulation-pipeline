// SimulationRunFactoryImpl.cpp - composes one run from mocks + plugin factories.

#include <Simulator/SimulationRunFactoryImpl.h>
#include <Simulator/Map3DImpl.h>
#include <Simulator/MockGPS.h>
#include <Simulator/MockLidar.h>
#include <Simulator/MockMovement.h>
#include <Simulator/NpyArray.h>
#include <Simulator/SimulationRunImpl.h>
#include <UserCommon/GeometryUtils.h>
#include <UserCommon/TimeUtils.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace simulator {

namespace ucm = user_common_213309941_213727837;

SimulationRunFactoryImpl::SimulationRunFactoryImpl(
    common::MappingAlgorithmFactory algorithm_factory,
    common::MissionControlFactory mission_control_factory,
    std::string algorithm_label,
    std::string mission_control_label,
    bool verbose)
    : algorithm_factory_(std::move(algorithm_factory)),
      mission_control_factory_(std::move(mission_control_factory)),
      algorithm_label_(std::move(algorithm_label)),
      mission_control_label_(std::move(mission_control_label)),
      verbose_(verbose) {}

std::unique_ptr<ISimulationRun> SimulationRunFactoryImpl::create(
    const types::SimulationConfigData& simulation,
    const common::types::MissionConfigData& mission,
    const common::types::DroneConfigData& drone,
    const common::types::LidarConfigData& lidar,
    const std::filesystem::path& output_path) {

    auto hidden_npy = std::make_shared<NpyArray>(NpyArray::load(simulation.map_filename));
    common::types::MapConfig hidden_cfg;
    hidden_cfg.resolution = simulation.map_resolution;
    hidden_cfg.offset = simulation.map_offset;
    const double res = ucm::cmOf(simulation.map_resolution);
    hidden_cfg.boundaries = common::types::MappingBounds{
        ucm::xCm(0.0), ucm::xCm(static_cast<double>(hidden_npy->nx()) * res),
        ucm::yCm(0.0), ucm::yCm(static_cast<double>(hidden_npy->ny()) * res),
        ucm::zCm(0.0), ucm::zCm(static_cast<double>(hidden_npy->nz()) * res),
    };
    auto hidden_map = std::make_unique<Map3DImpl>(hidden_npy, hidden_cfg);

    double factor = mission.output_mapping_resolution_factor;
    if (factor < 1.0) { factor = 1.0; }
    const double out_res = std::max(res, ucm::cmOf(mission.gps_resolution) * factor);
    // Array origin is the mission-bounds minimum so a cropped output map indexes
    // (world - min_bound) / resolution. Using sim.map_offset here (HW2) mis-sizes
    // missions whose bounds do not start at 0 (e.g. small_mission_room min_y=90).
    common::types::MapConfig output_cfg;
    output_cfg.resolution = ucm::lengthCm(out_res);
    output_cfg.offset = common::Position3D{
        mission.mission_bounds.min_x,
        mission.mission_bounds.min_y,
        mission.mission_bounds.min_height,
    };
    output_cfg.boundaries = mission.mission_bounds;

    auto dim = [out_res](double a, double b) {
        return std::max(std::size_t{1},
                        static_cast<std::size_t>(std::ceil(std::abs(b - a) / std::max(out_res, 1e-9))));
    };
    auto output_npy = std::make_shared<NpyArray>(
        dim(ucm::cmOf(output_cfg.boundaries.min_x), ucm::cmOf(output_cfg.boundaries.max_x)),
        dim(ucm::cmOf(output_cfg.boundaries.min_y), ucm::cmOf(output_cfg.boundaries.max_y)),
        dim(ucm::cmOf(output_cfg.boundaries.min_height), ucm::cmOf(output_cfg.boundaries.max_height)));
    auto output_map = std::make_unique<Map3DImpl>(output_npy, output_cfg);

    auto gps = std::make_unique<MockGPS>(
        simulation.initial_drone_position,
        common::Orientation{simulation.initial_angle, ucm::pitchDeg(0.0)},
        mission.gps_resolution);
    auto movement = std::make_unique<MockMovement>(*gps, *hidden_map, drone.radius);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);

    auto algorithm = algorithm_factory_({mission, lidar, drone, *output_map});
    if (!algorithm) { throw std::runtime_error("algorithm factory returned null"); }

    const int run_index = run_counter_.fetch_add(1, std::memory_order_relaxed);
    const std::string map_name = "output_map_" + ucm::sanitizeForFilename(mission_control_label_) +
                                 "_" + ucm::sanitizeForFilename(algorithm_label_) + "_" +
                                 simulation.map_filename.stem().string() + "_" +
                                 ucm::uniqueStamp() + "_" + std::to_string(run_index) + ".npy";
    const std::filesystem::path output_map_file = output_path / map_name;

    auto mission_control = mission_control_factory_({
        mission, drone, *lidar_impl, *gps, *movement, *output_map, *algorithm, output_map_file,
        verbose_});

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map), std::move(output_map), std::move(gps), std::move(movement),
        std::move(lidar_impl), std::move(algorithm), std::move(mission_control), simulation, mission,
        output_map_file);
}

} // namespace simulator
