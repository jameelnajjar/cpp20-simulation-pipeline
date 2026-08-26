#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationRunImpl.h>

#include "ErrorHandler.h"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace drone_mapper {

namespace {

// Build the hidden map config from simulation config.
types::MapConfig buildHiddenMapConfig(const types::SimulationConfigData& sim,
                                       const TinyNPY::Array& arr) {
    const double res = sim.map_resolution.numerical_value_in(cm);
    types::MapConfig cfg;
    cfg.resolution = sim.map_resolution;
    cfg.offset = sim.map_offset;
    cfg.boundaries = types::MappingBounds{
        0.0 * x_extent[cm],
        static_cast<double>(arr.nx()) * res * x_extent[cm],
        0.0 * y_extent[cm],
        static_cast<double>(arr.ny()) * res * y_extent[cm],
        0.0 * z_extent[cm],
        static_cast<double>(arr.nz()) * res * z_extent[cm],
    };
    return cfg;
}

// Build the output map config from mission config and simulation config.
types::MapConfig buildOutputMapConfig(const types::MissionConfigData& mission,
                                       const types::SimulationConfigData& sim) {
    double factor = mission.output_mapping_resolution_factor;
    if (factor < 1.0) factor = 1.0;

    const double base_res = sim.map_resolution.numerical_value_in(cm);
    const double out_res = std::max(base_res, mission.gps_resolution.numerical_value_in(cm) * factor);

    const auto& b = mission.mission_bounds;
    types::MapConfig cfg;
    cfg.resolution = out_res * cm;
    cfg.offset = sim.map_offset;
    cfg.boundaries = b;
    return cfg;
}

std::size_t computeNpyDim(double range_cm, double res_cm) {
    if (res_cm <= 0) return 1;
    return static_cast<std::size_t>(std::ceil(range_cm / res_cm));
}

std::string currentUtcTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << std::put_time(std::gmtime(&t), "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}

} // namespace

std::unique_ptr<ISimulationRun>
SimulationRunFactoryImpl::create(const types::SimulationConfigData& simulation,
                                  const types::MissionConfigData& mission,
                                  const types::DroneConfigData& drone,
                                  const types::LidarConfigData& lidar,
                                  const std::filesystem::path& output_path) {
    // --- Load hidden map ---
    std::shared_ptr<TinyNPY::Array> hidden_npy;
    try {
        hidden_npy = std::make_shared<TinyNPY::Array>(TinyNPY::Array::load(simulation.map_filename));
    } catch (const std::exception& e) {
        ErrorHandler::instance().logError("MAP_LOAD_ERROR", e.what());
        throw;
    }

    const types::MapConfig hidden_cfg = buildHiddenMapConfig(simulation, *hidden_npy);
    auto hidden_map = std::make_unique<Map3DImpl>(hidden_npy, hidden_cfg);

    // --- Create output map ---
    const types::MapConfig output_cfg = buildOutputMapConfig(mission, simulation);
    const double out_res = output_cfg.resolution.numerical_value_in(cm);

    const double range_x = output_cfg.boundaries.max_x.numerical_value_in(cm)
                         - output_cfg.boundaries.min_x.numerical_value_in(cm);
    const double range_y = output_cfg.boundaries.max_y.numerical_value_in(cm)
                         - output_cfg.boundaries.min_y.numerical_value_in(cm);
    const double range_z = output_cfg.boundaries.max_height.numerical_value_in(cm)
                         - output_cfg.boundaries.min_height.numerical_value_in(cm);

    const std::size_t nx = std::max(std::size_t{1}, computeNpyDim(range_x, out_res));
    const std::size_t ny = std::max(std::size_t{1}, computeNpyDim(range_y, out_res));
    const std::size_t nz = std::max(std::size_t{1}, computeNpyDim(range_z, out_res));

    auto output_npy = std::make_shared<TinyNPY::Array>(nx, ny, nz);
    auto output_map = std::make_unique<Map3DImpl>(output_npy, output_cfg);

    // --- Create GPS ---
    auto gps = std::make_unique<MockGPS>(
        simulation.initial_drone_position,
        Orientation{simulation.initial_angle, 0.0 * altitude_angle[deg]},
        mission.gps_resolution);

    // --- Create movement ---
    auto movement = std::make_unique<MockMovement>(*gps);

    // --- Create lidar ---
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);

    // --- Create mapping algorithm ---
    auto mapping_algorithm = std::make_unique<MappingAlgorithmImpl>(mission, lidar, drone, *output_map);

    // --- Create drone control ---
    auto drone_control = std::make_unique<DroneControlImpl>(
        drone, mission, *lidar_impl, *gps, *movement, *output_map, *mapping_algorithm);

    // --- Create output path ---
    const int run_index = run_counter_.fetch_add(1, std::memory_order_relaxed);
    const std::string map_filename = "output_map_"
        + simulation.map_filename.stem().string()
        + "_" + currentUtcTimestamp().substr(0, 10)
        + "_" + std::to_string(run_index) + ".npy";
    const std::filesystem::path output_map_file = output_path / "output_results" / map_filename;

    // --- Create mission control ---
    auto mission_control = std::make_unique<MissionControlImpl>(
        mission, drone, *hidden_map, *output_map, *drone_control, output_map_file);

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar_impl),
        std::move(mapping_algorithm),
        std::move(drone_control),
        std::move(mission_control),
        simulation,
        mission,
        output_map_file);
}

} // namespace drone_mapper
