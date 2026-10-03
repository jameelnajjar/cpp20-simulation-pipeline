// YamlParser.cpp - YAML config readers. Ported from assignment 2 YamlParser.

#include <Simulator/YamlParser.h>
#include <UserCommon/GeometryUtils.h>

#include <yaml-cpp/yaml.h>

#include <system_error>

namespace simulator {

namespace ucm = user_common_213309941_213727837;

namespace {

common::types::MappingBounds parseBoundaries(const YAML::Node& node) {
    common::types::MappingBounds b;
    const auto& x_bnd = node["x_boundary"];
    const auto& y_bnd = node["y_boundary"];
    const auto& h_bnd = node["height_boundary"];
    b.min_x = ucm::xCm(x_bnd["min_cm"].as<double>());
    b.max_x = ucm::xCm(x_bnd["max_cm"].as<double>());
    b.min_y = ucm::yCm(y_bnd["min_cm"].as<double>());
    b.max_y = ucm::yCm(y_bnd["max_cm"].as<double>());
    b.min_height = ucm::zCm(h_bnd["min_cm"].as<double>());
    b.max_height = ucm::zCm(h_bnd["max_cm"].as<double>());
    return b;
}

} // namespace

// Parses one assignment-2 drone YAML into common::types::DroneConfigData.
common::types::DroneConfigData YamlParser::parseDroneConfig(const std::filesystem::path& path) {
    const YAML::Node n = YAML::LoadFile(path.string())["drone_config"];
    common::types::DroneConfigData cfg;
    if (n["dimensions_cm"]) {
        cfg.radius = ucm::lengthCm(n["dimensions_cm"].as<double>() / 2.0);
    } else {
        cfg.radius = ucm::lengthCm(n["radius_cm"].as<double>());
    }
    cfg.max_rotate = ucm::yawDeg(n["max_rotate_deg"].as<double>());
    cfg.max_advance = ucm::lengthCm(n["max_advance_cm"].as<double>());
    cfg.max_elevate = ucm::lengthCm(n["max_elevate_cm"].as<double>());
    return cfg;
}

// Parses one assignment-2 lidar YAML into common::types::LidarConfigData.
common::types::LidarConfigData YamlParser::parseLidarConfig(const std::filesystem::path& path) {
    const YAML::Node n = YAML::LoadFile(path.string())["lidar_config"];
    common::types::LidarConfigData cfg;
    cfg.z_min = ucm::lengthCm(n["z_min_cm"].as<double>());
    cfg.z_max = ucm::lengthCm(n["z_max_cm"].as<double>());
    cfg.d = ucm::lengthCm(n["d_cm"].as<double>());
    cfg.fov_circles = n["fov_circles"].as<std::size_t>();
    return cfg;
}

// Parses one assignment-2 mission YAML into common::types::MissionConfigData.
common::types::MissionConfigData YamlParser::parseMissionConfig(const std::filesystem::path& path) {
    const YAML::Node n = YAML::LoadFile(path.string())["mission_config"];
    common::types::MissionConfigData cfg;
    cfg.max_steps = n["max_steps"].as<std::size_t>();
    cfg.gps_resolution = ucm::lengthCm(n["gps_resolution_cm"].as<double>());
    cfg.output_mapping_resolution_factor =
        n["output_mapping_resolution_factor"] ? n["output_mapping_resolution_factor"].as<double>()
                                              : 1.0;
    if (n["boundaries"]) { cfg.mission_bounds = parseBoundaries(n["boundaries"]); }
    return cfg;
}

// Parses one assignment-2 simulation YAML into simulator::types::SimulationConfigData.
types::SimulationConfigData YamlParser::parseSimulationConfig(const std::filesystem::path& path) {
    const YAML::Node n = YAML::LoadFile(path.string())["simulation_config"];
    types::SimulationConfigData cfg;
    cfg.map_filename = n["map_filename"].as<std::string>();
    cfg.map_resolution = ucm::lengthCm(n["map_resolution_cm"].as<double>());
    const auto& dpos = n["initial_drone_position"];
    cfg.initial_drone_position = ucm::positionCm(dpos["x_cm"].as<double>(), dpos["y_cm"].as<double>(),
                                                 dpos["height_cm"].as<double>());
    cfg.initial_angle = ucm::yawDeg(n["initial_angle_deg"].as<double>());
    if (n["map_axes_offset"]) {
        const auto& off = n["map_axes_offset"];
        cfg.map_offset = ucm::positionCm(off["x_offset"].as<double>(), off["y_offset"].as<double>(),
                                         off["height_offset"].as<double>());
    }
    return cfg;
}

// Parses the composition YAML that drives comparative / competitive cartesian products.
types::SimulationCompositionData YamlParser::parseComposition(const std::filesystem::path& path) {
    const std::filesystem::path base = path.parent_path();
    const YAML::Node comp = YAML::LoadFile(path.string())["simulation_compositions"];
    types::SimulationCompositionData data;
    data.composition_file = path;

    for (const auto& sim_entry : comp["simulations"]) {
        const std::filesystem::path sim_path = base / sim_entry["simulation_config"].as<std::string>();
        types::SimulationConfigData sim_cfg = parseSimulationConfig(sim_path);
        if (sim_cfg.map_filename.is_relative()) {
            // Staff YAMLs use either "map/foo.npy" (next to the sim yaml) or
            // "../map/foo.npy" (inputs/map). Pick the first path that exists.
            const std::filesystem::path relative = sim_cfg.map_filename;
            const std::filesystem::path candidates[] = {
                sim_path.parent_path() / relative,
                sim_path.parent_path().parent_path() / relative,
                base / relative,
            };
            bool found = false;
            for (const auto& candidate : candidates) {
                std::error_code ec;
                if (std::filesystem::is_regular_file(candidate, ec) && !ec) {
                    sim_cfg.map_filename = candidate;
                    found = true;
                    break;
                }
            }
            if (!found) { sim_cfg.map_filename = sim_path.parent_path() / relative; }
        }
        std::vector<common::types::MissionConfigData> missions;
        for (const auto& m_node : sim_entry["mission_configs"]) {
            missions.push_back(parseMissionConfig(base / m_node.as<std::string>()));
        }
        data.simulation_mission_groups.emplace_back(std::move(sim_cfg), std::move(missions));
    }
    for (const auto& d_node : comp["drone_configs"]) {
        data.drone_configs.push_back(parseDroneConfig(base / d_node.as<std::string>()));
    }
    for (const auto& l_node : comp["lidar_configs"]) {
        data.lidar_configs.push_back(parseLidarConfig(base / l_node.as<std::string>()));
    }
    return data;
}

} // namespace simulator
