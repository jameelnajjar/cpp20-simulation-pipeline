#include "YamlParser.h"

#include "ErrorHandler.h"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace drone_mapper {

namespace {

PhysicalLength parseCm(const YAML::Node& node, const std::string& key) {
    return node[key].as<double>() * cm;
}


types::MappingBounds parseBoundaries(const YAML::Node& node) {
    types::MappingBounds b;
    const auto& x_bnd = node["x_boundary"];
    const auto& y_bnd = node["y_boundary"];
    const auto& h_bnd = node["height_boundary"];

    b.min_x = x_bnd["min_cm"].as<double>() * x_extent[cm];
    b.max_x = x_bnd["max_cm"].as<double>() * x_extent[cm];
    b.min_y = y_bnd["min_cm"].as<double>() * y_extent[cm];
    b.max_y = y_bnd["max_cm"].as<double>() * y_extent[cm];
    b.min_height = h_bnd["min_cm"].as<double>() * z_extent[cm];
    b.max_height = h_bnd["max_cm"].as<double>() * z_extent[cm];
    return b;
}

} // namespace

types::DroneConfigData YamlParser::parseDroneConfig(const std::filesystem::path& path) {
    YAML::Node root = YAML::LoadFile(path.string());
    const YAML::Node& n = root["drone_config"];
    types::DroneConfigData cfg;
    // Note: YAML uses "dimensions_cm" for diameter; convert to radius
    if (n["dimensions_cm"]) {
        cfg.radius = (n["dimensions_cm"].as<double>() / 2.0) * cm;
    } else if (n["radius_cm"]) {
        cfg.radius = parseCm(n, "radius_cm");
    }
    cfg.max_rotate = n["max_rotate_deg"].as<double>() * horizontal_angle[deg];
    cfg.max_advance = parseCm(n, "max_advance_cm");
    cfg.max_elevate = parseCm(n, "max_elevate_cm");
    return cfg;
}

types::LidarConfigData YamlParser::parseLidarConfig(const std::filesystem::path& path) {
    YAML::Node root = YAML::LoadFile(path.string());
    const YAML::Node& n = root["lidar_config"];
    types::LidarConfigData cfg;
    cfg.z_min = parseCm(n, "z_min_cm");
    cfg.z_max = parseCm(n, "z_max_cm");
    cfg.d = parseCm(n, "d_cm");
    cfg.fov_circles = n["fov_circles"].as<std::size_t>();
    return cfg;
}

types::MissionConfigData YamlParser::parseMissionConfig(const std::filesystem::path& path) {
    YAML::Node root = YAML::LoadFile(path.string());
    const YAML::Node& n = root["mission_config"];
    types::MissionConfigData cfg;
    cfg.max_steps = n["max_steps"].as<std::size_t>();
    cfg.gps_resolution = parseCm(n, "gps_resolution_cm");
    if (n["output_mapping_resolution_factor"]) {
        cfg.output_mapping_resolution_factor = n["output_mapping_resolution_factor"].as<double>();
    } else {
        cfg.output_mapping_resolution_factor = 1.0;
    }
    if (n["boundaries"]) {
        cfg.mission_bounds = parseBoundaries(n["boundaries"]);
    }
    return cfg;
}

types::SimulationConfigData YamlParser::parseSimulationConfig(const std::filesystem::path& path) {
    YAML::Node root = YAML::LoadFile(path.string());
    const YAML::Node& n = root["simulation_config"];
    types::SimulationConfigData cfg;
    cfg.map_filename = n["map_filename"].as<std::string>();
    cfg.map_resolution = parseCm(n, "map_resolution_cm");

    const auto& dpos = n["initial_drone_position"];
    cfg.initial_drone_position = Position3D{
        dpos["x_cm"].as<double>() * x_extent[cm],
        dpos["y_cm"].as<double>() * y_extent[cm],
        dpos["height_cm"].as<double>() * z_extent[cm],
    };
    cfg.initial_angle = n["initial_angle_deg"].as<double>() * horizontal_angle[deg];

    if (n["map_axes_offset"]) {
        const auto& off = n["map_axes_offset"];
        cfg.map_offset = Position3D{
            off["x_offset"].as<double>() * x_extent[cm],
            off["y_offset"].as<double>() * y_extent[cm],
            off["height_offset"].as<double>() * z_extent[cm],
        };
    }
    return cfg;
}

types::SimulationCompositionData YamlParser::parseComposition(const std::filesystem::path& path) {
    const std::filesystem::path base_dir = path.parent_path();
    YAML::Node root = YAML::LoadFile(path.string());
    const YAML::Node& comp = root["simulation_compositions"];

    types::SimulationCompositionData data;
    data.composition_file = path;

    // Parse simulations with their mission groups
    for (const auto& sim_entry : comp["simulations"]) {
        const std::string sim_path_str = sim_entry["simulation_config"].as<std::string>();
        const std::filesystem::path sim_path = base_dir / sim_path_str;

        types::SimulationConfigData sim_cfg;
        try {
            sim_cfg = parseSimulationConfig(sim_path);
            // Make map_filename relative to simulation config file's directory
            if (sim_cfg.map_filename.is_relative()) {
                sim_cfg.map_filename = sim_path.parent_path() / sim_cfg.map_filename;
            }
        } catch (const std::exception& e) {
            ErrorHandler::instance().logError("SIM_CONFIG_PARSE_ERROR",
                "Failed to parse " + sim_path.string() + ": " + e.what());
            continue;
        }

        std::vector<types::MissionConfigData> missions;
        for (const auto& m_node : sim_entry["mission_configs"]) {
            const std::string m_path_str = m_node.as<std::string>();
            const std::filesystem::path m_path = base_dir / m_path_str;
            try {
                missions.push_back(parseMissionConfig(m_path));
            } catch (const std::exception& e) {
                ErrorHandler::instance().logError("MISSION_CONFIG_PARSE_ERROR",
                    "Failed to parse " + m_path.string() + ": " + e.what());
            }
        }
        data.simulation_mission_groups.emplace_back(sim_cfg, std::move(missions));
    }

    // Parse drone configs
    for (const auto& d_node : comp["drone_configs"]) {
        const std::filesystem::path d_path = base_dir / d_node.as<std::string>();
        try {
            data.drones.push_back(parseDroneConfig(d_path));
        } catch (const std::exception& e) {
            ErrorHandler::instance().logError("DRONE_CONFIG_PARSE_ERROR",
                "Failed to parse " + d_path.string() + ": " + e.what());
        }
    }

    // Parse lidar configs
    for (const auto& l_node : comp["lidar_configs"]) {
        const std::filesystem::path l_path = base_dir / l_node.as<std::string>();
        try {
            data.lidars.push_back(parseLidarConfig(l_path));
        } catch (const std::exception& e) {
            ErrorHandler::instance().logError("LIDAR_CONFIG_PARSE_ERROR",
                "Failed to parse " + l_path.string() + ": " + e.what());
        }
    }

    return data;
}

} // namespace drone_mapper
