#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/NpyArray.h>

#include "YamlParser.h"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;
using namespace drone_mapper;

namespace {

// Parse optional comparison_config yaml
struct ComparisonConfig {
    types::MapConfig origin_cfg;
    types::MapConfig target_cfg;
};

types::MapConfig defaultMapConfig(const NpyArray& arr, double res_cm = 10.0) {
    types::MapConfig cfg;
    cfg.resolution = res_cm * cm;
    cfg.offset = Position3D{};
    cfg.boundaries = types::MappingBounds{
        0.0 * x_extent[cm],
        static_cast<double>(arr.nx()) * res_cm * x_extent[cm],
        0.0 * y_extent[cm],
        static_cast<double>(arr.ny()) * res_cm * y_extent[cm],
        0.0 * z_extent[cm],
        static_cast<double>(arr.nz()) * res_cm * z_extent[cm],
    };
    return cfg;
}

types::MapConfig parseMapCfgFromNode(const YAML::Node& n, const NpyArray& arr) {
    types::MapConfig cfg;
    cfg.resolution = n["map_res_cm"].as<double>() * cm;
    if (n["map_offset"]) {
        cfg.offset = Position3D{
            n["map_offset"]["x_offset"].as<double>() * x_extent[cm],
            n["map_offset"]["y_offset"].as<double>() * y_extent[cm],
            n["map_offset"]["height_offset"].as<double>() * z_extent[cm],
        };
    }
    if (n["map_boundaries"]) {
        const auto& b = n["map_boundaries"];
        cfg.boundaries = types::MappingBounds{
            b["x_boundary"]["min_cm"].as<double>() * x_extent[cm],
            b["x_boundary"]["max_cm"].as<double>() * x_extent[cm],
            b["y_boundary"]["min_cm"].as<double>() * y_extent[cm],
            b["y_boundary"]["max_cm"].as<double>() * y_extent[cm],
            b["height_boundary"]["min_cm"].as<double>() * z_extent[cm],
            b["height_boundary"]["max_cm"].as<double>() * z_extent[cm],
        };
    } else {
        cfg.boundaries = defaultMapConfig(arr).boundaries;
    }
    return cfg;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0]
                  << " <origin_map.npy> <target_map.npy> [comparison_config=<config.yaml>]\n";
        std::cout << -1 << "\n";
        return 1;
    }

    const fs::path origin_path{argv[1]};
    const fs::path target_path{argv[2]};
    fs::path config_path{};

    for (int i = 3; i < argc; ++i) {
        const std::string arg{argv[i]};
        const std::string prefix = "comparison_config=";
        if (arg.substr(0, prefix.size()) == prefix) {
            config_path = arg.substr(prefix.size());
        }
    }

    try {
        const NpyArray origin_arr = NpyArray::load(origin_path);
        const NpyArray target_arr = NpyArray::load(target_path);

        types::MapConfig origin_cfg = defaultMapConfig(origin_arr);
        types::MapConfig target_cfg = defaultMapConfig(target_arr);

        if (!config_path.empty()) {
            YAML::Node root = YAML::LoadFile(config_path.string());
            const auto& comp = root["comparison_config"];
            if (comp["original"]) origin_cfg = parseMapCfgFromNode(comp["original"], origin_arr);
            if (comp["target"])   target_cfg = parseMapCfgFromNode(comp["target"],   target_arr);
        }

        auto origin_npy = std::make_shared<NpyArray>(origin_arr);
        auto target_npy = std::make_shared<NpyArray>(target_arr);

        Map3DImpl origin_map(origin_npy, origin_cfg);
        Map3DImpl target_map(target_npy, target_cfg);

        std::vector<IMap3D*> targets = {&target_map};
        const std::vector<double> scores = MapsComparison::compare(origin_map, targets);

        std::cout << scores[0] << "\n";
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        std::cout << -1 << "\n";
        return 1;
    }
}
