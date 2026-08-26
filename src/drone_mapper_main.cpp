// drone_mapper — assignment submission entry point.
// Usage: drone_mapper [<input_output_files_path>]
//   <input_output_files_path>  directory containing sim_compose.yaml and all
//                               referenced config files (default: current dir).
// Output written to <input_output_files_path>/map_output.txt (human-readable
// summary) and <input_output_files_path>/output_results/ (NPY maps + logs).
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>

#include "ErrorHandler.h"
#include "YamlParser.h"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

namespace fs = std::filesystem;
using namespace drone_mapper;

namespace {

void writeMapOutputTxt(const types::SimulationManagerReport& report,
                       const fs::path& output_path) {
    const fs::path out_file = output_path / "map_output.txt";
    std::ofstream f(out_file);
    if (!f) {
        std::cerr << "Warning: could not write " << out_file << "\n";
        return;
    }

    f << "Drone Mapper Simulation Report\n";
    f << "==============================\n";
    f << "Generated: " << report.generated_at_utc << "\n";
    f << "Metric:    " << report.metric << "\n";
    f << "\n";

    int run_idx = 0;
    for (const auto& r : report.runs) {
        ++run_idx;
        f << "Run " << run_idx << ":\n";
        f << "  map:          " << r.simulation_config.map_filename.string() << "\n";
        f << "  output_map:   " << r.output_map_file.string() << "\n";
        f << "  score:        " << r.mission_score << "\n";

        if (!r.mission_results.empty()) {
            const auto& mr = r.mission_results[0];
            f << "  steps:        " << static_cast<int>(mr.steps) << "\n";
            std::string status_str;
            switch (mr.status) {
                case types::MissionRunStatus::Completed: status_str = "completed"; break;
                case types::MissionRunStatus::MaxSteps:  status_str = "max_steps"; break;
                case types::MissionRunStatus::Error:     status_str = "error";     break;
            }
            f << "  status:       " << status_str << "\n";
        }
        f << "\n";
    }

    int scored = 0;
    double total = 0.0;
    for (const auto& r : report.runs) {
        if (r.mission_score >= 0) {
            ++scored;
            total += r.mission_score;
        }
    }
    if (scored > 0) {
        f << "Average score: " << (total / scored) << " / 100\n";
    }
}

} // anonymous namespace

int main(int argc, char** argv) {
    const fs::path io_path =
        (argc >= 2) ? fs::path{argv[1]} : fs::current_path();

    const fs::path composition_file = io_path / "sim_compose.yaml";

    if (!fs::exists(composition_file)) {
        std::cerr << "Error: sim_compose.yaml not found in " << io_path << "\n";
        return 1;
    }

    const fs::path error_log = io_path / "output_results" / "error_log.txt";
    fs::create_directories(error_log.parent_path());
    ErrorHandler::instance().setLogFile(error_log);

    types::SimulationCompositionData composition;
    try {
        composition = YamlParser::parseComposition(composition_file);
    } catch (const std::exception& e) {
        std::cerr << "Fatal: could not parse composition file: " << e.what() << "\n";
        return 1;
    }

    auto run_factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager{std::move(run_factory)};
    const types::SimulationManagerReport report = manager.run(composition, io_path);

    try {
        writeMapOutputTxt(report, io_path);
    } catch (const std::exception& e) {
        std::cerr << "Warning: could not write map_output.txt: " << e.what() << "\n";
    }

    for (const auto& r : report.runs) {
        std::cout << "score=" << r.mission_score;
        if (!r.mission_results.empty()) {
            std::cout << " steps=" << r.mission_results[0].steps;
        }
        std::cout << " output=" << r.output_map_file << "\n";
    }

    return 0;
}
