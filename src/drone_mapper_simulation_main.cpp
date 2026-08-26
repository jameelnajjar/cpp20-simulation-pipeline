#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>

#include "ErrorHandler.h"
#include "YamlParser.h"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace fs = std::filesystem;
using namespace drone_mapper;

namespace {

// Resolve composition file path following assignment spec.
fs::path resolveCompositionPath(const std::string& arg) {
    const fs::path p{arg};
    if (p.is_absolute()) return p;
    return fs::current_path() / p;
}

// Write simulation_output.yaml
void writeSimulationOutput(const types::SimulationManagerReport& report,
                            const fs::path& output_path) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "score_report" << YAML::Value;
    out << YAML::BeginMap;
    out << YAML::Key << "generated_at_utc" << YAML::Value << report.generated_at_utc;
    out << YAML::Key << "metric" << YAML::Value << report.metric;
    out << YAML::Key << "score_range" << YAML::Value;
    out << YAML::BeginMap;
    out << YAML::Key << "min" << YAML::Value << std::get<0>(report.score_range);
    out << YAML::Key << "max" << YAML::Value << std::get<1>(report.score_range);
    out << YAML::EndMap;
    out << YAML::Key << "error_score" << YAML::Value << report.error_score;

    // Summary
    int scored = 0, errors = 0;
    double total = 0.0, min_s = 100.0, max_s = 0.0;
    for (const auto& r : report.runs) {
        if (r.mission_score < 0) { ++errors; }
        else { ++scored; total += r.mission_score; min_s = std::min(min_s, r.mission_score); max_s = std::max(max_s, r.mission_score); }
    }
    out << YAML::Key << "summary" << YAML::Value;
    out << YAML::BeginMap;
    out << YAML::Key << "total_runs" << YAML::Value << static_cast<int>(report.runs.size());
    out << YAML::Key << "scored_runs" << YAML::Value << scored;
    out << YAML::Key << "error_runs" << YAML::Value << errors;
    out << YAML::Key << "average_score" << YAML::Value << (scored > 0 ? total / scored : 0.0);
    out << YAML::Key << "min_score" << YAML::Value << (scored > 0 ? min_s : 0.0);
    out << YAML::Key << "max_score" << YAML::Value << (scored > 0 ? max_s : 0.0);
    out << YAML::EndMap;

    // Runs
    out << YAML::Key << "runs" << YAML::Value << YAML::BeginSeq;
    for (const auto& r : report.runs) {
        out << YAML::BeginMap;
        out << YAML::Key << "simulation_map" << YAML::Value << r.simulation_config.map_filename.string();
        out << YAML::Key << "output_map_file" << YAML::Value << r.output_map_file.string();
        out << YAML::Key << "mission_score" << YAML::Value << r.mission_score;
        out << YAML::Key << "steps" << YAML::Value
            << static_cast<int>(r.mission_results.empty() ? 0 : r.mission_results[0].steps);
        if (!r.mission_results.empty()) {
            const auto& mr = r.mission_results[0];
            std::string status_str;
            switch (mr.status) {
                case types::MissionRunStatus::Completed: status_str = "completed"; break;
                case types::MissionRunStatus::MaxSteps:  status_str = "max_steps"; break;
                case types::MissionRunStatus::Error:     status_str = "error";     break;
            }
            out << YAML::Key << "status" << YAML::Value << status_str;
        }
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    out << YAML::EndMap << YAML::EndMap;

    const fs::path out_file = output_path / "simulation_output.yaml";
    fs::create_directories(output_path);
    std::ofstream f(out_file);
    f << out.c_str();
}

} // namespace

int main(int argc, char** argv) {
    const fs::path composition_file =
        (argc >= 2) ? resolveCompositionPath(argv[1]) : fs::current_path() / "simulation.yaml";
    const fs::path output_path =
        (argc >= 3) ? fs::path{argv[2]} : fs::current_path();

    // Set up error log
    const fs::path error_log = output_path / "output_results" / "error_log.txt";
    fs::create_directories(error_log.parent_path());
    ErrorHandler::instance().setLogFile(error_log);

    // Parse composition
    types::SimulationCompositionData composition;
    try {
        composition = YamlParser::parseComposition(composition_file);
    } catch (const std::exception& e) {
        std::cerr << "Fatal: could not parse composition file: " << e.what() << "\n";
        return 1;
    }

    // Run simulation
    auto run_factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager{std::move(run_factory)};
    const types::SimulationManagerReport report = manager.run(composition, output_path);

    // Write output YAML
    try {
        writeSimulationOutput(report, output_path);
    } catch (const std::exception& e) {
        std::cerr << "Warning: could not write simulation_output.yaml: " << e.what() << "\n";
    }

    std::cout << "Simulation complete. " << report.runs.size() << " run(s).\n";
    return 0;
}
