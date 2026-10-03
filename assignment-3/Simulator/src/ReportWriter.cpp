// ReportWriter.cpp - YAML reports for assignment 2 per-plugin files and assignment 3 summaries.

#include <Simulator/ReportWriter.h>
#include <UserCommon/TimeUtils.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <tuple>

namespace simulator {

namespace ucm = user_common_213309941_213727837;

namespace {

void dumpToFile(const YAML::Emitter& out, const std::filesystem::path& file) {
    if (file.has_parent_path()) { std::filesystem::create_directories(file.parent_path()); }
    std::ofstream stream(file);
    stream << out.c_str();
}

struct Totals {
    double score = 0.0;
    std::size_t steps = 0;
    bool ok = true;
};

Totals summarise(const types::SimulationManagerReport& report) {
    Totals t;
    for (const auto& run : report.runs) {
        if (run.mission_score < 0.0) { t.ok = false; }
        else { t.score += run.mission_score; }
        if (!run.mission_results.empty()) { t.steps += run.mission_results[0].steps; }
    }
    return t;
}

} // namespace

void writeAssignment2Report(const types::SimulationManagerReport& report,
                            const std::filesystem::path& file,
                            const std::string& plugin_name) {
    // Per-plugin YAML named with the plugin so comparative/competitive files do not collide.
    YAML::Emitter out;
    out << YAML::BeginMap << YAML::Key << "score_report" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "plugin" << YAML::Value << plugin_name;
    out << YAML::Key << "generated_at_utc" << YAML::Value << report.generated_at_utc;
    out << YAML::Key << "metric" << YAML::Value << report.metric;
    out << YAML::Key << "score_range" << YAML::BeginMap;
    out << YAML::Key << "min" << YAML::Value << std::get<0>(report.score_range);
    out << YAML::Key << "max" << YAML::Value << std::get<1>(report.score_range);
    out << YAML::EndMap;
    out << YAML::Key << "error_score" << YAML::Value << report.error_score;
    out << YAML::Key << "runs" << YAML::BeginSeq;
    for (const auto& r : report.runs) {
        out << YAML::BeginMap;
        out << YAML::Key << "simulation_map" << YAML::Value << r.simulation_config.map_filename.string();
        out << YAML::Key << "output_map_file" << YAML::Value << r.output_map_file.string();
        out << YAML::Key << "mission_score" << YAML::Value << r.mission_score;
        out << YAML::Key << "steps" << YAML::Value
            << static_cast<int>(r.mission_results.empty() ? 0 : r.mission_results[0].steps);
        out << YAML::EndMap;
    }
    out << YAML::EndSeq << YAML::EndMap << YAML::EndMap;
    dumpToFile(out, file);
}

void writeComparativeReport(const std::filesystem::path& composition_file,
                            const std::filesystem::path& mission_control_folder,
                            const std::vector<PluginBatchResult>& batches,
                            const std::filesystem::path& file) {
    // Groups agreeing managers by (rounded total_score, total_steps), largest group first.
    struct Group { std::vector<std::string> names; double score = 0; std::size_t steps = 0; };
    std::map<std::pair<long long, std::size_t>, Group> groups;
    std::vector<std::string> errors;
    for (const auto& batch : batches) {
        if (!batch.loaded) { errors.push_back(batch.plugin_filename); continue; }
        const Totals t = summarise(batch.report);
        if (!t.ok) { errors.push_back(batch.plugin_filename); continue; }
        const auto key = std::make_pair(std::llround(t.score * 100.0), t.steps);
        groups[key].names.push_back(batch.plugin_filename);
        groups[key].score = t.score;
        groups[key].steps = t.steps;
    }
    std::vector<Group> ordered;
    for (auto& [_, g] : groups) { ordered.push_back(std::move(g)); }
    std::sort(ordered.begin(), ordered.end(), [](const Group& a, const Group& b) {
        if (a.names.size() != b.names.size()) { return a.names.size() > b.names.size(); }
        return a.score > b.score;
    });

    YAML::Emitter out;
    out << YAML::BeginMap << YAML::Key << "comparative_report" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "composition_file" << YAML::Value << composition_file.string();
    out << YAML::Key << "mission_control_folder" << YAML::Value << mission_control_folder.string();
    out << YAML::Key << "generated_at_utc" << YAML::Value << ucm::utcTimestamp();
    out << YAML::Key << "results_summary" << YAML::BeginSeq;
    for (const auto& g : ordered) {
        out << YAML::BeginMap;
        out << YAML::Key << "same_results" << YAML::Value << g.names;
        out << YAML::Key << "total_score" << YAML::Value << g.score;
        out << YAML::Key << "total_steps" << YAML::Value << static_cast<int>(g.steps);
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    out << YAML::Key << "errors" << YAML::Value << errors;
    out << YAML::EndMap << YAML::EndMap;
    dumpToFile(out, file);
}

void writeCompetitiveReport(const std::filesystem::path& composition_file,
                            const std::filesystem::path& mission_control_file,
                            const std::vector<PluginBatchResult>& batches,
                            const std::filesystem::path& file) {
    // Rank algorithms by score descending, then steps ascending (assignment ranking).
    struct Row { std::string name; double score = 0; std::size_t steps = 0; };
    std::vector<Row> rows;
    std::vector<std::string> errors;
    for (const auto& batch : batches) {
        if (!batch.loaded) { errors.push_back(batch.plugin_filename); continue; }
        const Totals t = summarise(batch.report);
        if (!t.ok) { errors.push_back(batch.plugin_filename); continue; }
        rows.push_back({batch.plugin_filename, t.score, t.steps});
    }
    std::sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) {
        if (a.score != b.score) { return a.score > b.score; }
        return a.steps < b.steps;
    });

    YAML::Emitter out;
    out << YAML::BeginMap << YAML::Key << "competitive_report" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "composition_file" << YAML::Value << composition_file.string();
    out << YAML::Key << "mission_control" << YAML::Value << mission_control_file.string();
    out << YAML::Key << "generated_at_utc" << YAML::Value << ucm::utcTimestamp();
    out << YAML::Key << "results_summary" << YAML::BeginSeq;
    for (const auto& r : rows) {
        out << YAML::BeginMap;
        out << YAML::Key << "algorithm" << YAML::Value << r.name;
        out << YAML::Key << "total_score" << YAML::Value << r.score;
        out << YAML::Key << "total_steps" << YAML::Value << static_cast<int>(r.steps);
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    out << YAML::Key << "errors" << YAML::Value << errors;
    out << YAML::EndMap << YAML::EndMap;
    dumpToFile(out, file);
}

} // namespace simulator
