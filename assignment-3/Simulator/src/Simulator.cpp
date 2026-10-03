// Simulator.cpp - comparative / competitive orchestration with lazy .so loading
// and a worker pool. Bonus: each plugin is dlopen'ed once, used for all of its
// jobs, then released (never re-loaded).

#include <Simulator/Simulator.h>
#include <Simulator/Registrar.h>
#include <Simulator/ReportWriter.h>
#include <Simulator/SimulationManager.h>
#include <Simulator/SimulationRunFactoryImpl.h>
#include <Simulator/YamlParser.h>
#include <UserCommon/Logger.h>
#include <UserCommon/TimeUtils.h>

#include <atomic>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace simulator {

namespace ucm = user_common_213309941_213727837; // namespace for user common library

namespace {

// Drops factory std::functions (they close over plugin code) then dlclose.
// Recitation 10: never dlclose while plugin callables are still alive.
void unloadPlugin(LoadedPlugin& plugin) {
    const auto path = plugin.path;
    plugin.algorithms.clear();
    plugin.mission_controls.clear();
    if (!path.empty()) { Registrar::instance().release(path); }
}

void runWorkers(std::size_t requested, std::size_t job_count, const std::function<void(std::size_t)>& job) {
    // requested==1 or missing → main thread only. requested>=2 → that many extra workers
    // (in addition to main, which joins). Never spawn more workers than jobs.
    if (job_count == 0) { return; }
    std::size_t workers = 1;
    if (requested >= 2) { workers = std::min(requested, job_count); }
    if (workers <= 1) {
        for (std::size_t i = 0; i < job_count; ++i) { job(i); }
        return;
    }
    std::atomic<std::size_t> next{0};
    std::vector<std::thread> pool;
    pool.reserve(workers);
    for (std::size_t w = 0; w < workers; ++w) {
        pool.emplace_back([&] {
            while (true) {
                const std::size_t i = next.fetch_add(1);
                if (i >= job_count) { return; }
                job(i);
            }
        });
    }
    for (auto& t : pool) { t.join(); }
}

PluginBatchResult runPluginBatch(const LoadedPlugin& plugin, PluginKind kind,
                                 const LoadedPlugin& other,
                                 const types::SimulationCompositionData& composition,
                                 const std::filesystem::path& output_dir, bool verbose) {
    // One .so × the full composition. `plugin` is the varying side; `other` is the fixed side.
    PluginBatchResult batch;
    batch.plugin_filename = plugin.filename;
    try {
        const auto& algo_factory =
            (kind == PluginKind::Algorithm) ? plugin.algorithms.front() : other.algorithms.front();
        const auto& mc_factory = (kind == PluginKind::MissionControl)
                                     ? plugin.mission_controls.front()
                                     : other.mission_controls.front();
        const std::string algo_label =
            (kind == PluginKind::Algorithm) ? plugin.filename : other.filename;
        const std::string mc_label =
            (kind == PluginKind::MissionControl) ? plugin.filename : other.filename;
        auto factory = std::make_unique<SimulationRunFactoryImpl>(
            algo_factory, mc_factory, algo_label, mc_label, verbose);
        SimulationManager manager{std::move(factory)};
        batch.report = manager.run(composition, output_dir);
        writeAssignment2Report(batch.report,
                               output_dir / ("simulation_output_" +
                                             ucm::sanitizeForFilename(batch.plugin_filename) + ".yaml"),
                               batch.plugin_filename);
    } catch (const std::exception& e) {
        batch.loaded = false;
        batch.error = e.what();
    }
    return batch;
}

} // namespace

int Simulator::run(const CommandLine& cli) const { // comparative or competitive orchestration
    types::SimulationCompositionData composition;
    try {
        composition = YamlParser::parseComposition(cli.simulation);
    } catch (const std::exception& e) {
        std::cerr << "Failed to parse composition: " << e.what() << "\n";
        return 1;
    }

    const std::string stamp = ucm::uniqueStamp();
    std::filesystem::path output_dir;
    if (cli.mode == RunMode::Comparative) {
        output_dir = cli.mission_control_folder / ("comparative_results_" + stamp);
    } else {
        output_dir = cli.algorithms_folder / ("competition_" + stamp);
    }
    std::error_code ec;
    std::filesystem::create_directories(output_dir, ec);
    if (ec) {
        std::cerr << "Cannot create results folder " << output_dir << ": " << ec.message() << "\n";
        return 1;
    }

    user_common_213309941_213727837::Logger sim_log(output_dir / "error_log.txt");
    auto& registrar = Registrar::instance();

    std::vector<PluginBatchResult> batches;

    if (cli.mode == RunMode::Comparative) {
        LoadedPlugin algorithm;
        try {
            algorithm = registrar.acquire(cli.algorithm, PluginKind::Algorithm);
        } catch (const std::exception& e) {
            std::cerr << "Cannot load algorithm: " << e.what() << "\n";
            return 1;
        }
        const auto mc_files = listSharedObjects(cli.mission_control_folder);
        batches.resize(mc_files.size());
        runWorkers(static_cast<std::size_t>(cli.num_threads), mc_files.size(),
                   [&](std::size_t i) {
                       try {
                           auto mc = registrar.acquire(mc_files[i], PluginKind::MissionControl);
                           batches[i] = runPluginBatch(mc, PluginKind::MissionControl, algorithm,
                                                       composition, output_dir, cli.verbose);
                           unloadPlugin(mc);
                       } catch (const std::exception& e) {
                           batches[i].loaded = false;
                           batches[i].plugin_filename = mc_files[i].filename().string();
                           batches[i].error = e.what();
                           sim_log.error("PLUGIN_LOAD", e.what());
                       }
                   });
        unloadPlugin(algorithm); 
        writeComparativeReport(cli.simulation, cli.mission_control_folder, batches,
                               output_dir / "comparative_report.yaml");
    } else {
        LoadedPlugin mission_control;
        try {
            mission_control = registrar.acquire(cli.mission_control, PluginKind::MissionControl);
        } catch (const std::exception& e) {
            std::cerr << "Cannot load mission control: " << e.what() << "\n";
            return 1;
        }
        const auto algo_files = listSharedObjects(cli.algorithms_folder);
        batches.resize(algo_files.size());
        runWorkers(static_cast<std::size_t>(cli.num_threads), algo_files.size(),
                   [&](std::size_t i) {
                       try {
                           auto algo = registrar.acquire(algo_files[i], PluginKind::Algorithm);
                           batches[i] = runPluginBatch(algo, PluginKind::Algorithm, mission_control,
                                                       composition, output_dir, cli.verbose);
                           unloadPlugin(algo);
                       } catch (const std::exception& e) {
                           batches[i].loaded = false;
                           batches[i].plugin_filename = algo_files[i].filename().string();
                           batches[i].error = e.what();
                           sim_log.error("PLUGIN_LOAD", e.what());
                       }
                   });
        unloadPlugin(mission_control);
        writeCompetitiveReport(cli.simulation, cli.mission_control, batches,
                               output_dir / "competitive_report.yaml");
    }

    std::cout << "Simulation finished. Results in " << output_dir << "\n";
    return 0;
}

} // namespace simulator
