// CommandLine.cpp - argv parser. Arguments may appear in any order.

#include <Simulator/CommandLine.h>

#include <algorithm>
#include <iostream>

namespace simulator {

namespace {

void printUsage(const std::string& message) {
    std::cerr << message << "\n\n"
              << "Usage:\n"
              << "  simulator_<ids> -comparative simulation=<yaml> "
                 "mission_control_folder=<dir> algorithm=<so> [num_threads=N] [-verbose]\n"
              << "  simulator_<ids> -competition simulation=<yaml> "
                 "mission_control=<so> algorithms_folder=<dir> [num_threads=N] [-verbose]\n";
}

[[nodiscard]] bool readableFile(const std::filesystem::path& path) {
    std::error_code ec;
    return std::filesystem::is_regular_file(path, ec) && !ec; // check if the file is readable (exists and is a regular file)
}

[[nodiscard]] bool traversableDir(const std::filesystem::path& path) {
    std::error_code ec;   
    return std::filesystem::is_directory(path, ec) && !ec; // check if the directory is traversable (exists and is a directory)
}

} // namespace

std::vector<std::filesystem::path> listSharedObjects(const std::filesystem::path& folder) {
    std::vector<std::filesystem::path> files;
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(folder, ec)) {
        if (ec) { break; }
        if (!entry.is_regular_file()) { continue; }
        if (entry.path().extension() == ".so") { files.push_back(entry.path()); }
    }
    std::sort(files.begin(), files.end(), [](const std::filesystem::path& a,
                                            const std::filesystem::path& b) {
        return a.filename().string() < b.filename().string();
    });
    return files;
}

std::optional<CommandLine> CommandLine::parse(int argc, char** argv) { // any-order argv; usage on error
    CommandLine cli;
    std::vector<std::string> unsupported;
    bool saw_comparative = false;
    bool saw_competition = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg{argv[i]};
        if (arg == "-comparative") { saw_comparative = true; continue; }
        if (arg == "-competition") { saw_competition = true; continue; }
        if (arg == "-verbose") { cli.verbose = true; continue; }
        const auto eq = arg.find('=');
        if (eq == std::string::npos) { 
            unsupported.push_back(arg);
            continue;
        }
        const std::string key = arg.substr(0, eq);
        const std::string value = arg.substr(eq + 1);
        if (key == "simulation") { cli.simulation = value; }
        else if (key == "mission_control_folder") { cli.mission_control_folder = value; }
        else if (key == "algorithm") { cli.algorithm = value; }
        else if (key == "mission_control") { cli.mission_control = value; }
        else if (key == "algorithms_folder") { cli.algorithms_folder = value; }
        else if (key == "num_threads") {
            try { cli.num_threads = std::stoi(value); }
            catch (...) { unsupported.push_back(arg); }
            if (cli.num_threads < 1) { cli.num_threads = 1; }
        } else {
            unsupported.push_back(arg);
        }
    }

    if (!unsupported.empty()) {
        std::string msg = "Unsupported command line argument(s):";
        for (const auto& u : unsupported) { msg += " " + u; }
        printUsage(msg);
        return std::nullopt;
    }
    if (saw_comparative == saw_competition) {
        printUsage("Exactly one of -comparative / -competition is required.");
        return std::nullopt;
    }
    cli.mode = saw_comparative ? RunMode::Comparative : RunMode::Competitive;

    std::vector<std::string> missing;
    if (cli.simulation.empty()) { missing.emplace_back("simulation"); }
    if (cli.mode == RunMode::Comparative) {
        if (cli.mission_control_folder.empty()) { missing.emplace_back("mission_control_folder"); }
        if (cli.algorithm.empty()) { missing.emplace_back("algorithm"); }
    } else {
        if (cli.mission_control.empty()) { missing.emplace_back("mission_control"); }
        if (cli.algorithms_folder.empty()) { missing.emplace_back("algorithms_folder"); }
    }
    if (!missing.empty()) {
        std::string msg = "Missing command line argument(s):";
        for (const auto& m : missing) { msg += " " + m; }
        printUsage(msg);
        return std::nullopt;
    }

    if (!readableFile(cli.simulation)) {
        printUsage("Cannot open simulation composition file: " + cli.simulation.string());
        return std::nullopt;
    }
    if (cli.mode == RunMode::Comparative) {
        if (!traversableDir(cli.mission_control_folder)) {
            printUsage("Cannot traverse mission_control_folder: " + cli.mission_control_folder.string());
            return std::nullopt;
        }
        if (listSharedObjects(cli.mission_control_folder).empty()) {
            printUsage("mission_control_folder contains no .so files: " +
                       cli.mission_control_folder.string());
            return std::nullopt;
        }
        if (!readableFile(cli.algorithm)) {
            printUsage("Cannot open algorithm .so: " + cli.algorithm.string());
            return std::nullopt;
        }
    } else {
        if (!readableFile(cli.mission_control)) {
            printUsage("Cannot open mission_control .so: " + cli.mission_control.string());
            return std::nullopt;
        }
        if (!traversableDir(cli.algorithms_folder)) {
            printUsage("Cannot traverse algorithms_folder: " + cli.algorithms_folder.string());
            return std::nullopt;
        }
        if (listSharedObjects(cli.algorithms_folder).empty()) {
            printUsage("algorithms_folder contains no .so files: " + cli.algorithms_folder.string());
            return std::nullopt;
        }
    }
    return cli;
}

} // namespace simulator
