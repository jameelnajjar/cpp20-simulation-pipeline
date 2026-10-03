#pragma once

// Registrar.h - Simulator-owned singleton that receives auto-registration from
// plugins (via MappingAlgorithmRegistration / MissionControlRegistration) and
// owns every dlopen handle until it is safe to dlclose.
// Pattern inherited from recitation 10 (dynamic loading) and the assignment's
// "Automatic Registration" section.

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

namespace simulator {

enum class PluginKind { Algorithm, MissionControl };

struct LoadedPlugin {
    std::filesystem::path path{};
    std::string filename{};
    // Factory copies close over plugin code; clear them before Registrar::release/dlclose.
    std::vector<common::MappingAlgorithmFactory> algorithms{};
    std::vector<common::MissionControlFactory> mission_controls{};
};

class Registrar {
public:
    [[nodiscard]] static Registrar& instance(); // Meyer's singleton; process-wide

    Registrar(const Registrar&) = delete;
    Registrar& operator=(const Registrar&) = delete;

    // Called from registration-object constructors while dlopen() is in progress.
    void addAlgorithm(common::MappingAlgorithmFactory factory);
    void addMissionControl(common::MissionControlFactory factory);

    // Bonus lazy-load: open a .so the first time it is needed, never dlopen the
    // same path twice, and keep it mapped until release() drops the last user.
    [[nodiscard]] LoadedPlugin acquire(const std::filesystem::path& path, PluginKind kind);

    // Drops one user of `path`. When the last user leaves, factories are destroyed
    // first (they may close over plugin code) and then dlclose runs.
    void release(const std::filesystem::path& path);

    ~Registrar();

private:
    class LibraryHandle {
    public:
        explicit LibraryHandle(std::filesystem::path path);
        ~LibraryHandle();
        LibraryHandle(const LibraryHandle&) = delete;
        LibraryHandle& operator=(const LibraryHandle&) = delete;
        LibraryHandle(LibraryHandle&& other) noexcept;
        LibraryHandle& operator=(LibraryHandle&& other) noexcept;
        [[nodiscard]] const std::filesystem::path& path() const { return path_; }

    private:
        void close() noexcept;
        std::filesystem::path path_;
        void* handle_ = nullptr; // void* from dlopen; owned uniquely
    };

    struct Entry {
        LibraryHandle library;
        LoadedPlugin plugin;
        int users = 0; // how many in-flight jobs currently need this .so
    };

    Registrar() = default; // 

    std::mutex mutex_;               // serialises dlopen + registration capture
    std::vector<Entry> entries_;     // loaded plugins, one per unique path
    std::vector<common::MappingAlgorithmFactory> pending_algorithms_; // algorithms to be registered
    std::vector<common::MissionControlFactory> pending_mission_controls_; // mission controls to be registered
 };

} // namespace simulator
