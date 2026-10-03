// Registrar.cpp - implements Simulator/Registrar.h. Recitation 10 LibraryHandle pattern:
// dlopen in the constructor, dlclose in the destructor, factories destroyed first.

#include <Simulator/Registrar.h>

#include <dlfcn.h> 

#include <stdexcept>
#include <utility>

namespace simulator {

Registrar& Registrar::instance() {
    static Registrar registrar;
    return registrar;
}

Registrar::~Registrar() {
    // Plugin std::functions may close over .so code; drop them before dlclose.
    for (Entry& entry : entries_) {
        entry.plugin.algorithms.clear();
        entry.plugin.mission_controls.clear();
    }
    entries_.clear();
}

void Registrar::addAlgorithm(common::MappingAlgorithmFactory factory) {
    pending_algorithms_.push_back(std::move(factory)); // move the factory to the pending_algorithms_ vector
}

void Registrar::addMissionControl(common::MissionControlFactory factory) {
    pending_mission_controls_.push_back(std::move(factory));
}

Registrar::LibraryHandle::LibraryHandle(std::filesystem::path path) : path_(std::move(path)) {
    dlerror(); // clear any previous errors
    handle_ = dlopen(path_.c_str(), RTLD_NOW | RTLD_LOCAL); // open the library
    if (handle_ == nullptr) {
        const char* error = dlerror();
        throw std::runtime_error("Cannot load " + path_.string() + ": " +
                                 (error == nullptr ? "unknown dlopen error" : error));
    }
}

Registrar::LibraryHandle::~LibraryHandle() { close(); }

Registrar::LibraryHandle::LibraryHandle(LibraryHandle&& other) noexcept
    : path_(std::move(other.path_)), handle_(std::exchange(other.handle_, nullptr)) {} 

Registrar::LibraryHandle& Registrar::LibraryHandle::operator=(LibraryHandle&& other) noexcept {
    if (this != &other) {
        close();
        path_ = std::move(other.path_);
        handle_ = std::exchange(other.handle_, nullptr);
    }
    return *this;
}

void Registrar::LibraryHandle::close() noexcept {
    if (handle_ == nullptr) { return; }
    dlclose(handle_);
    handle_ = nullptr;
}

LoadedPlugin Registrar::acquire(const std::filesystem::path& path, PluginKind kind) { // The lazy-load bonus is: open a .so once, never dlopen the same library twice, unload when unused. This is achieved by checking if the library is already loaded and if so, returning the existing plugin. If not, it loads the library and returns a new plugin.
    const std::lock_guard lock{mutex_}; // lock the mutex to prevent multiple threads from loading the same library simultaneously
    const auto canonical = std::filesystem::weakly_canonical(path); // get the canonical path of the library
    
    for (Entry& entry : entries_) {
        if (entry.library.path() == canonical || entry.plugin.path == path) { // 
            ++entry.users; 
            return entry.plugin; 
        }
    }

    pending_algorithms_.clear();
    pending_mission_controls_.clear();
    LibraryHandle library{canonical};

    LoadedPlugin plugin;
    plugin.path = canonical;
    plugin.filename = path.filename().string();
    plugin.algorithms = pending_algorithms_;
    plugin.mission_controls = pending_mission_controls_;
    pending_algorithms_.clear();
    pending_mission_controls_.clear();

    if (kind == PluginKind::Algorithm && plugin.algorithms.empty()) {
        throw std::runtime_error(path.string() + " did not register a mapping algorithm");
    }
    if (kind == PluginKind::MissionControl && plugin.mission_controls.empty()) {
        throw std::runtime_error(path.string() + " did not register a mission control");
    }

    entries_.push_back(Entry{std::move(library), plugin, 1}); 
    return plugin;
}

void Registrar::release(const std::filesystem::path& path) {
    const std::lock_guard lock{mutex_};
    const auto canonical = std::filesystem::weakly_canonical(path);
    for (auto it = entries_.begin(); it != entries_.end(); ++it) {
        if (it->library.path() == canonical || it->plugin.path == path) {
            if (--it->users <= 0) {
                it->plugin.algorithms.clear();
                it->plugin.mission_controls.clear();
                entries_.erase(it); // LibraryHandle dtor calls dlclose
            }
            return;
        }
    }
}

} // namespace simulator
