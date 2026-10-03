# Assignment 3 — Component map and how it was built

This file is the full walkthrough of what the course asked for and how each piece is implemented. Staff-owned files were **not** edited.

Submitters: Omar Abu Shah (`213309941`), Jameel Najjar (`213727837`).
Student IDs used in artifact and namespace names: `213309941`, `213727837`.

---

## 0. Locked vs student-owned

| Path | Owner | Action |
|---|---|---|
| `common/**` (headers + `common/CMakeLists.txt`) | Course | **Do not change, do not add files** |
| `Simulator/common_simulator/include/Simulator/{ISimulation,ISimulationRun,ISimulationRunFactory,SimulationTypes}.h` | Course | **Do not change** |
| `MissionControl/common_mission_control/include/MissionControl/IDroneControl.h` | Course | **Do not change** |
| `CMakePresets.json`, `vcpkg.json`, `vcpkg-configuration.json` | Course skeleton | Left as given |
| `Algorithm/**`, `MissionControl/{include,src}/**`, `Simulator/{include,src}/**`, `UserCommon/**` | Us | Implement |
| Per-folder `CMakeLists.txt` | Skeleton TODOs | Filled in; root CMake extended only to find yaml-cpp / threads |

Registration **headers** stay in `common/`. Registration **`.cpp`** files live in `Simulator/src/` only, as the assignment requires.

---

## 1. What the assignment asks the program to do

A simulator process loads **MissionControl** and **MappingAlgorithm** implementations as `.so` files and runs many missions:

- **Comparative:** one algorithm `.so` × every MissionControl `.so` in a folder × the full YAML composition.
- **Competitive:** one MissionControl `.so` × every algorithm `.so` in a folder × the full YAML composition.

Threading: `num_threads` missing or 1 → main thread only. `>= 2` → that many **additional** workers; main joins. Never start idle workers.

Output directories: `comparative_results_<unique>` under the MC folder, or `competition_<unique>` under the algorithms folder. Each contains maps, error logs, Assignment-2-style YAML per plugin, and one comparative/competitive summary YAML.

---

## 2. Automatic registration (recitation 10)

Staff macros (unchanged):

```
REGISTER_MAPPING_ALGORITHM(class_name)
REGISTER_MISSION_CONTROL(class_name)
```

Each expands to a global `MappingAlgorithmRegistration` / `MissionControlRegistration` object. Its constructor runs during `dlopen`. We implement those constructors in the **Simulator** so they forward the factory into `simulator::Registrar`.

`Registrar` (from recitation 10 `LibraryHandle`):

- `acquire(path, kind)` — `dlopen` once, capture factories registered during that call, increment a user count.
- `release(path)` — decrement; at zero, destroy factories **then** `dlclose`.
- Callers must also clear any **copied** factories (`LoadedPlugin`) before `release`, because `std::function` closes over plugin code. `Simulator.cpp` does this in `unloadPlugin`.
- Destructor of the singleton does the same for anything still loaded.

This is the **lazy-load bonus**: plugins are not preloaded, are never opened twice, and are unloaded when no remaining job needs them.

The executable is built with `ENABLE_EXPORTS ON` so the plugin can resolve the registration constructor (recitation 10 CMake).

Class names passed to the macros cannot contain `::`, so we alias:

```
using MappingAlgorithmImpl_213309941_213727837 = algorithm_...::MappingAlgorithmImpl;
REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl_213309941_213727837);
```

---

## 3. Folder-by-folder components

### 3.1 `UserCommon/` — `namespace user_common_213309941_213727837`

No makefile (assignment rule). Sources are listed in the Algorithm/MC/Simulator CMake files that need them.

| File | Role |
|---|---|
| `GeometryUtils.h` | Header-only unit conversion, beam direction, bounds tests, `VoxelKey`. Shared so MockLidar and ScanResultToVoxels agree on geometry. |
| `Logger.h/.cpp` | Thread-safe append-only log. **Not** a singleton (parallel missions each own one). |
| `TimeUtils.h/.cpp` | `utcTimestamp`, `uniqueStamp` (time + atomic counter) for collision-free folder names. |

### 3.2 `Algorithm/` — `namespace algorithm_213309941_213727837`

| Class / function | Inherits / composed from | Job |
|---|---|---|
| `MappingAlgorithmImpl` | `common::IMappingAlgorithm` | Frontier BFS over **known Empty** cells with hull clearance. Never commands a wall. Skips scans that cannot reveal Unmapped voxels. |
| `nextStep` | overrides interface | State machine: Scan → Plan → Navigate → Done. |
| `findPathToFrontier` | uses `VoxelKey` from UserCommon | 6-connected BFS. |
| `buildMovement` | uses `drone_config_` from the base class | One legal primitive, clamped to max_rotate/advance/elevate. |

Constructor takes `common::MappingAlgorithmDependencies` **by value**, matching the registration lambda.

### 3.3 `MissionControl/` — `namespace mission_control_213309941_213727837`

| Class | Inherits / composed from | Job |
|---|---|---|
| `MissionControlImpl` | `common::IMissionControl` | Built from `MissionControlDependencies`. Creates `DroneControlImpl`, loops `step()` up to `max_steps`, saves the output map, writes error/verbose logs. |
| `DroneControlImpl` | `mission_control::IDroneControl` (staff) | Asks the algorithm for a command, sanitises it, flies, scans, updates the map. |
| `ScanResultToVoxels` | static utility (HW2) | Paints Empty / Occupied / PotentiallyOccupied with strongest-wins. |

`DroneControlImpl` Common-issues handling:

| Row | Behaviour |
|---|---|
| Illegal out-of-bounds move | Amend; ignore if nothing legal remains |
| Invalid command values | Retry N times, then throw |
| Empty NOOP | Retry N times, then throw |
| Wall collision | Not caught here — `MockMovement` throws (mandatory) |
| Empty LiDAR | Retry N times, then throw |
| Movement `false` | Retry N times, then throw |
| Oversize movement | Split into several legal steps |
| GPS OOB | Ignore GPS unless dead reckoning also OOB, then throw |
| Impossible GPS after move | Re-read N times, then Error |

`N = 3` (`kMaxRetries`).

### 3.4 `Simulator/` — `namespace simulator`

| Class | Inherits / composed from | Job |
|---|---|---|
| `Simulator` | none | CLI modes, output folders, worker pool, plugin batches. |
| `CommandLine` | none | Parses flags/`key=value` in any order; prints usage on bad argv. |
| `Registrar` | singleton | `dlopen` / registration / `dlclose`. |
| `NpyArray` | HW2 grid | Load/save `.npy`. |
| `Map3DImpl` | `common::IMutableMap3D` | Hidden + output maps. |
| `MockGPS` | `common::IGPS` | Pose store; mutated by MockMovement. |
| `MockLidar` | `common::ILidar` | Ring beams against hidden map + GPS pose. |
| `MockMovement` | `common::IDroneMovement` | Updates GPS; **throws on Occupied hull collision**. |
| `MapsComparison` | HW2 scorer | Jaccard-style occupied-voxel score. |
| `YamlParser` | HW2 parser | Composition + config files. |
| `SimulationRunFactoryImpl` | `simulator::ISimulationRunFactory` | Wires one run. Thread-safe besides an atomic filename counter. |
| `SimulationRunImpl` | `simulator::ISimulationRun` | `runMission()` + score. Catches exceptions so the Simulator does not crash. |
| `SimulationManager` | `simulator::ISimulation` | Cartesian product of composition entries. |
| `ReportWriter` | YAML::Emitter | Assignment 2 per-plugin YAML + comparative/competitive summaries. |

Factory wiring (one run):

1. Load hidden `.npy` → `Map3DImpl`.
2. Empty output `NpyArray` sized from mission bounds and resolution factor.
3. `MockGPS` at initial pose.
4. `MockMovement(gps, hidden_map, drone.radius)`.
5. `MockLidar(config, hidden_map, gps)`.
6. `algorithm_factory({mission, lidar, drone, output_map})`.
7. `mission_control_factory({..., algorithm, output_map_file, verbose})`.
8. `SimulationRunImpl` owns all of the unique_ptrs (so they outlive the mission).

---

## 4. Threading model

`Simulator::runWorkers`:

- `requested < 2` or one job → run on the caller (main) thread.
- else `workers = min(requested, job_count)` extra threads; main joins.

Jobs are **one plugin × the full composition** (a `SimulationManager::run`). That keeps a `.so` loaded for the duration of its jobs, then `release` unloads it — the bonus path.

No lock is held while a mission runs. Registrar mutex is only around `dlopen` / factory capture / `dlclose`.

---

## 5. YAML outputs

**Comparative** `comparative_report.yaml`: groups MissionControls with the same `(total_score, total_steps)`, sorted by group size. `errors` lists plugins that failed to load or produced a negative score.

**Competitive** `competitive_report.yaml`: one row per algorithm, score descending then steps ascending.

Per plugin: `simulation_output_<plugin>.yaml` in Assignment 2 shape, filename includes the plugin so files do not collide.

Maps: `output_map_<mc>_<algo>_<mapstem>_<stamp>_<n>.npy`.

---

## 6. CMake target names (HW2 lesson)

- Library of algorithms: target `Algorithm_213309941_213727837` (that is also the `.so` name).
- Library of mission control: target `MissionControl_213309941_213727837`.
- Executable target: `simulator_213309941_213727837`.

The staff injection example in HW2 used `drone_mapper` as the **library** name. Here we keep the assignment's required output names as the **real CMake target names**, so a `target_compile_definitions(simulator_... PRIVATE FLAG)` reaches the implementation sources compiled into that executable.

Plugins are separate `.so` files; their tests will typically inject into those targets.

---

## 7. Files we did not touch

All of `common/include/Common/**`, `common/CMakeLists.txt`, `Simulator/common_simulator/**`, `MissionControl/common_mission_control/**`, `CMakePresets.json`, `vcpkg.json`.

Root `CMakeLists.txt` was extended (find yaml-cpp, FetchContent fallback, `USERCOMMON_INCLUDE`). That file is a skeleton we are allowed to complete; we did not add student headers into `common/`.
