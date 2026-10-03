# C++20 Simulation Pipeline

Two related C++20 projects for a 3D drone mapping simulator. Assignment 2 stays at the **repository root**. Assignment 3 is a **separate project** under [`assignment-3/`](assignment-3/) and does not replace or rewrite the original tree.

| Folder | Project | What it is |
|---|---|---|
| `/` (this directory) | [Assignment 2 — Drone Mapper](#drone-mapper--assignment-2) | Single-process simulator. Algorithm, mission control, mocks, and scoring are linked together and talk through abstract interfaces. |
| [`assignment-3/`](assignment-3/) | [Assignment 3 — Concurrent plugin host](assignment-3/README.md) | Same mission model, but the simulator is a host process. It `dlopen`s third-party Algorithm / MissionControl `.so` plugins, runs comparative and competitive campaigns on a thread pool, and writes ranked YAML reports. |

The two assignments share the same physical model (YAML composition × missions × drones × lidars, hidden `.npy` map, MockGPS / MockLidar / MockMovement, Jaccard-style occupancy score). Assignment 3 is the system-design step: **stable APIs, dynamic loading, factory registration, ownership across `dlclose`, and concurrent plugin batches**.

Contributors: Omar Abu Shah (213309941), Jameel Najjar (213727837).

---

# Drone Mapper — Assignment 2

## Contributors

- ID: 213309941
- ID: 213727837

---

## Building

### Requirements

- CMake ≥ 3.22
- GCC ≥ 11.4 (C++20)
- Internet access (FetchContent downloads mp-units and yaml-cpp automatically)

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make drone_mapper
```

---

## Running

```bash
./drone_mapper [<input_output_files_path>]
```

If no path is given, the current directory is used.

The program expects the following files inside `<input_output_files_path>`:

| File | Description |
|------|-------------|
| `sim_compose.yaml` | Composition file listing simulation, mission, drone, and lidar configs |
| `simulation_config.yaml` | Map file path, resolution, and initial drone position |
| `mission_config.yaml` | Exploration bounds, step limit, GPS resolution |
| `drone_config.yaml` | Drone dimensions and movement limits |
| `lidar_config.yaml` | LiDAR FOV and range parameters |
| `map.npy` | 3D occupancy map (NumPy `.npy` format) |

### Example

```bash
./drone_mapper inputs/scenario_small
./drone_mapper inputs/scenario_large
./drone_mapper inputs/scenario_obstacles
```

---

## Output Format

All output is written to `<input_output_files_path>/`:

| File / Folder | Description |
|---------------|-------------|
| `map_output.txt` | Human-readable summary: score, steps, status, output map path |
| `output_results/output_map_<name>_<date>.npy` | Reconstructed 3D occupancy map in NPY format |
| `output_results/error_log.txt` | Error log (empty on a clean run) |

### `map_output.txt` example

```
Drone Mapper Simulation Report
==============================
Generated: 2026-07-05T12:00:00Z
Metric:    output_map_accuracy

Run 1:
  map:          inputs/scenario_small/map.npy
  output_map:   inputs/scenario_small/output_results/output_map_map_2026-07-05.npy
  score:        67.6
  steps:        2000
  status:       max_steps

Average score: 67.6 / 100
```

---

## Input File Formats

### `sim_compose.yaml`
```yaml
simulation_compositions:
  simulations:
    - simulation_config: "simulation_config.yaml"
      mission_configs:
        - "mission_config.yaml"
  drone_configs:
    - "drone_config.yaml"
  lidar_configs:
    - "lidar_config.yaml"
```

### `simulation_config.yaml`
```yaml
simulation_config:
  map_filename: "map.npy"
  map_resolution_cm: 10
  initial_drone_position:
    x_cm: 50
    y_cm: 50
    height_cm: 50
  initial_angle_deg: 0
  map_axes_offset:
    x_offset: 0
    y_offset: 0
    height_offset: 0
```

### `mission_config.yaml`
```yaml
mission_config:
  max_steps: 2000
  boundaries:
    x_boundary: {min_cm: 0, max_cm: 100}
    y_boundary: {min_cm: 0, max_cm: 100}
    height_boundary: {min_cm: 0, max_cm: 100}
  gps_resolution_cm: 10
  output_mapping_resolution_factor: 1
```

### `drone_config.yaml`
```yaml
drone_config:
  dimensions_cm: 30
  max_rotate_deg: 45
  max_advance_cm: 50
  max_elevate_cm: 40
```

### `lidar_config.yaml`
```yaml
lidar_config:
  z_min_cm: 20
  z_max_cm: 120
  d_cm: 2.5
  fov_circles: 5
```

---

## Map Format (`.npy`)

Maps are 3D boolean arrays stored in NumPy `.npy` format:
- Shape: `(nx, ny, nz)` — each voxel is `true` (occupied) or `false` (empty)
- Dtype: `|b1` (boolean)
- Voxel size: defined by `map_resolution_cm` in the simulation config
- World coordinate: `world_x = offset.x + index_x * resolution`

---

## External Libraries

- **mp-units** (v2.5.0): Strong-type physical quantities (distances, angles). Required by the assignment skeleton. Fetched automatically via CMake `FetchContent`.
- **yaml-cpp** (v0.8.0): YAML configuration file parsing. Fetched automatically via CMake `FetchContent`.

No manual installation required — `cmake ..` downloads both libraries automatically.

---

## Scenario Descriptions

### Scenario 1 — Small Room (`scenario_small`)
A 10×10×10 voxel space (100×100×100 cm). Solid floor at z=0, vertical wall at x=7.
Drone starts at (50, 50, 50) cm heading East. Achieved score: **67.6 / 100** (2000 steps).

### Scenario 2 — Two-Room Office (`scenario_large`)
A 20×20×15 voxel space (200×200×150 cm). Outer walls with a dividing wall at x=10.
Drone starts at (100, 100, 75) cm. Achieved score: **98.05 / 100** (5000 steps).

### Scenario 3 — Obstacle Course (`scenario_obstacles`)
A 15×15×10 voxel space (150×150×100 cm). Floor + 5 pillar obstacles.
Drone starts at (75, 75, 50) cm. Achieved score: **99.56 / 100** (3000 steps).
