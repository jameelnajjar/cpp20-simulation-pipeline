#!/usr/bin/env python3
"""Verify the submission zip meets all requirements."""
import zipfile
import sys

ZIP_PATH = "/mnt/c/Users/97254/OneDrive/Desktop/213309941_213727837.zip"

with zipfile.ZipFile(ZIP_PATH) as z:
    names = z.namelist()

checks = []

def check(desc, result, detail=""):
    status = "PASS" if result else "FAIL"
    checks.append((status, desc, detail))
    print(f"[{status}] {desc}" + (f" — {detail}" if detail else ""))
    return result

# Top-level required files
check("CMakeLists.txt present", "CMakeLists.txt" in names)
check("README.md present", "README.md" in names)
check("HLD.pdf present", "HLD.pdf" in names)

# No tests/ directory
test_files = [n for n in names if n.startswith("tests/")]
check("No tests/ directory", len(test_files) == 0, f"{len(test_files)} test files found")

# No build artifacts (binaries except .npy)
bad_exts = ['.o', '.a', '.so', '.exe']
binary_files = [n for n in names if any(n.endswith(e) for e in bad_exts)]
# .npy files are OK
check("No compiled binaries", len(binary_files) == 0, str(binary_files))

# No build/ directory
build_files = [n for n in names if n.startswith("build/") or "CMakeFiles" in n or n == "CMakeCache.txt"]
check("No build artifacts", len(build_files) == 0, f"found: {build_files[:3]}")

# Three scenario folders
for s in ["scenario_small", "scenario_large", "scenario_obstacles"]:
    base = f"inputs/{s}/"
    has_drone = any(n.startswith(base) and "drone" in n for n in names)
    has_lidar = any(n.startswith(base) and "lidar" in n for n in names)
    has_mission = any(n.startswith(base) and "mission" in n for n in names)
    has_sim = any(n.startswith(base) and "simulation_config" in n for n in names)
    has_compose = f"{base}sim_compose.yaml" in names
    has_map_npy = f"{base}map.npy" in names

    orig = f"{base}original_output/"
    has_map_txt = f"{orig}map_output.txt" in names
    has_npy_out = any(n.startswith(f"{orig}output_results/") and n.endswith(".npy") for n in names)

    check(f"{s}: all yaml configs present",
          all([has_drone, has_lidar, has_mission, has_sim, has_compose]),
          f"drone={has_drone} lidar={has_lidar} mission={has_mission} sim={has_sim} compose={has_compose}")
    check(f"{s}: map.npy present", has_map_npy)
    check(f"{s}: original_output/map_output.txt present", has_map_txt)
    check(f"{s}: original_output/output_results/*.npy present", has_npy_out)

# Check CMakeLists.txt has drone_mapper target, not test target
cmake_content = zipfile.ZipFile(ZIP_PATH).read("CMakeLists.txt").decode()
check("CMakeLists.txt has drone_mapper target", "add_executable(drone_mapper " in cmake_content)
check("CMakeLists.txt has no test/ reference", "tests/" not in cmake_content)
check("CMakeLists.txt has -Wall -Wextra -Werror -pedantic", "-Wall" in cmake_content and "-Wextra" in cmake_content and "-Werror" in cmake_content and "-pedantic" in cmake_content)
check("CMakeLists.txt uses c++20", "cxx_std_20" in cmake_content or "CMAKE_CXX_STANDARD 20" in cmake_content)

# src files
check("src/drone_mapper_main.cpp present", "src/drone_mapper_main.cpp" in names)

# Summary
failures = [c for c in checks if c[0] == "FAIL"]
print(f"\n{'='*50}")
print(f"Results: {len(checks) - len(failures)}/{len(checks)} checks passed")
if failures:
    print("FAILURES:")
    for _, desc, detail in failures:
        print(f"  - {desc}: {detail}")
else:
    print("All checks passed!")
