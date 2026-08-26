#!/bin/bash
set -e
BASE=/mnt/c/Users/97254/OneDrive/Desktop/HW2_CPP
BUILD=$BASE/build

for scenario in scenario_small scenario_large scenario_obstacles; do
    SCENARIO_DIR="$BASE/inputs/$scenario"
    OUT_DIR="$SCENARIO_DIR/original_output"
    mkdir -p "$OUT_DIR"
    echo "=== Running $scenario ==="
    cd "$SCENARIO_DIR"
    timeout 90 "$BUILD/drone_mapper_simulation" "$SCENARIO_DIR/sim_compose.yaml" "$OUT_DIR" 2>&1
    echo "Done $scenario - exit: $?"
    ls "$OUT_DIR/" 2>&1
done
echo "All scenarios completed."
