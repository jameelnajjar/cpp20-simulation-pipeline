#!/bin/bash
DEST="/mnt/c/Users/97254/OneDrive/Desktop/submission_HW2"

# Remove map_output.txt at scenario root level (only keep original_output ones)
rm -f "$DEST/inputs/scenario_small/map_output.txt"
rm -f "$DEST/inputs/scenario_large/map_output.txt"
rm -f "$DEST/inputs/scenario_obstacles/map_output.txt"

# Remove output_results at scenario root level (keep only original_output/output_results)
rm -rf "$DEST/inputs/scenario_small/output_results"
rm -rf "$DEST/inputs/scenario_large/output_results"
rm -rf "$DEST/inputs/scenario_obstacles/output_results"

echo "Cleanup done"
echo "=== scenario_small ==="
ls "$DEST/inputs/scenario_small/"
echo "=== scenario_small/original_output ==="
ls "$DEST/inputs/scenario_small/original_output/"
echo "=== scenario_small/original_output/output_results ==="
ls "$DEST/inputs/scenario_small/original_output/output_results/"
