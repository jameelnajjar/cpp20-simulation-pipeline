#!/bin/bash
export PATH="$HOME/.local/bin:$PATH"
BUILD="/mnt/c/Users/97254/OneDrive/Desktop/HW2_CPP/build"
cd "$BUILD"
echo "=== Running Tests ==="
./drone_mapper_simulation_test --gtest_output=json:/tmp/test_results.json 2>&1
echo "=== Tests done. Exit: $? ==="
