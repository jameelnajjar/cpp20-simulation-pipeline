#!/bin/bash
set -e
export PATH="$HOME/.local/bin:$PATH"
SRC="/mnt/c/Users/97254/OneDrive/Desktop/HW2_CPP"
BUILD="$SRC/build"
mkdir -p "$BUILD"
cd "$BUILD"
echo "=== CMake Configure ==="
cmake "$SRC" -DCMAKE_BUILD_TYPE=Debug
echo "=== Build ==="
cmake --build . -j$(nproc) 2>&1
echo "=== Build done ==="
