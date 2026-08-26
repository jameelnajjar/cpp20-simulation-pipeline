#!/bin/bash
BASE="/mnt/c/Users/97254/OneDrive/Desktop/HW2_CPP"
DEST="/mnt/c/Users/97254/OneDrive/Desktop/submission_HW2"

rm -rf "$DEST"
mkdir -p "$DEST"

cp "$BASE/CMakeLists.txt" "$DEST/"
cp "$BASE/README.md" "$DEST/"
cp "$BASE/HLD.pdf" "$DEST/"

cp -r "$BASE/include" "$DEST/"
cp -r "$BASE/src" "$DEST/"

mkdir -p "$DEST/inputs"
for s in scenario_small scenario_large scenario_obstacles; do
  cp -r "$BASE/inputs/$s" "$DEST/inputs/$s"
done

echo "Done creating submission_HW2"
find "$DEST" | grep -v "output_results" | head -80
echo "--- original_output for scenario_small ---"
ls "$DEST/inputs/scenario_small/original_output/"
ls "$DEST/inputs/scenario_small/original_output/output_results/"
