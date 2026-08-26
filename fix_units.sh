#!/bin/bash
# Fix force_numerical_value_in calls across all .cpp files
SRC="/mnt/c/Users/97254/OneDrive/Desktop/HW2_CPP"
FILES=$(find "$SRC/tests" -name "*.cpp")
for f in $FILES; do
    perl -i -pe '
        s/\.force_numerical_value_in\(x_extent\[cm\]\)/.numerical_value_in(cm)/g;
        s/\.force_numerical_value_in\(y_extent\[cm\]\)/.numerical_value_in(cm)/g;
        s/\.force_numerical_value_in\(z_extent\[cm\]\)/.numerical_value_in(cm)/g;
        s/\.force_numerical_value_in\(horizontal_angle\[deg\]\)/.numerical_value_in(deg)/g;
        s/\.force_numerical_value_in\(altitude_angle\[deg\]\)/.numerical_value_in(deg)/g;
        s/\.force_numerical_value_in\(cm\)/.numerical_value_in(cm)/g;
    ' "$f"
    echo "Fixed: $f"
done
echo "All done"
