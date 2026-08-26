#!/bin/bash
DEST="/mnt/c/Users/97254/OneDrive/Desktop/submission_HW2"
ZIP_OUT="/mnt/c/Users/97254/OneDrive/Desktop/213309941_213727837.zip"

rm -f "$ZIP_OUT"
cd /mnt/c/Users/97254/OneDrive/Desktop

zip -r "$ZIP_OUT" submission_HW2/ \
  --exclude "*/build/*" \
  --exclude "*/.git/*" \
  --exclude "*/CMakeFiles/*" \
  --exclude "*.o" \
  --exclude "*.a" \
  --exclude "*.so" \
  --exclude "*/CMakeCache.txt" \
  --exclude "*/cmake_install.cmake"

echo "Zip created: $ZIP_OUT"
echo "Size: $(du -sh "$ZIP_OUT" | cut -f1)"
