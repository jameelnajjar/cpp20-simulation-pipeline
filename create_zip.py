#!/usr/bin/env python3
"""Create the submission zip using Python's zipfile module."""
import zipfile
import os
import sys

SRC_DIR = "/mnt/c/Users/97254/OneDrive/Desktop/submission_HW2"
ZIP_OUT = "/mnt/c/Users/97254/OneDrive/Desktop/213309941_213727837.zip"

# Remove old zip if exists
if os.path.exists(ZIP_OUT):
    os.remove(ZIP_OUT)

with zipfile.ZipFile(ZIP_OUT, 'w', zipfile.ZIP_DEFLATED) as zf:
    for root, dirs, files in os.walk(SRC_DIR):
        # Skip directories that shouldn't be in the zip
        dirs[:] = [d for d in dirs if d not in {'build', '.git', 'CMakeFiles'}]
        for fname in files:
            fpath = os.path.join(root, fname)
            # Skip compiled artifacts
            if fname.endswith(('.o', '.a', '.so', '.exe')):
                continue
            if fname in ('CMakeCache.txt', 'cmake_install.cmake'):
                continue
            # Archive path relative to SRC_DIR itself (no outer folder)
            arcname = os.path.relpath(fpath, SRC_DIR)
            zf.write(fpath, arcname)
            print(f"  added: {arcname}")

size = os.path.getsize(ZIP_OUT)
print(f"\nCreated: {ZIP_OUT}")
print(f"Size: {size / 1024:.1f} KB")
