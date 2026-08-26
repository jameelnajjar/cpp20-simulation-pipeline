#!/usr/bin/env python3
"""Generate test .npy map files for HW2 scenarios."""
import numpy as np

# Scenario 1: Small 10x10x10 map with a central wall.
# Shape is (nx, ny, nz), dtype bool.
def make_small():
    arr = np.zeros((10, 10, 10), dtype=bool)
    # Wall at x=7 (all y, all z)
    arr[7, :, :] = True
    # Floor at z=0
    arr[:, :, 0] = True
    return arr

# Scenario 2: Larger 20x20x15 map with two rooms.
def make_large():
    arr = np.zeros((20, 20, 15), dtype=bool)
    # Outer walls
    arr[0, :, :] = True
    arr[19, :, :] = True
    arr[:, 0, :] = True
    arr[:, 19, :] = True
    arr[:, :, 0] = True
    # Dividing wall with a passage at y=10
    arr[10, :8, :8] = True
    arr[10, 12:, :8] = True
    return arr

# Scenario 3: Obstacle course 15x15x10.
def make_obstacles():
    arr = np.zeros((15, 15, 10), dtype=bool)
    # Scattered obstacles
    arr[3, 3, :5] = True
    arr[7, 7, :5] = True
    arr[11, 3, :5] = True
    arr[3, 11, :5] = True
    arr[11, 11, :5] = True
    arr[:, :, 0] = True
    return arr

if __name__ == "__main__":
    import os
    os.makedirs(os.path.dirname(__file__), exist_ok=True)
    out_dir = os.path.dirname(os.path.abspath(__file__))
    np.save(os.path.join(out_dir, "scenario_small.npy"), make_small())
    np.save(os.path.join(out_dir, "scenario_large.npy"), make_large())
    np.save(os.path.join(out_dir, "scenario_obstacles.npy"), make_obstacles())
    print("Maps generated.")
