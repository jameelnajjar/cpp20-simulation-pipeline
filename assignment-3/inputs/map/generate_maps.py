#!/usr/bin/env python3
"""Generate occupancy .npy maps for local smoke tests (not staff maps)."""
import os
import struct

def write_npy(filename, shape, occupied):
    nx, ny, nz = shape
    data = bytearray(nx * ny * nz)
    for (x, y, z) in occupied:
        if 0 <= x < nx and 0 <= y < ny and 0 <= z < nz:
            data[x * (ny * nz) + y * nz + z] = 1
    hdr = "{'descr': '|i1', 'fortran_order': False, 'shape': (%d, %d, %d), }" % (nx, ny, nz)
    total_before = 10 + len(hdr) + 1
    pad = (64 - (total_before % 64)) % 64
    hdr = hdr + (" " * pad) + "\n"
    os.makedirs(os.path.dirname(filename) or ".", exist_ok=True)
    with open(filename, "wb") as f:
        f.write(b"\x93NUMPY")
        f.write(bytes([1, 0]))
        f.write(struct.pack("<H", len(hdr)))
        f.write(hdr.encode("latin-1"))
        f.write(bytes(data))
    print("Wrote", filename, "shape=", shape)


def add_floor(occ, nx, ny):
    for x in range(nx):
        for y in range(ny):
            occ.add((x, y, 0))


def add_box(occ, x0, x1, y0, y1, z0, z1):
    for x in range(x0, x1):
        for y in range(y0, y1):
            for z in range(z0, z1):
                occ.add((x, y, z))


out_dir = os.path.dirname(os.path.abspath(__file__))

# small: 20x20x10 at 10cm -> 200x200x100 cm (matches small_mission_room roughly)
occ_small = set()
add_floor(occ_small, 20, 20)
add_box(occ_small, 7, 8, 0, 20, 0, 10)  # wall
write_npy(os.path.join(out_dir, "scenario_small.npy"), (20, 20, 10), occ_small)

# big / large: 40x30x25
occ_big = set()
add_floor(occ_big, 40, 30)
add_box(occ_big, 0, 1, 0, 30, 0, 20)
add_box(occ_big, 39, 40, 0, 30, 0, 20)
add_box(occ_big, 0, 40, 0, 1, 0, 20)
add_box(occ_big, 0, 40, 29, 30, 0, 20)
add_box(occ_big, 20, 21, 0, 12, 0, 12)
add_box(occ_big, 20, 21, 18, 30, 0, 12)
write_npy(os.path.join(out_dir, "scenario_big.npy"), (40, 30, 25), occ_big)
write_npy(os.path.join(out_dir, "scenario_large.npy"), (40, 30, 25), occ_big)

# house: 29x30x15 (290x300x150 cm at 10cm)
occ_house = set()
add_floor(occ_house, 29, 30)
add_box(occ_house, 0, 1, 0, 30, 0, 15)
add_box(occ_house, 28, 29, 0, 30, 0, 15)
add_box(occ_house, 0, 29, 0, 1, 0, 15)
add_box(occ_house, 0, 29, 29, 30, 0, 15)
add_box(occ_house, 14, 15, 0, 12, 0, 12)
add_box(occ_house, 14, 15, 18, 30, 0, 12)
write_npy(os.path.join(out_dir, "scenario_house.npy"), (29, 30, 15), occ_house)

print("All local maps generated.")
