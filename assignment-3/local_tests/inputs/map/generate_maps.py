#!/usr/bin/env python3
"""Tiny occupancy maps for local HW3 tests (not staff maps)."""
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

# 12x12x8 at 10cm -> 120 x 120 x 80 cm
nx, ny, nz = 12, 12, 8

occ_open = set()
add_floor(occ_open, nx, ny)
add_box(occ_open, 0, 1, 0, ny, 0, nz)
add_box(occ_open, nx - 1, nx, 0, ny, 0, nz)
add_box(occ_open, 0, nx, 0, 1, 0, nz)
add_box(occ_open, 0, nx, ny - 1, ny, 0, nz)
write_npy(os.path.join(out_dir, "open.npy"), (nx, ny, nz), occ_open)

occ_wall = set(occ_open)
# Slab at x=6 (60-70cm), spanning most of Y, so +X from x=25cm hits it.
add_box(occ_wall, 6, 7, 2, 10, 0, nz)
write_npy(os.path.join(out_dir, "walled.npy"), (nx, ny, nz), occ_wall)

print("Local test maps generated.")
