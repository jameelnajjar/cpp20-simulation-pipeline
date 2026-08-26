#!/usr/bin/env python3
"""Generate NPY map files using only the Python standard library."""
import struct
import os

def write_npy(filename, shape, occupied_set):
    nx, ny, nz = shape
    data = bytearray(nx * ny * nz)
    for (x, y, z) in occupied_set:
        if 0 <= x < nx and 0 <= y < ny and 0 <= z < nz:
            data[x * (ny * nz) + y * nz + z] = 1

    hdr = "{'descr': '|b1', 'fortran_order': False, 'shape': (%d, %d, %d), }" % (nx, ny, nz)
    # Pad so that (6+1+1+2+len(hdr)) % 64 == 0
    total_before = 10 + len(hdr) + 1  # 10 = magic+ver+hlen, +1 for trailing newline
    pad = (64 - (total_before % 64)) % 64
    hdr = hdr + ' ' * pad + '\n'
    hlen = len(hdr)

    os.makedirs(os.path.dirname(filename) or '.', exist_ok=True)
    with open(filename, 'wb') as f:
        f.write(b'\x93NUMPY')
        f.write(bytes([1, 0]))
        f.write(struct.pack('<H', hlen))
        f.write(hdr.encode('latin-1'))
        f.write(bytes(data))
    print('Wrote', filename, 'shape=(%d,%d,%d)' % (nx, ny, nz))


out_dir = os.path.dirname(os.path.abspath(__file__))

# --- Scenario 1: small 10x10x10, wall + floor ---
occ1 = set()
for y in range(10):
    for z in range(10):
        occ1.add((7, y, z))  # wall
for x in range(10):
    for y in range(10):
        occ1.add((x, y, 0))  # floor
write_npy(os.path.join(out_dir, 'scenario_small.npy'), (10, 10, 10), occ1)

# --- Scenario 2: larger 20x20x15, two-room layout ---
occ2 = set()
for y in range(20):
    for z in range(15):
        occ2.add((0, y, z))
        occ2.add((19, y, z))
for x in range(20):
    for z in range(15):
        occ2.add((x, 0, z))
        occ2.add((x, 19, z))
for x in range(20):
    for y in range(20):
        occ2.add((x, y, 0))
# dividing wall with passage
for y in range(8):
    for z in range(8):
        occ2.add((10, y, z))
for y in range(12, 20):
    for z in range(8):
        occ2.add((10, y, z))
write_npy(os.path.join(out_dir, 'scenario_large.npy'), (20, 20, 15), occ2)

# --- Scenario 3: obstacle course 15x15x10 ---
occ3 = set()
for x in range(15):
    for y in range(15):
        occ3.add((x, y, 0))  # floor
for ox, oy in [(3, 3), (7, 7), (11, 3), (3, 11), (11, 11)]:
    for z in range(1, 5):
        occ3.add((ox, oy, z))
write_npy(os.path.join(out_dir, 'scenario_obstacles.npy'), (15, 15, 10), occ3)

print('All maps generated.')
