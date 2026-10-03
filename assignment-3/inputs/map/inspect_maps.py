#!/usr/bin/env python3
"""Prints shape / dtype / occupancy of every .npy scenario map, using only the stdlib.

The NPY format is documented enough to read by hand: a 6-byte magic, a 2-byte version,
a little-endian header length, then an ASCII Python-dict header, then raw C-order data.
"""
import ast
import struct
from pathlib import Path


def read_npy(path):
    with open(path, "rb") as handle:
        magic = handle.read(6)
        if magic != b"\x93NUMPY":
            raise ValueError(f"{path}: not an NPY file")
        major, _minor = handle.read(2)
        header_len = struct.unpack("<H" if major == 1 else "<I",
                                   handle.read(2 if major == 1 else 4))[0]
        header = ast.literal_eval(handle.read(header_len).decode("latin1").strip())
        return header["descr"], header["shape"], header["fortran_order"], handle.read()


for path in sorted(Path(__file__).parent.glob("*.npy")):
    descr, shape, fortran, data = read_npy(path)
    nx, ny, nz = shape
    occupied = sum(1 for b in data if b != 0)
    print(f"{path.name:24s} shape={shape} dtype={descr} fortran={fortran} "
          f"bytes={len(data)} occupied={occupied} ({100.0 * occupied / len(data):.1f}%)")
    # index = ((x * ny) + y) * nz + z for C-order (nx, ny, nz)
    per_z = [0] * nz
    for x in range(nx):
        for y in range(ny):
            base = (x * ny + y) * nz
            for z in range(nz):
                if data[base + z] != 0:
                    per_z[z] += 1
    print(f"    occupied per z index: {per_z}")
