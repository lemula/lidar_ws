#!/usr/bin/python3
"""Generate a low-face convex collision STL from the detailed bucket mesh."""

from __future__ import annotations

import argparse
import struct
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull


def read_binary_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"invalid binary STL: {path}")
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    expected_size = 84 + 50 * triangle_count
    if len(data) != expected_size:
        raise ValueError(f"expected binary STL of {expected_size} bytes, got {len(data)}")
    triangle_dtype = np.dtype(
        [("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]
    )
    triangles = np.frombuffer(data, dtype=triangle_dtype, count=triangle_count, offset=84)
    return triangles["vertices"].reshape(-1, 3).astype(np.float64)


def write_binary_stl(path: Path, vertices: np.ndarray, faces: np.ndarray) -> None:
    center = vertices.mean(axis=0)
    with path.open("wb") as stream:
        stream.write(b"excavator bucket low-face convex collision".ljust(80, b"\0"))
        stream.write(struct.pack("<I", len(faces)))
        for indices in faces:
            triangle = vertices[indices].copy()
            normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
            if np.dot(normal, triangle.mean(axis=0) - center) < 0.0:
                triangle[[1, 2]] = triangle[[2, 1]]
                normal = -normal
            norm = np.linalg.norm(normal)
            normal = normal / norm if norm > 0.0 else np.zeros(3)
            stream.write(struct.pack("<12fH", *(normal.tolist() + triangle.reshape(-1).tolist()), 0))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--voxel-size", type=float, default=0.01)
    args = parser.parse_args()

    points = read_binary_stl(args.input)
    quantized = np.unique(np.round(points / args.voxel_size) * args.voxel_size, axis=0)
    hull = ConvexHull(quantized)
    write_binary_stl(args.output, quantized, hull.simplices)
    print(
        f"generated {args.output}: {len(hull.simplices)} triangles, "
        f"{len(hull.vertices)} hull vertices, voxel={args.voxel_size} m"
    )


if __name__ == "__main__":
    main()
