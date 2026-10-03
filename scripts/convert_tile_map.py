import sys
from pathlib import Path

import laspy
import numpy as np
from pypcd4 import Encoding, PointCloud


VOXEL_SIZE = 0.1  # 米，必须大于 0
UAV_MODE = False  # True：UAV；False：扫描仪


def read_map(path):
    if path.suffix.lower() == ".las":
        cloud = laspy.read(path)
        fields = {
            name: np.array(cloud[name])
            for name in cloud.point_format.dimension_names
            if name not in ("X", "Y", "Z")
        }
        fields.update(
            x=np.array(cloud.x),
            y=np.array(cloud.y),
            z=np.array(cloud.z),
        )

        if all(name in fields for name in ("red", "green", "blue")):
            r = fields["red"].astype(np.uint32) >> 8
            g = fields["green"].astype(np.uint32) >> 8
            b = fields["blue"].astype(np.uint32) >> 8
            fields["rgb"] = ((r << 16) | (g << 8) | b).view(np.float32)

        return fields

    if path.suffix.lower() == ".pcd":
        cloud = PointCloud.from_path(path)
        return {
            name: cloud.pc_data[name].copy()
            for name in cloud.pc_data.dtype.names
        }

    raise ValueError("仅支持 LAS 和 PCD")


def voxel_filter(fields):
    xyz = np.column_stack(
        (fields["x"], fields["y"], fields["z"])
    ).astype(np.float64)
    grid = np.floor(xyz / VOXEL_SIZE).astype(np.int64)
    _, indices = np.unique(grid, axis=0, return_index=True)
    indices = np.sort(indices)
    return {name: values[indices] for name, values in fields.items()}


def transform_map(fields):
    x, y = fields["x"], fields["y"]
    if UAV_MODE:
        fields["x"] = y.astype(np.float32)
        fields["y"] = (-x).astype(np.float32)
    else:
        fields["x"] = x.astype(np.float32)
        fields["y"] = y.astype(np.float32)
    fields["z"] = fields["z"].astype(np.float32)

    count = len(x)
    fields["intensity"] = np.asarray(
        fields.get("intensity", np.zeros(count)), dtype=np.float32
    )
    fields["time"] = np.asarray(
        fields.get("time", np.zeros(count)), dtype=np.float64
    )
    return fields


def save_tile_map(fields, directory):
    xy = np.column_stack((fields["x"], fields["y"])).astype(np.float64)
    grid = np.floor(xy * float(np.float32(1.0 / 100.0)) + 0.5).astype(np.int64)
    tiles, tile_ids = np.unique(grid, axis=0, return_inverse=True)

    directory.mkdir(parents=True)
    names = tuple(fields)
    types = tuple(fields[name].dtype for name in names)

    with (directory / "index.txt").open("w") as index:
        index.write("0 0 0\n")

        for tile_id, (gx, gy) in enumerate(tiles):
            mask = tile_ids == tile_id
            points = [fields[name][mask] for name in names]
            tile = PointCloud.from_points(points, names, types)
            tile.save(directory / f"{tile_id}.pcd", encoding=Encoding.BINARY)
            index.write(f"{tile_id} {gx} {gy} {tile_id}.pcd\n")

        index.write("# functional points\n")
        index.write("start 0 0 0 0 0 0 1\n")


def main():
    src, dst = map(Path, sys.argv[1:])
    fields = read_map(src)
    fields = voxel_filter(fields)
    fields = transform_map(fields)
    save_tile_map(fields, dst)
    print(dst)


if __name__ == "__main__":
    main()
