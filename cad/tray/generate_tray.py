#!/usr/bin/env python3
"""Generate a one-piece, full-scale arm/tray/Astra Pro holder STL (millimetres).

The layout follows the current camera_x/y/z/pitch Xacro defaults. This is a
first-fit prototype: the arm uses a locating ring, not undocumented screw holes.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import vtk
from vtk.util.numpy_support import numpy_to_vtk


WORKSPACE = Path(__file__).resolve().parents[2]
CAMERA_XACRO = (
    WORKSPACE / "src/robot_arm_description/urdf/sensors/fixed_rgbd_camera.urdf.xacro"
)
BASE_STL = WORKSPACE / "src/robot_arm_description/meshes/base_link.STL"


def camera_mount_mm():
    root = ET.parse(CAMERA_XACRO).getroot()
    defaults = {
        element.attrib["name"]: float(element.attrib["default"])
        for element in root
        if element.tag.endswith("arg") and element.attrib.get("name") in
        {"camera_x", "camera_y", "camera_z", "camera_pitch"}
    }
    # camera_mast_to_camera adds 10 mm in the mast's +Y.
    return (
        1000.0 * defaults["camera_x"],
        1000.0 * (defaults["camera_y"] + 0.01),
        1000.0 * defaults["camera_z"],
        defaults["camera_pitch"],
    )


def base_radius_mm():
    data = BASE_STL.read_bytes()
    triangle_count = int.from_bytes(data[80:84], "little")
    if len(data) != 84 + 50 * triangle_count:
        raise ValueError("Expected binary STL for base_link")
    triangles = np.frombuffer(
        data,
        dtype=np.dtype([
            ("normal", "<f4", (3,)),
            ("vertices", "<f4", (3, 3)),
            ("attribute", "<u2"),
        ]),
        count=triangle_count,
        offset=84,
    )
    points = triangles["vertices"].reshape(-1, 3)
    return 1000.0 * float(np.max(np.hypot(points[:, 0], points[:, 1])))


def box_sdf(x, y, z, cx, cy, cz, hx, hy, hz):
    dx = np.abs(x - cx) - hx
    dy = np.abs(y - cy) - hy
    dz = np.abs(z - cz) - hz
    return (
        np.sqrt(np.maximum(dx, 0) ** 2 +
                np.maximum(dy, 0) ** 2 +
                np.maximum(dz, 0) ** 2)
        + np.minimum(np.maximum(np.maximum(dx, dy), dz), 0)
    )


def capsule_sdf(x, y, z, a, b, radius):
    ax, ay, az = a
    bx, by, bz = b
    abx, aby, abz = bx - ax, by - ay, bz - az
    t = np.clip(
        ((x - ax) * abx + (y - ay) * aby + (z - az) * abz) /
        (abx * abx + aby * aby + abz * abz),
        0, 1,
    )
    return np.sqrt(
        (x - (ax + t * abx)) ** 2 +
        (y - (ay + t * aby)) ** 2 +
        (z - (az + t * abz)) ** 2
    ) - radius


def field_slice(x, y, z, arm_inner_radius, mount):
    mast_x, camera_y, camera_z, pitch = mount
    mast_y = camera_y - 10.0

    # 420 x 480 mm tray, 4 mm floor, 4 mm perimeter rim.
    solid = box_sdf(x, y, z, 110, -130, -8, 210, 240, 2)
    for cx, cy, hx, hy in (
        (-98, -130, 2, 240), (318, -130, 2, 240),
        (110, -368, 210, 2), (110, 108, 210, 2),
    ):
        np.minimum(solid, box_sdf(x, y, z, cx, cy, -2, hx, hy, 6),
                   out=solid)

    # Base ring gives 1.5 mm radial clearance around the actual arm mesh.
    radial = np.hypot(x, y)
    ring = np.maximum(
        np.maximum(arm_inner_radius - radial,
                   radial - (arm_inner_radius + 6.0)),
        np.maximum(-9.0 - z, z - 4.0),
    )
    np.minimum(solid, ring, out=solid)

    # Solid foot, 34 mm hollow mast ending below the camera, and four stays.
    np.minimum(
        solid,
        box_sdf(x, y, z, mast_x, mast_y, 35, 30, 30, 43),
        out=solid,
    )
    mast_outer = box_sdf(x, y, z, mast_x, mast_y, 267.5, 17, 17, 207.5)
    mast_inner = box_sdf(x, y, z, mast_x, mast_y, 279, 12, 12, 201)
    np.minimum(solid, np.maximum(mast_outer, -mast_inner), out=solid)
    for ox, oy in ((-48, 0), (48, 0), (0, -48), (0, 48)):
        brace = capsule_sdf(
            x, y, z,
            (mast_x + ox, mast_y + oy, -4),
            (mast_x, mast_y, 160),
            7,
        )
        np.minimum(solid, brace, out=solid)

    # Two diagonal ribs support the long camera cradle near its ends.
    cosine, sine = np.cos(pitch), np.sin(pitch)
    for local_y in (-72.0, 72.0):
        # Local support point: behind and below the camera's body envelope.
        end_x = mast_x - local_y
        end_y = camera_y - 18 * cosine - 28 * sine
        end_z = camera_z + 18 * sine - 28 * cosine
        np.minimum(
            solid,
            capsule_sdf(x, y, z,
                        (mast_x, mast_y, 375),
                        (end_x, end_y, end_z), 7),
            out=solid,
        )

    # Oriented U cradle. Its cavity fits the 165 x 30 x 40 mm camera body
    # with about 2 mm side clearance. The front stays open for the optics.
    dx, dy, dz = x - mast_x, y - camera_y, z - camera_z
    lx = cosine * dy - sine * dz
    ly = -dx
    lz = sine * dy + cosine * dz
    for cx, cy, cz, hx, hy, hz in (
        (-21, 0, 0, 3, 88, 24),       # rear plate
        (-1, 0, -23.5, 23, 88, 2.5), # lower shelf
        (-1, 86.5, 0, 23, 2, 24),   # side stops
        (-1, -86.5, 0, 23, 2, 24),
        (20, 0, -20, 2, 88, 6),      # low front retention lip
    ):
        np.minimum(
            solid,
            box_sdf(lx, ly, lz, cx, cy, cz, hx, hy, hz),
            out=solid,
        )
    # Open the hollow mast through the underside, avoiding a sealed cavity.
    bore = box_sdf(x, y, z, mast_x, mast_y, 232.5, 12, 12, 247.5)
    np.maximum(solid, -bore, out=solid)
    return solid


def write_stl(output: Path, resolution: float):
    mount = camera_mount_mm()
    arm_inner_radius = base_radius_mm() + 1.5
    x_min, x_max = -104.0, 324.0
    y_min, y_max = -374.0, 114.0
    z_min, z_max = -14.0, 542.0
    nx = int(np.ceil((x_max - x_min) / resolution)) + 1
    ny = int(np.ceil((y_max - y_min) / resolution)) + 1
    nz = int(np.ceil((z_max - z_min) / resolution)) + 1
    x = x_min + resolution * np.arange(nx, dtype=np.float32)[None, :]
    y = y_min + resolution * np.arange(ny, dtype=np.float32)[:, None]
    volume = np.empty((nz, ny, nx), dtype=np.float32)
    for k in range(nz):
        z = z_min + resolution * k
        volume[k] = field_slice(x, y, z, arm_inner_radius, mount)
        if k % 80 == 0:
            print(f"Sampled {k}/{nz} layers", flush=True)

    image = vtk.vtkImageData()
    image.SetDimensions(nx, ny, nz)
    image.SetOrigin(x_min, y_min, z_min)
    image.SetSpacing(resolution, resolution, resolution)
    image.GetPointData().SetScalars(
        numpy_to_vtk(volume.ravel(order="C"), deep=False)
    )
    contour = vtk.vtkFlyingEdges3D()
    contour.SetInputData(image)
    contour.SetValue(0, 0.037)
    contour.ComputeNormalsOff()
    contour.Update()

    triangles = vtk.vtkTriangleFilter()
    triangles.SetInputConnection(contour.GetOutputPort())
    triangles.Update()
    mesh = triangles.GetOutput()
    if mesh.GetNumberOfPolys() == 0:
        raise RuntimeError("Empty tray mesh")
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = vtk.vtkSTLWriter()
    writer.SetFileName(str(output))
    writer.SetFileTypeToBinary()
    writer.SetInputData(mesh)
    if writer.Write() != 1:
        raise RuntimeError("Failed to write STL")

    edges = vtk.vtkFeatureEdges()
    edges.SetInputData(mesh)
    edges.BoundaryEdgesOn()
    edges.NonManifoldEdgesOn()
    edges.FeatureEdgesOff()
    edges.ManifoldEdgesOff()
    edges.Update()
    components = vtk.vtkPolyDataConnectivityFilter()
    components.SetInputData(mesh)
    components.SetExtractionModeToAllRegions()
    components.Update()
    bounds = mesh.GetBounds()
    print(f"Output: {output}")
    print(f"Triangles: {mesh.GetNumberOfPolys():,}")
    print(f"Boundary/non-manifold edges: {edges.GetOutput().GetNumberOfCells()}")
    print(f"Connected surface components: {components.GetNumberOfExtractedRegions()}")
    print("Envelope (mm): " +
          " x ".join(f"{bounds[i+1]-bounds[i]:.1f}" for i in (0, 2, 4)))
    print(f"Arm base radius measured: {arm_inner_radius-1.5:.2f} mm")
    print(f"Camera optical origin (mm): {mount[:3]}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path,
        default=Path(__file__).with_name("arm_mast_astra_tray_full_size.stl"),
    )
    parser.add_argument("--resolution", type=float, default=1.5,
                        help="Surface sampling pitch in mm")
    args = parser.parse_args()
    if not 0.75 <= args.resolution <= 4:
        parser.error("resolution must be in [0.75, 4] mm")
    write_stl(args.output, args.resolution)


if __name__ == "__main__":
    main()
