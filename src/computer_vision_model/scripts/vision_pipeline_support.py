"""Geometry and append-only phase logging shared by the vision pipeline."""
import datetime
import json
import os
from pathlib import Path

import cv2
import numpy as np


def phase_log(path, phase, status, detail, payload=None):
    if not path:
        return
    # The API key must never appear in logs, even inside SDK exception text.
    key = os.environ.get('ROBOFLOW_API_KEY', '')
    record = (f"\n[{datetime.datetime.now().astimezone().isoformat()}] "
              f"{phase} — {status}\n{detail}\n")
    if payload is not None:
        record += json.dumps(payload, indent=2, allow_nan=False) + '\n'
    if key:
        record = record.replace(key, '[REDACTED]')
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    # Single append write keeps records from different nodes together.
    fd = os.open(destination, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
    try:
        os.write(fd, record.encode('utf-8'))
    finally:
        os.close(fd)


def quaternion_matrix(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-9:
        raise ValueError('Invalid rotation quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def project_box_to_plane(detection, metadata, translation, quaternion, plane_z):
    """Undistort a raw-image pixel, then intersect its TF-rotated ray with Z=plane_z.

    Returns a point on the assumed plane, not a measurement of fruit height.
    Translation/rotation map the RGB optical frame into base_link. Units: metres.
    """
    intr = metadata['camera_intrinsics']
    width, height = int(metadata['rgb']['width']), int(metadata['rgb']['height'])
    u, v = float(detection['center_x']), float(detection['center_y'])
    if not (np.isfinite([u, v]).all() and 0 <= u < width and 0 <= v < height):
        raise ValueError('Bounding-box centre is outside the captured image')
    fx, fy, cx, cy = [float(intr[k]) for k in ('fx', 'fy', 'cx', 'cy')]
    if not np.isfinite([fx, fy, cx, cy]).all() or min(fx, fy) <= 0:
        raise ValueError('Invalid camera intrinsics')
    if any(int(b) > 1 for b in intr.get('binning', [0, 0])):
        raise ValueError('Binned camera images require adjusted intrinsics; use full resolution')
    roi = intr.get('roi', [0, 0, 0, 0])
    if roi not in ([0, 0, 0, 0], [0, 0, width, height]):
        raise ValueError('Cropped CameraInfo ROI is unsupported; use the full image')
    matrix = np.array([[fx, 0., cx], [0., fy, cy], [0., 0., 1.]])
    distortion = np.asarray(intr.get('d', []), dtype=float)
    if not np.isfinite(distortion).all():
        raise ValueError('Invalid distortion coefficients')
    model = intr.get('distortion_model', '')
    pixel = np.array([[[u, v]]], dtype=np.float64)
    if model == 'equidistant':
        if distortion.size != 4:
            raise ValueError('Equidistant distortion requires four coefficients')
        xy = cv2.fisheye.undistortPoints(pixel, matrix, distortion).reshape(2)
    elif model in ('', 'plumb_bob', 'rational_polynomial'):
        xy = cv2.undistortPoints(pixel, matrix, distortion if distortion.size else None).reshape(2)
    else:
        raise ValueError(f'Unsupported distortion model: {model}')
    ray_camera = np.array([xy[0], xy[1], 1.])
    origin = np.asarray(translation, dtype=float)
    ray_base = quaternion_matrix(quaternion) @ ray_camera
    if not np.isfinite([*origin, *ray_base, plane_z]).all():
        raise ValueError('Non-finite transform or plane height')
    if abs(ray_base[2]) < 1e-8:
        raise ValueError('Viewing ray is parallel to the fruit plane')
    distance = (plane_z - origin[2]) / ray_base[2]
    if distance <= 0:
        raise ValueError('Fruit plane intersects behind the camera')
    point_camera = distance * ray_camera
    point_base = origin + distance * ray_base
    point_base[2] = float(plane_z)
    return {'pixel_uv': [u, v], 'ray_camera': ray_camera.tolist(),
            'point_camera_m': point_camera.tolist(), 'point_base_m': point_base.tolist()}
