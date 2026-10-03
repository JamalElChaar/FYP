"""Load an image with its matching intrinsics and recorded optical-to-base pose."""
from copy import deepcopy
from pathlib import Path

import cv2
import numpy as np
import yaml


def correct_default_principal_point(metadata):
    """Undo the Astra driver's 4:3 assumption in a recorded principal point.

    When no calibration file is loaded, the driver synthesises intrinsics with
    cy = width * 3/8 - 0.5 (astra_camera/src/utils.cpp). That is the true
    centre only for a 4:3 mode; at 16:9 it is wrong by a quarter of the image
    height -- 479.5 instead of 359.5 at 1280x720, about 8 degrees of ray tilt.
    Because the plane intersection pins z, that tilt lands entirely in x and y:
    roughly 150 mm of ground error at this camera's geometry.

    Only a value matching the driver's formula is touched, and only when the
    image is not 4:3, so a genuine calibration is never overwritten. Returns
    (old, new) when a correction was made, otherwise None.
    """
    try:
        width = int(metadata['rgb']['width'])
        height = int(metadata['rgb']['height'])
        recorded = float(metadata['camera_intrinsics']['cy'])
    except (KeyError, TypeError, ValueError):
        return None
    if width <= 0 or height <= 0:
        return None
    driver_default = width * 3.0 / 8.0 - 0.5
    true_centre = height / 2.0 - 0.5
    # Nothing to do when the formula happens to be right (a 4:3 capture).
    if abs(driver_default - true_centre) <= 1.0:
        return None
    if abs(recorded - driver_default) > 0.51:
        return None          # a real calibration, not the synthesised default
    metadata['camera_intrinsics']['cy'] = true_centre
    return recorded, true_centre


def prepare_saved_image(image_path, calibration_path, output_directory):
    source = Path(image_path).expanduser().resolve()
    if not source.is_file():
        raise ValueError(f'Input image not found: {source}. Put an image in input_images/ or pass image_path:=...')
    # Keep the raw sensor pixel orientation; EXIF display rotation invalidates K.
    image = cv2.imread(str(source), cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
    if image is None or image.size == 0:
        raise ValueError(f'Cannot decode input image: {source}')
    if calibration_path:
        calibration = Path(calibration_path).expanduser().resolve()
    else:
        candidates = [source.with_suffix('.yaml'), source.parent / 'camera_calibration.yaml',
                      source.parent / 'latest_capture.yaml']
        calibration = next((p for p in candidates if p.is_file()), candidates[0])
    if not calibration.is_file():
        raise ValueError('Matching calibration YAML not found. Supply calibration_path:=... or place '
                         f'{source.stem}.yaml, camera_calibration.yaml, or latest_capture.yaml beside the image. '
                         'Metric arm coordinates need intrinsics and a recorded camera-to-base transform.')
    metadata = yaml.safe_load(calibration.read_text(encoding='utf-8'))
    if not isinstance(metadata, dict):
        raise ValueError('Calibration YAML must contain a mapping')
    metadata = deepcopy(metadata)
    # Fix the driver's 4:3 principal point before anything reads the intrinsics.
    corrected = correct_default_principal_point(metadata)
    if corrected:
        metadata.setdefault('_corrections', {})['cy'] = {
            'recorded': corrected[0], 'used': corrected[1],
            'reason': "Astra driver default assumes 4:3 (cy = width*3/8 - 0.5); "
                      "replaced with the true centre for this image's aspect ratio"}
    try:
        width, height = int(metadata['rgb']['width']), int(metadata['rgb']['height'])
        frame = str(metadata['capture']['rgb_frame_id']).strip()
        intrinsics = metadata['camera_intrinsics']
        fx, fy, cx, cy = [float(intrinsics[k]) for k in ('fx', 'fy', 'cx', 'cy')]
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError('Calibration requires rgb.width/height, capture.rgb_frame_id, '
                         'and numeric camera_intrinsics.fx/fy/cx/cy. Use a live latest_capture.yaml.') from error
    if (width, height) != (image.shape[1], image.shape[0]):
        raise ValueError(f'Image is {image.shape[1]}x{image.shape[0]} but calibration is {width}x{height}; '
                         'use the original image without resizing/cropping')
    if not frame or frame == 'None' or not np.isfinite([fx, fy, cx, cy]).all() or min(fx, fy) <= 0:
        raise ValueError('Calibration has an invalid optical frame or camera intrinsics')
    tf = metadata.get('transform_camera_to_base', {})
    try:
        translation = np.asarray(tf['translation_m'], dtype=float)
        quaternion = np.asarray(tf['quaternion_xyzw'], dtype=float)
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError('Calibration needs transform_camera_to_base with translation_m and quaternion_xyzw. '
                         'A completed live projection now saves these in latest_capture.yaml.') from error
    if (translation.shape != (3,) or quaternion.shape != (4,) or
            not np.isfinite(translation).all() or not np.isfinite(quaternion).all() or
            np.linalg.norm(quaternion) < 1e-9):
        raise ValueError('Invalid recorded camera-to-base translation/quaternion')
    # Frame names make the direction of the saved transform unambiguous.
    if tf.get('parent_frame') != 'base_link' or tf.get('child_frame') != frame:
        raise ValueError('Recorded transform must have parent_frame: base_link and '
                         'child_frame equal to capture.rgb_frame_id (the RGB optical frame)')
    metadata['transform_camera_to_base']['quaternion_xyzw'] = (quaternion / np.linalg.norm(quaternion)).tolist()
    metadata['capture']['source_rgb_stamp'] = metadata['capture'].get('rgb_stamp')
    metadata['capture']['rgb_stamp'] = [0, 0]
    metadata['capture']['rgb_stamp_sec'] = 0.0
    metadata['capture']['require_depth'] = False
    metadata['capture']['input_source'] = 'saved_image'
    metadata['capture']['source_image'] = str(source)
    metadata['capture']['source_calibration'] = str(calibration)
    output = Path(output_directory).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    destination = output / 'latest_rgb.jpg'
    temporary_image = output / 'latest_rgb.tmp.jpg'
    if not cv2.imwrite(str(temporary_image), image, [cv2.IMWRITE_JPEG_QUALITY, 95]):
        raise ValueError('Failed to write saved-image capture')
    temporary_image.replace(destination)
    metadata['rgb']['file'] = destination.name
    metadata_path = output / 'latest_capture.yaml'
    temporary_metadata = metadata_path.with_suffix('.yaml.tmp')
    temporary_metadata.write_text(yaml.safe_dump(metadata, sort_keys=False), encoding='utf-8')
    temporary_metadata.replace(metadata_path)
    return destination, metadata
