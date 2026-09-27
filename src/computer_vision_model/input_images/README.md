# Saved images

Put the test image here as **fruit.jpg**, with matching camera calibration as
**fruit.yaml**. JPG/JPEG and PNG are supported. Choose another file with
`image_path:=/absolute/path/to/image.png`.

```bash
ros2 launch computer_vision_model fruit_image_pipeline.launch.py
```

This uses the Roboflow API and still requires `ROBOFLOW_API_KEY` and internet.
It starts no camera driver and needs no live RGB, depth, or camera TF messages.
It calculates a pose and MoveIt joint angles; it sends no motor commands.

## Getting the calibration

Run the live `fruit_plane_pipeline.launch.py` with the camera at its calibrated
position. Once the TRANSFORM phase succeeds, copy these two files from that
run's `logs/<timestamp>/captures/` directory:

- `latest_rgb.jpg` → `input_images/fruit.jpg`
- `latest_capture.yaml` → `input_images/fruit.yaml`

The YAML now contains intrinsics, distortion, image dimensions, and the
optical-camera-to-base transform captured at the image timestamp. You can also
replay directly from the logs without copying, using `image_path:=.../latest_rgb.jpg`.
The corresponding `latest_capture.yaml` will be found automatically.

For a different photo from the same fixed camera, copy its matching calibration
only if the camera position, lens settings, image resolution, and pixel crop
are unchanged. Photos from another camera or viewpoint need their own calibration;
an arbitrary internet/phone photo cannot establish distances relative to this arm.

Calibration lookup order when `calibration_path` is omitted:

1. Same filename with `.yaml` instead of the image extension.
2. `camera_calibration.yaml` beside the image.
3. `latest_capture.yaml` beside the image.

An explicit `calibration_path:=/path/to/calibration.yaml` overrides lookup.
`camera_calibration.example.yaml` documents the schema; its null values must be
replaced with measured calibration. It is deliberately not selected automatically.
Older captures without `transform_camera_to_base` need that transform added from
their matching `pipeline_result.json`, including parent/child frame names.

The image is decoded in raw pixel orientation, without EXIF display rotation,
and is neither resized nor cropped. The pipeline writes a clean JPEG for inference.
The source image and calibration are never modified. Local images/calibration
are ignored by git; this README and the schema example are versioned.
