# computer_vision_model

This package captures synchronized Orbbec Astra Pro RGB-D data and can send the
captured JPG to a Roboflow object-detection model.

```text
astra_camera -> rgbd_capture_node -> latest_rgb.jpg -> fruit_detection_node
                                      matching depth    -> detection JSON topic
```

## One-shot RGB → plane coordinates → MoveIt joint angles

```bash
cd /home/jamal/FYP_ws2
source /opt/ros/humble/setup.bash
colcon build --packages-select computer_vision_model robot_arm_description robot_arm_gazebo --symlink-install
source install/setup.bash
# Set ROBOFLOW_API_KEY in this shell before launching; do not put it in a launch argument.
ros2 launch computer_vision_model fruit_plane_pipeline.launch.py
```

This starts the Astra Pro RGB camera, an RGB-only capture service, Roboflow
inference, `camera_to_base_node`, and MoveIt for IK. It captures **once**, selects
the highest-confidence detection above `minimum_confidence` (default 0.25),
publishes a pose, and requests joint angles. It never executes a trajectory or
publishes motor commands. Nodes remain running so the latched results can be
inspected; stop the launch with Ctrl+C and relaunch for a new run.

**Geometry:** fruits are assumed to lie on `base_link` Z = **-0.006 m**. The
bounding-box centre is undistorted using the captured CameraInfo, converted to a
viewing ray, rotated using TF at the image timestamp, and intersected with that
plane. The result is not a measurement of fruit height. No depth stream is
required or used by this launch. Raw RGB image dimensions and calibration must
match; cropped/binned CameraInfo is rejected unless the intrinsics are adapted.

The camera body reference defaults to **(0.22, -0.29, 0.434) m**, RPY
**(0, 0.872664626, 1.570796327) rad**. It is 44 cm above the base support plane.
The driver supplies the camera-body-to-RGB-optical transform. These defaults
match the model layout; use measured/calibrated `cam_x`, `cam_y`, `cam_z`,
`cam_roll`, `cam_pitch`, and `cam_yaw` on the real setup. `cam_y` here is the
camera body Y (-0.29), whereas the description launch's `camera_y` is the mast
Y (-0.30) before the holder's +0.01 m offset.

The default target has Z = -0.006 m with **no extra approach offset**.
`target_offset_z` can add an explicit base-frame height above the projected
point. The target orientation uses the existing arm configuration, normalised
quaternion (-0.548, 0.447, 0.006, 0.707); override `ox`, `oy`, `oz`, `ow` as needed.
IK targets the current **link_6 origin** (`tip_link`), not a separately calibrated
fork-tip frame. This launch does not change the pending end-effector redesign.

IK uses recent `/joint_states` if present, otherwise the configured SRDF home
angles as an explicitly logged seed, not as measured hardware positions.
Collision checking is enabled for the loaded MoveIt scene. IK success only
establishes a joint solution; it does not check a motion path or automatically
add the physical table/fruit/tray to the planning scene. Unreachable targets,
missing TF, missing camera calibration, API errors, and timeouts are recorded.

### Logs and output

Each run creates `src/computer_vision_model/logs/<timestamp>/` in this workspace:

- `launch.log`: combined launch/process stdout and stderr after launch setup.
- `phases.txt`: timestamped phase-by-phase report: capture paths, every retained
  detection, selected box, camera-frame ray and point, TF, base-frame point,
  target pose, IK seed, joint angles in radians/degrees, and errors.
- `captures/latest_rgb.jpg`, `latest_capture.yaml`, `latest_detections.json`.
  After a successful projection, the YAML also includes the optical-to-base
  transform so the image/calibration pair can be replayed later.
- `captures/pipeline_result.json`: pose/IK success or failure with available data.
- `ros/`: additional ROS node logs.

`logs_directory` overrides the root. Captures and logs are excluded from git.
No API key is included in the phase report. These are ROS joint angles; any
ESP32 servo offsets/directions belong to the hardware controller.

Published results:

- `/computer_vision/detections` (`std_msgs/String`, JSON).
- `/computer_vision/fruit_pose` (`geometry_msgs/PoseStamped`, metres).
- `/computer_vision/ik_solution` (`std_msgs/String`, JSON, only on IK success).
- `/computer_vision/pipeline_result` (`std_msgs/String`, JSON, success/error).

If the camera, robot description, and MoveIt are already running, avoid duplicate
processes and camera transforms:

```bash
ros2 launch computer_vision_model fruit_plane_pipeline.launch.py \
  start_camera:=false start_moveit:=false publish_camera_tf:=false
```

For Gazebo, additionally use `use_sim_time:=true`. The existing camera TF must
connect the image's optical frame to `base_link`. The model launch and the
Gazebo camera output are separate from this one-shot processing launch.

Geometry regression checks (no network/camera):

```bash
python3 -m unittest discover -s src/computer_vision_model/test -v
```

## Run from a saved image (no connected camera)

Place `fruit.jpg` and its matching `fruit.yaml` in
[`input_images/`](input_images/README.md), then run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch computer_vision_model fruit_image_pipeline.launch.py
```

Or specify files explicitly (JPG/JPEG/PNG):

```bash
ros2 launch computer_vision_model fruit_image_pipeline.launch.py \
  image_path:=/absolute/path/to/photo.png \
  calibration_path:=/absolute/path/to/calibration.yaml
```

Both launches share the same detection, highest-confidence selection, known-plane
projection, MoveIt IK, and logging. Image mode automatically skips the camera
driver, live capture, and camera TF publisher. It uses the **recorded optical-to-base
transform from the YAML**, even if it differs from today's mount defaults.
Roboflow inference still requires internet and `ROBOFLOW_API_KEY`.

Live runs now add the measured/configured TF used for projection to
`captures/latest_capture.yaml` when the TRANSFORM phase succeeds. Copy that file
alongside its image, or replay the original pair directly:

```bash
ros2 launch computer_vision_model fruit_image_pipeline.launch.py \
  image_path:=/absolute/path/to/logs/RUN/captures/latest_rgb.jpg
```

Calibration is found beside the image as `<image-stem>.yaml`,
`camera_calibration.yaml`, or `latest_capture.yaml` (in that order). Missing
calibration, invalid transforms, and dimensions that do not match the image are
logged as errors before inference. The source files are left intact.

Use an original camera image without resizing/cropping and with calibration for
that camera pose and lens settings. A random photo can contain detectable fruit,
but it does not provide metric coordinates in the robot's workspace without
this calibration. See [the schema and setup notes](input_images/README.md).

## Existing RGB-D workflow

### Output files

Every successful capture atomically replaces these files:

- `latest_rgb.jpg`: normal color image; this is the future CV/API input.
- `latest_depth.png`: aligned 16-bit depth image stored in millimeters.
- `latest_depth_preview.jpg`: colorized depth for human inspection only.
- `latest_capture.yaml`: timestamps, frames, encodings, camera intrinsics, and
  a diagnostic scene-center depth/3D point.
- `latest_detections.json`: labels, confidence scores, center coordinates, and
  bounding-box corners returned by Roboflow.

The diagnostic point is not a fruit location. The separate
`fruit_localization.launch.py` workflow uses depth to estimate fruit XYZ; the
new `fruit_plane_pipeline.launch.py` workflow uses the known plane instead.

## Build

```bash
cd /home/jamal/FYP_ws2
source /opt/ros/humble/setup.bash
colcon build --packages-select computer_vision_model --symlink-install
source install/setup.bash
```

Install the Roboflow HTTP client in the same Python environment used by ROS:

```bash
python3 -m pip install --user inference-sdk
```

## Test without a camera

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch computer_vision_model synthetic_capture.launch.py
```

The first synthetic RGB-D pair is captured automatically. Capture another pair:

```bash
ros2 service call /computer_vision/capture_latest std_srvs/srv/Trigger '{}'
```

Default output directory:

```text
/tmp/computer_vision_captures
```

## Astra Pro driver prerequisite

The original Astra Pro is an OpenNI device with a UVC color camera. Install
Orbbec's legacy `ros2_astra_camera` repository so that ROS provides the
`astra_camera` package and its dedicated `astra_pro.launch.xml` file. The modern
Orbbec SDK v2 branch is not the correct default for this legacy model.

After installing the driver and its udev rules, connect the camera and verify:

```bash
lsusb | grep -i -E 'orbbec|2bc5'
ros2 pkg prefix astra_camera
```

Then launch the real camera and capture node together:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch computer_vision_model astra_pro_capture.launch.py
```

To save somewhere else:

```bash
ros2 launch computer_vision_model astra_pro_capture.launch.py \
  output_directory:=$PWD/captures
```

The launch enables RGB, depth, depth-to-color registration, and timestamp
synchronization. The capture node rejects mismatched RGB/depth dimensions so a
misconfigured, unaligned stream cannot silently produce incorrect fruit depth.

## Driver topics expected by this package

```text
/camera/color/image_raw
/camera/depth/image_raw
/camera/color/camera_info
```

Topic names are parameters and can be changed if a driver installation uses a
different naming convention.

## Capture and detect once on startup

Stop any separately running Astra driver first because this launch starts the
driver itself. Supply the API key through the environment; it is intentionally
not accepted as a ROS parameter or stored in this repository.

```bash
cd /home/jamal/FYP_ws2
source /opt/ros/humble/setup.bash
source install/setup.bash
read -rsp "Roboflow API key: " ROBOFLOW_API_KEY
echo
export ROBOFLOW_API_KEY
ros2 launch computer_vision_model fruit_detection.launch.py
```

The default model is `sliced-fruits-and-vegetables-rnw8f/1`. On startup, the
detection node requests one synchronized capture, sends `latest_rgb.jpg` to the
Roboflow hosted inference API, prints a summary grouped by label, saves
`latest_detections.json`, and publishes the complete JSON result on:

```text
/computer_vision/detections
```

The last result uses transient-local durability, so a confirmation node that
starts later can still receive it. Inspect the result with compatible QoS:

```bash
ros2 topic echo --qos-durability transient_local \
  /computer_vision/detections std_msgs/msg/String
```

Run another capture and inference request at any time:

```bash
ros2 service call /computer_vision/run_detection std_srvs/srv/Trigger '{}'
```

Progress and errors are published on
`/computer_vision/detection_status`. To start the nodes without automatically
calling the model, use `run_on_start:=false`.

## Detection and depth boundary

Only `latest_rgb.jpg` should be sent to the object-detection API. Retain
`latest_depth.png` and `latest_capture.yaml` locally. The API response's bounding
box coordinates must refer to the original image dimensions; those pixels will
then be applied to the matching depth image to calculate fruit depth and XYZ.
Depth-based fruit localization is implemented in `fruit_localization_node.py`.
The plane pipeline described above is the RGB-only alternative.
