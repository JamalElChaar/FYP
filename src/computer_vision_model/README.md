# computer_vision_model

This package captures synchronized Orbbec Astra Pro RGB-D data and can send the
captured JPG to a Roboflow object-detection model.

```text
astra_camera -> rgbd_capture_node -> latest_rgb.jpg -> fruit_detection_node
                                      matching depth    -> detection JSON topic
```

## Output files

Every successful capture atomically replaces these files:

- `latest_rgb.jpg`: normal color image; this is the future CV/API input.
- `latest_depth.png`: aligned 16-bit depth image stored in millimeters.
- `latest_depth_preview.jpg`: colorized depth for human inspection only.
- `latest_capture.yaml`: timestamps, frames, encodings, camera intrinsics, and
  a diagnostic scene-center depth/3D point.
- `latest_detections.json`: labels, confidence scores, center coordinates, and
  bounding-box corners returned by Roboflow.

The diagnostic point is not a fruit location. Once the detector returns a fruit
bounding box or mask, the matching raw depth pixels will be sampled to compute
the fruit's median depth and camera-frame XYZ.

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
That fruit-depth association and the user confirmation service are intentionally
the next integration step; the current node publishes everything they need.
