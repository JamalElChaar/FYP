# Fixed RGB-D camera setup

## Camera appearance

The RViz/Gazebo camera visual is an Astra Pro shaped approximation: a
165 x 30 x 40 mm black housing with three front optical elements. Its body
dimensions follow the Orbbec Astra Pro datasheet. Lens spacing and decorative
details are illustrative, not an OEM CAD model. The collision envelope uses
the full housing size.

## Placement

Mount the Astra Pro on a rigid stand beside the arm. The example pose in the
robot description is measured from `base_link`: 0.22 m in +X, 0.30 m in -Y,
and 0.50 m up. Its optical axis points toward +Y and 50 degrees downward.
This is a starting layout for a nearby tabletop, not a calibrated measurement.
The camera should see the full reachable work area and the gripper without
being struck by the arm. A rigid external stand is preferable to a wrist camera
for the first version because the camera-to-base transform stays fixed.

Adjust `camera_x`, `camera_y`, `camera_z` (metres), and `camera_pitch`
(radians) to the **measured optical mount pose**. Keep the mount physically
fixed between calibration and operation.

## Simulation

Build and start the robot with the camera enabled:

```bash
cd /home/jamal/FYP_ws2
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_arm_description robot_arm_gazebo --symlink-install
source install/setup.bash
ros2 launch robot_arm_gazebo robot_arm.gazebo.launch.py use_camera:=true
```

The launch publishes the fixed `base_link -> camera_link` transform and
simulated color/depth optical frames. Gazebo publishes aligned RGB-D images at
`/camera/color/image_raw` and `/camera/depth/image_raw`, with intrinsics at
`/camera/color/camera_info`. These match the vision package's capture topics.
The Gazebo world is currently empty, so add target objects to the world before
testing detections.

Check the mount and data in separate terminals:

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
ros2 topic echo /camera/color/camera_info --once
ros2 topic hz /camera/color/image_raw
```

Then start the existing capture node (with the simulation launch still running):

```bash
ros2 run computer_vision_model rgbd_capture_node --ros-args \
  -p use_sim_time:=true \
  -p rgb_topic:=/camera/color/image_raw \
  -p depth_topic:=/camera/depth/image_raw \
  -p camera_info_topic:=/camera/color/camera_info
```

## Physical camera and coordinates

The real Astra driver publishes `camera_link` and its own calibrated
`camera_color_optical_frame` and `camera_depth_optical_frame` transforms.
Run robot_state_publisher with `use_camera:=true use_gazebo:=false` in the
hardware setup, using the measured mount arguments. Do not also publish the
driver's optical transforms from another source. The existing
`astra_pro_capture.launch.py` supplies the camera data.

For a detection at color pixel `(u,v)`, sample a robust depth `z` from
the registered depth image. With the matching color `CameraInfo`, compute
`x=(u-cx)*z/fx`, `y=(v-cy)*z/fy`, `z=z` in the **color optical frame**.
Transform that stamped 3D point through TF to `base_link` before giving it
to MoveIt. Add the grasp approach offset and tool orientation there, then
check reachability and collisions in simulation before commanding hardware.
A 2D bounding box by itself cannot give the arm a metric target.

Calibrate the physical camera intrinsics and the fixed camera-to-base
transform using a calibration target at several positions in the work area.
Do not assume the example X/Y/Z/pitch values are accurate enough for grasping.
Compare known test points transformed into `base_link` with measured
positions before enabling motion.
