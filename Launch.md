# MoveIt 2 to ESP32: real-hardware launch guide

This guide brings up the complete real-hardware path:

```text
MoveIt target
  -> MoveIt trajectory (joint positions in radians)
  -> arm_controller / FollowJointTrajectory
  -> CustomHardwareInterface
  -> servo angles in degrees on /esp32/joint_commands
  -> micro-ROS Agent over Wi-Fi/UDP
  -> ESP32 PWM outputs
  -> servos
```

The ESP32 publishes `/esp32/joint_states` in the opposite direction. The hardware
interface converts those servo degrees back to ROS joint radians, and
`joint_state_broadcaster` publishes `/joint_states` for MoveIt.

## Important safety and current limitations

Read this before applying servo power.

- Use a separate, regulated 5-6 V servo supply sized for the combined current of
  all servos. Never power the servos from the ESP32 3.3 V/5 V pin or the USB port.
- Connect the external supply ground and ESP32 ground together. The PWM signal
  needs this common ground.
- Use an accessible switch or emergency stop that physically disconnects the
  servo power rail. `Ctrl+C` is not a hardware emergency stop.
- Support the arm, remove the load, and keep people clear during first motion.
- The firmware commands every enabled servo to 90 degrees during boot. Centre
  each servo and fit its horn at the robot's mechanical zero before testing.
- The current ESP32 firmware has **joint 1 disabled** with
  `SERVO_PINS[0] = -1`. Set the correct GPIO before expecting joint 1 to move.
- The ESP32 does not read encoders. `/esp32/joint_states` echoes the most recent
  commanded servo angles; it does not prove that a joint physically reached them.
- The host hardware interface currently assumes all six servo offsets are 90
  degrees and all directions are `+1`. Calibrate these values before larger moves.
- Do the first tests through `moveit_hardware_node`. It rejects requested joint
  targets outside +/-90 degrees, checks every point of a Cartesian plan against
  that range, uses 10% velocity/acceleration, shows final servo angles, and asks
  for confirmation. Direct RViz execution bypasses these extra checks.
- Do **not** run `test_motion_node` on the real arm. It contains an automatic
  Cartesian test at a higher speed and does not ask for confirmation.

## 1. Build and start the project container on the external PC

The external PC only needs Ubuntu, Docker, the repository, Wi-Fi access to the
ESP32, a working display for RViz, and Arduino IDE for flashing the ESP32. The
project image already contains ROS 2 Humble Desktop, MoveIt 2, `ros2_control`, the
controllers, RViz, Xacro, and the normal build tools. Run ROS commands inside this
container rather than installing the project dependencies directly on the host
PC.

### First time only: build the image

In a **host PC terminal**, change to the repository root and build the image:

```bash
cd /path/to/FYP_ws2
docker build -t fyp_robot_arm:latest -f docker/dockerfile .
```

The image build downloads the main dependencies and can take a while. It only
needs to be repeated after changing the Dockerfile or its installation scripts.

### First time only: create the named container

The start script uses host networking, passes through the display, mounts the
repository at `/home/user/ros2_ws`, and names the container `fyp_robot_arm`.
Choose the ROS domain before creating it:

```bash
cd /path/to/FYP_ws2
export ROS_DOMAIN_ID=0
./docker/start_container.sh
```

The prompt should begin with `[humble-fyp]`. The script stores the selected domain
in the container. Do not leave `ROS_DOMAIN_ID` unset: the script otherwise defaults
to domain `1`, while the ESP32 firmware currently defaults to domain `0`.

If Docker requires elevated permissions, configure the host user for Docker or
use `sudo` consistently with the Docker commands.

### First time inside the container: build the project workspace

Run these commands at the `[humble-fyp]` prompt:

```bash
cd /home/user/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

The repository is a host bind mount, so its source, build, install, and log files
remain on the external PC. Re-run `colcon build --symlink-install` after changing
host-side C++, launch files, servo calibration, or MoveIt configuration.

### First time inside the container: download and build micro-ROS Agent

The image does not currently include `micro_ros_agent`. Build it once in a
separate workspace stored in the named container:

```bash
source /opt/ros/humble/setup.bash

mkdir -p /home/user/microros_ws/src
cd /home/user/microros_ws
git clone -b humble \
  https://github.com/micro-ROS/micro_ros_setup.git \
  src/micro_ros_setup

sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/local_setup.bash

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash

ros2 pkg prefix micro_ros_agent
```

The final command must print a path under `/home/user/microros_ws/install`.
Configure future interactive container terminals to source this workspace:

```bash
grep -qxF \
  'source /home/user/microros_ws/install/local_setup.bash' \
  /home/user/.bashrc || \
  echo 'source /home/user/microros_ws/install/local_setup.bash' \
  >> /home/user/.bashrc
```

This Agent download/build is only required once for this named container.
Official Agent build instructions:
<https://github.com/micro-ROS/micro_ros_setup#building-micro-ros-agent>

### Exit and reopen the same container later

Typing `exit` at the `[humble-fyp]` prompt stops the container but does not delete
it:

```bash
exit
```

On the next day or after a reboot, reopen the **same** container from a host PC
terminal:

```bash
xhost +local:docker
docker start -ai fyp_robot_arm
```

This preserves `/home/user/microros_ws`, so the Agent and its downloaded packages
do not need to be rebuilt. Do **not** run `./docker/start_container.sh` again just
to reopen the environment: that script removes any existing `fyp_robot_arm`
container and creates a fresh one, which would discard the Agent workspace stored
inside it.

To open extra terminals while the container is already running, use this command
from each new host PC terminal:

```bash
docker exec -it fyp_robot_arm /bin/bash
```

The container's `.bashrc` sources ROS, the project workspace when built, and the
micro-ROS workspace configured above. If a package is unexpectedly missing, source
them explicitly in this order:

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
source /home/user/microros_ws/install/local_setup.bash
```

## 2. Prepare and flash the ESP32

Use this firmware, not the direct-PWM or TCP firmware:

```text
src/custom_hardware/esp32_firmware/esp32_microros_servo/esp32_microros_servo.ino
```

In Arduino IDE:

1. Install the ESP32 board support package and select the correct ESP32 board and
   serial port. The micro-ROS Arduino project lists ESP32 Dev Module support with
   ESP32 Arduino core 2.0.2.
2. Download the Humble branch/release of `micro_ros_arduino` and add it with
   **Sketch -> Include Library -> Add .ZIP Library**.
3. Install `ESP32Servo` from the Arduino Library Manager.
4. Open `esp32_microros_servo.ino`.
5. Edit `WIFI_SSID` and `WIFI_PASSWORD`.
6. Find the PC's Wi-Fi address with `hostname -I`. Choose the address on the same
   subnet as the ESP32, then set `agent_ip` to that address.
7. Keep `agent_port = 8888`, unless the same new port is used everywhere below.
8. Set `ROS_DOMAIN_ID` to the same number used by every ROS terminal. This guide
   uses domain `0`.
9. Replace every required entry in `SERVO_PINS`. A value of `-1` intentionally
   disables that joint. Verify that each selected GPIO is safe for the exact ESP32
   board; some ESP32 pins affect boot mode.
10. Upload the sketch. Use the 115200-baud Serial Monitor to read the ESP32 IP,
    configured Agent IP, and connection status.

Official Arduino library: <https://github.com/micro-ROS/micro_ros_arduino>

### Wiring

For each servo:

```text
servo signal       -> configured ESP32 GPIO
servo positive     -> external regulated 5-6 V positive
servo ground       -> external supply ground
ESP32 ground       -> external supply ground (common ground)
ESP32 USB           -> PC/USB supply for logic and flashing
```

Leave the servo power rail switched off for the communication and dry-run checks.

## 3. Make the ROS domain consistent

Unless explicitly labelled as a host PC command, run every remaining command in
the project container. Confirm the domain in each container terminal:

```bash
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
source /home/user/microros_ws/install/local_setup.bash
```

The ESP32 firmware, micro-ROS Agent, and all ROS nodes must use the same domain.
The number itself is unimportant; it only needs to match. Because the container
uses host networking, the Agent inside it can listen directly on the external
PC's Wi-Fi address and UDP port 8888.

## 4. Dry-run the complete MoveIt stack without the ESP32

Do this after every control or MoveIt change and before enabling real hardware.

Terminal A:

```bash
cd /home/user/ros2_ws
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch control_arm moveit_hardware.launch.py test_mode:=true
```

This starts MoveIt, RViz, `ros2_control`, the position trajectory controller, and
the custom interface in simulated-state mode. It does not start the micro-ROS
Agent.

Terminal B:

```bash
cd /home/user/ros2_ws
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 control list_controllers
ros2 launch control_arm plan_hardware.launch.py
```

`arm_controller` and `joint_state_broadcaster` must both show `active`. In the
interactive menu, select `2`, then preset `1` (`joint1 +20deg`), inspect the plan,
and answer `y`. Return to home with menu option `1`. Confirm the RViz model follows
the motion.

Stop both dry-run terminals with `Ctrl+C` before starting the real stack.

## 5. Start the micro-ROS Agent

Run exactly one Agent. Start it before resetting or powering the ESP32 because the
current firmware does not implement automatic Agent reconnection.

Open an additional terminal in the already-running project container. From a host
PC terminal:

```bash
docker exec -it fyp_robot_arm /bin/bash
```

Then, inside that new container terminal:

```bash
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source /home/user/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

Keep this terminal running. The later MoveIt launch must use
`start_agent:=false` because the Agent is already active. Running it separately
makes connection and transport errors visible.

Alternatively, skip the manual Agent command and let
`moveit_hardware.launch.py` start the Agent by leaving `start_agent` at its
default value of `true`. Do not use both methods simultaneously.

## 6. Connect and verify the ESP32 with servo power off

1. Confirm the Agent is listening on UDP port 8888.
2. Keep the servo power rail off.
3. Power or reset the ESP32. Its Serial Monitor should report Wi-Fi connected and
   the correct Agent address.
4. In a ROS terminal, run:

```bash
cd /home/user/ros2_ws
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 node list | grep esp32_servo_node
ros2 topic info /esp32/joint_states --verbose
ros2 topic echo /esp32/joint_states --once
ros2 topic hz /esp32/joint_states
```

Expected results:

- `/esp32_servo_node` is listed.
- `/esp32/joint_states` has one publisher.
- The message contains six values, initially near
  `[90, 90, 90, 90, 90, 90]`.
- The state topic is close to 50 Hz.

Do not continue if these checks fail. The host interface can temporarily simulate
state before receiving an ESP32 message, so MoveIt appearing healthy by itself is
not proof of an ESP32 connection.

## 7. Optional low-level calibration test

This step bypasses MoveIt and is only for pin, centre, and direction calibration.
Do not run the MoveIt hardware launch at the same time, because that would create
two command publishers.

1. Remove servo horns or support the joint so a small move cannot damage the arm.
2. Switch on the external servo supply.
3. Centre all enabled servos:

```bash
ros2 topic pub --once /esp32/joint_commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]}"
```

4. Test one enabled servo with a five-degree change. This example tests joint 2:

```bash
ros2 topic pub --once /esp32/joint_commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [90.0, 95.0, 90.0, 90.0, 90.0, 90.0]}"

ros2 topic pub --once /esp32/joint_commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]}"
```

5. Repeat one joint at a time. If positive ROS motion must turn a servo the other
   way, add a per-joint override after the default-configuration loop in
   `src/custom_hardware/src/custom_hardware.cpp`, for example
   `joint_directions_[1] = -1.0;` for joint 2. Set the matching
   `joint_offsets_[1]` override if mechanical zero is not 90 degrees. Rebuild,
   re-source, and keep limits conservative until the physical limits are known.
6. Switch servo power off when calibration is complete.

The low-level messages above are already servo degrees. Normal MoveIt commands
are radians and must not be published directly to `/esp32/joint_commands`.

## 8. Launch MoveIt with the real hardware interface

Keep the Agent and ESP32 running, with servo power still off initially.

Terminal A (MoveIt, RViz, controllers, and hardware interface):

```bash
cd /home/user/ros2_ws
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source install/setup.bash
source /home/user/microros_ws/install/local_setup.bash

ros2 launch control_arm moveit_hardware.launch.py \
  start_agent:=false \
  agent_port:=8888
```

If the MoveIt launch should start the Agent itself, omit `start_agent:=false` and
do not run the separate Agent terminal from section 5.

Terminal B (checks):

```bash
cd /home/user/ros2_ws
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 control list_controllers
ros2 action list | grep arm_controller
ros2 topic hz /esp32/joint_states
ros2 topic echo /joint_states --once
```

Required before motor power is enabled:

- `arm_controller` is `active`.
- `joint_state_broadcaster` is `active`.
- `/arm_controller/follow_joint_trajectory` is listed.
- `/esp32/joint_states` is still arriving near 50 Hz.
- `/joint_states` contains all six joints near zero radians when the ESP32 reports
  90-degree servo centres.

For visibility into the exact servo-degree commands, use another terminal:

```bash
ros2 topic echo /esp32/joint_commands
```

This topic only publishes when a command changes, so it may remain quiet until a
trajectory starts.

## 9. First MoveIt motion on hardware

1. Put the arm in its calibrated zero configuration and support it.
2. Make sure the emergency power cutoff is reachable.
3. Enable the external servo power.
4. Start the safe interactive node in Terminal C:

```bash
cd /home/user/ros2_ws
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch control_arm plan_hardware.launch.py
```

5. Select menu option `5` and confirm all current joints are close to 0 rad / 0
   degrees. If the displayed state does not match the physical pose, stop and fix
   calibration before moving.
6. Select option `3` and enter a single small move on an enabled joint. For a
   five-degree joint-2 test, enter:

```text
0 5 0 0 0 0
```

7. Inspect the printed plan. Its final servo values should be approximately:

```text
90 95 90 90 90 90
```

8. Only then enter `y` to execute. Be ready to cut servo power.
9. Return to home with option `1` and confirm execution.

### Test a hardcoded safe pose

The interactive node already contains hardcoded joint-space presets in
`MoveItHardwareNode::initSafePoses()`. They are passed to MoveIt, which creates a
time-parameterized joint trajectory; the controller then streams the intermediate
joint angles to the ESP32.

After every joint is calibrated, select option `2`, then preset `2` (`tilt fwd`):

```text
ROS joints:   [0, +15, -15, 0, 0, 0] degrees
Servo output: [90, 105, 75, 90, 90, 90] degrees
```

Inspect the plan and execute only if the physical arm has clearance.

### Test MoveIt inverse kinematics from a Cartesian pose

For a Cartesian target, MoveIt—not the ESP32—calculates the joint angles:

1. Select option `5` and record the current end-effector position and quaternion.
2. Select option `4`.
3. Enter a target only a few millimetres from the current position, for example
   add `0.005` m to one coordinate while keeping the others unchanged.
4. Enter the current quaternion in the menu's requested order: **w x y z**.
5. MoveIt solves IK, plans a trajectory, checks that all planned joints remain
   within +/-90 degrees, and prints the final ROS and servo angles.
6. Confirm with `y` only after checking the plan and workspace clearance.

There is an older hardcoded Cartesian target in `test_motion_node.cpp`, but that
node automatically executes and is intentionally not used for hardware bring-up.

## 10. RViz planning after commissioning

Once offsets, directions, limits, and every joint have been tested:

1. Open the MotionPlanning panel in RViz.
2. Set a small target with the interactive marker.
3. Click **Plan** first and inspect the full trajectory.
4. Use **Execute** only after the robot has been commissioned.

RViz execution sends the MoveIt trajectory through the same `arm_controller` and
hardware interface, but it does not use the interactive node's terminal
confirmation or its additional +/-90-degree trajectory check. The URDF currently
declares continuous joints, while the servo hardware is limited, so do not use
large RViz motions until realistic joint position limits are configured.

## 11. Shutdown order

1. Use the interactive node to return to the known home pose if it is safe.
2. Stop `plan_hardware.launch.py` with `Ctrl+C`.
3. Switch off the external servo power rail.
4. Stop `moveit_hardware.launch.py` with `Ctrl+C`.
5. Power down or disconnect the ESP32.
6. Stop the micro-ROS Agent with `Ctrl+C`.

In an emergency, disconnect servo power first.

## Testing

### Build

```bash
cd /home/jamal/FYP_ws2
source /opt/ros/humble/setup.bash
colcon build --packages-select control_arm
source install/setup.bash
```

### Terminal 1: micro-ROS Agent

```bash
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

### ESP32

```bash
# Upload with Arduino IDE, then reset the ESP32:
# src/custom_hardware/esp32_firmware/esp32_moveit_direct_tests/esp32_moveit_direct_tests.ino
```

### Terminal 2: final angles, one joint at a time

```bash
cd /home/jamal/FYP_ws2
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch control_arm final_angles_sequential.launch.py
```

### Terminal 2: full MoveIt path at 0.2-second intervals

```bash
cd /home/jamal/FYP_ws2
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch control_arm slow_moveit_trajectory.launch.py
```

## Troubleshooting

### `Package 'micro_ros_agent' not found`

The one-time Agent build in section 1 was not completed or its workspace is not
sourced. Inside the container, run:

```bash
source /opt/ros/humble/setup.bash
source /home/user/microros_ws/install/local_setup.bash
ros2 pkg prefix micro_ros_agent
```

If the install file is absent, repeat the one-time Agent build. If the entire
`/home/user/microros_ws` directory is absent, check whether the named container
was accidentally recreated instead of reopened with `docker start -ai`.

### ESP32 node or topics do not appear

- Start the Agent first, then reset the ESP32.
- Verify `agent_ip`, UDP port 8888, Wi-Fi subnet, and `ROS_DOMAIN_ID`.
- Check that a firewall is not blocking UDP port 8888.
- Read the 115200-baud Serial Monitor. A fast-blinking LED indicates the
  firmware entered its error loop.
- Make sure only one Agent is using port 8888.

### MoveIt runs but the arm receives no commands

```bash
ros2 control list_controllers
ros2 action list | grep follow_joint_trajectory
ros2 topic info /esp32/joint_commands --verbose
ros2 topic echo /esp32/joint_commands
```

Rebuild and source the workspace if an installed launch file or executable is
stale.

### Joint 1 does not move

The shipped firmware disables it. Replace `SERVO_PINS[0] = -1` with the verified
GPIO, flash again, and reset the ESP32 after the Agent is running.

### Servo moves the wrong way or does not match RViz

Cut servo power. Correct that joint's offset/direction in
`custom_hardware.cpp`, rebuild, re-source, and repeat the five-degree test. Do not
compensate by swapping random pose signs; the calibrated mapping must be correct
for every future MoveIt plan.

### Servos jump when the ESP32 boots

The firmware writes 90 degrees immediately. Remove the horn/load, centre the
servo, then install the horn at the model's zero pose. If the robot cannot safely
use a 90-degree startup, change the startup/calibration design before operating
the assembled arm.

### MoveIt reports success even though a motor did not move

This is expected with the current open-loop feedback design: the ESP32 echoes its
commanded angle. Add real joint sensing and publish measured positions before
treating trajectory success as proof of physical motion.
