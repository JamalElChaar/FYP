# SANAD FYP

Instructions to run the project

## Prerequisites

- Install Docker: https://docs.docker.com/get-docker/
- At login screen, select gear icon before logging in and select Ubuntu on Xorg

## Commands to start:

### 1. Clone the github repo:
```sh
git clone 
```
### 1. Build the Docker Image

From the **workspace root** (`FYP_ws2/`):

```sh
./docker/build_image.sh
```

This installs all ROS 2 dependencies (MoveIt 2, Gazebo, ros2_control, etc.) into the image. It only needs to be run once, or when dependencies change.

### 2. Start the Container

```sh
./docker/start_container.sh
```

This will:
- Mount the workspace into the container at `/home/user/ros2_ws`
- Enable X11 forwarding for GUI apps (RViz, Gazebo)
- Auto-detect NVIDIA GPU and enable passthrough if available
- Drop you into a bash shell inside the container

### 3. Build the Workspace (inside the container)

```sh
cd /home/user/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. Launch the Project

**Gazebo simulation + MoveIt:**
```sh
ros2 launch robot_arm_gazebo robot_arm.gazebo.launch.py
```

**MoveIt only (with mock hardware):**
```sh
ros2 launch robot_arm_movit_config demo.launch.py
```

**Real hardware:**
```sh
ros2 launch custom_hardware robot_arm_hardware.launch.py
```

## Useful Commands

**Open another terminal in the running container:**
```sh
docker exec -it fyp_robot_arm /bin/bash
```

**Stop the container:**
```sh
docker stop fyp_robot_arm
```

**Remove the container:**
```sh
docker rm fyp_robot_arm
```

## File Structure

```
docker/
├── dockerfile           # Main Dockerfile (ROS 2 Humble desktop-full)
├── build_image.sh       # Build script
├── start_container.sh   # Container launch script
├── install_tools.sh     # System tools installation
├── install_rosdeps.sh   # ROS 2 package dependencies
└── README.md            # This file
```

## Notes

- The workspace is **bind-mounted** — edits you make on the host are reflected inside the container in real time, and vice versa.
- The container runs as a non-root `user` (UID 1000) with sudo access.
- VS Code server cache is persisted at `~/.vscode-server-cache` for Remote-Container development.
- `ROS_DOMAIN_ID` defaults to `1` if not set in your host environment.
