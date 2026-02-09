# SANAD FYP

Instructions to run the project

## Prerequisites

- Install Docker: https://docs.docker.com/get-docker/
- At login screen, select gear icon before logging in and select Ubuntu on Xorg

## Commands to start:

### 1. Clone the github repo:
```sh
git clone https://github.com/JamalElChaar/FYP.git
```
### 2. Build the Docker Image

From the **workspace root** (`FYP_ws2/`):

```sh
./docker/build_image.sh
```

(Ha tekhod ktir wa2t to download)

### 3. Start the Container
Also from the workspace root:
```sh
./docker/start_container.sh
```
mafrud ybayen [humble-fyp] had user aal shmel juwet el shell
### 4. Build the Workspace (inside the container)

```sh
cd /home/user/ros2_ws
colcon build 
source install/setup.bash
```

### 5. Launch the Project

Test to see if launch works:

```sh
ros2 launch robot_arm_movit_config demo.launch.py
```
_________________
hon wa2fo w rakbo el hardware byerjaa kamlo
_________________

### 6. Launch ros2_control for hardware




## Useful Commands

**Open another terminal in the running container:**
Ftah a new terminal then run:

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

to exit the container, juwet el container type:
```sh
exit
```
