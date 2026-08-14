# Astra Pro connection test

Check that Linux sees the physical Astra Pro (no ROS driver required):

```bash
cd /home/jamal/FYP_ws2
python3 test_cam/test_camera.py
```

For a complete ROS test, first launch the Astra driver and then run:

```bash
source /opt/ros/humble/setup.bash
source /home/jamal/FYP_ws2/install/setup.bash
python3 test_cam/test_camera.py --ros
```

The ROS test waits for one message on each expected topic:

- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/camera/color/camera_info`

The USB check and ROS check are intentionally separate: USB detection proves
that the laptop can enumerate the hardware, while live topic messages prove that
the camera driver is installed, running, and reading frames.
