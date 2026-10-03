#!/usr/bin/env bash
# Re-create the Astra's video device nodes inside the running fyp_robot_arm
# container, without recreating the container.
#
# The container's /dev was snapshotted when it was created, so device nodes
# that appear afterwards (like the Astra when it is plugged in) are not
# visible inside it. The container runs privileged, so it is permitted to
# reach every host device -- it is only missing the node files. This copies
# the current major/minor numbers from the host and recreates them.
#
# Run this on the HOST after plugging in the camera, or after a reboot.
#
#   ./docker/sync_astra_devices.sh
#
# A permanent fix would be adding "-v /dev:/dev --group-add video" to
# start_container.sh, but that requires recreating the container and would
# discard the micro-ROS workspace and libuvc install stored inside it.

set -euo pipefail

CONTAINER="${CONTAINER:-fyp_robot_arm}"
VENDOR_ID="2bc5"   # Orbbec

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "error: container '$CONTAINER' is not running" >&2
  exit 1
fi

# Recreate one device node inside the container, copying the host's
# major/minor/ownership/mode. -u 0: mknod needs root in the container;
# without it docker exec runs as the default user and is denied.
clone_node() {
  local path="$1"
  local major minor owner mode
  major=$(stat -c '%Hr' "$path")
  minor=$(stat -c '%Lr' "$path")
  owner=$(stat -c '%U:%G' "$path")
  mode=$(stat -c '%a' "$path")
  echo "  $path  ($major:$minor  $owner $mode)"
  docker exec -u 0 "$CONTAINER" bash -c "
    mkdir -p '$(dirname "$path")'
    rm -f '$path'
    mknod '$path' c $major $minor
    chown $owner '$path' 2>/dev/null || true
    chmod $mode '$path'
  "
}

# 1. V4L2 nodes -- the UVC colour stream (use_uvc_camera:=true) reads these.
mapfile -t VIDEO_NODES < <(
  for v in /dev/video*; do
    [ -e "$v" ] || continue
    vid=$(udevadm info --query=property --name="$v" 2>/dev/null |
          sed -n 's/^ID_VENDOR_ID=//p')
    [ "$vid" = "$VENDOR_ID" ] && echo "$v"
  done
)

# 2. Raw USB nodes -- OpenNI2 opens the depth device (2bc5:0403) through
#    /dev/bus/usb directly, and libuvc uses raw USB too. These are the nodes
#    that produce 'Could not open "2bc5/0403@1/57": USB device not found!'
#    when they are missing inside the container.
mapfile -t USB_NODES < <(
  lsusb 2>/dev/null | grep -i "$VENDOR_ID:" | while read -r line; do
    bus=$(echo "$line" | sed -E 's/Bus ([0-9]+) Device ([0-9]+).*/\1/')
    dev=$(echo "$line" | sed -E 's/Bus ([0-9]+) Device ([0-9]+).*/\2/')
    node="/dev/bus/usb/$bus/$dev"
    [ -e "$node" ] && echo "$node"
  done
)

if [ "${#VIDEO_NODES[@]}" -eq 0 ] && [ "${#USB_NODES[@]}" -eq 0 ]; then
  echo "error: no Orbbec ($VENDOR_ID) device found on the host." >&2
  echo "       Plug the Astra in and check it appears in: lsusb | grep $VENDOR_ID" >&2
  exit 1
fi

NODES=("${VIDEO_NODES[@]}")

echo "Astra V4L2 nodes (UVC colour stream): ${#VIDEO_NODES[@]}"
for v in "${VIDEO_NODES[@]}"; do clone_node "$v"; done

echo
echo "Astra raw USB nodes (OpenNI depth + libuvc): ${#USB_NODES[@]}"
for u in "${USB_NODES[@]}"; do clone_node "$u"; done

echo
echo "Verifying from inside the container:"
# -i so the heredoc reaches python3 on stdin. Deliberately NOT -u 0: this
# checks the ordinary container user can open the device, which is what the
# driver will run as.
docker exec -i "$CONTAINER" python3 - "${NODES[@]}" <<'PY'
import fcntl, sys
for dev in sys.argv[1:]:
    try:
        buf = bytearray(104)
        with open(dev, 'rb', buffering=0) as f:
            fcntl.ioctl(f, 0x80685600, buf, True)   # VIDIOC_QUERYCAP
        card = bytes(buf[16:48]).split(b'\0')[0].decode()
        print(f'  {dev}: {card}')
    except Exception as error:
        print(f'  {dev}: FAILED - {type(error).__name__}: {error}')
PY

echo
echo "Done. In the container:"
echo "  ros2 launch astra_camera astra_pro.launch.xml"
