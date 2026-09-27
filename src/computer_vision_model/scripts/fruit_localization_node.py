#!/usr/bin/env python3
"""Turn a Roboflow fruit bounding box into a base_link pose for the arm.

Pipeline:
    /computer_vision/detections (bounding boxes, pixels)
      + latest_depth.png (16-bit millimeters)
      + latest_capture.yaml (fx, fy, cx, cy, frame ids)
      -> median depth inside the box
      -> back-projected XYZ in the camera optical frame
      -> TF transform into base_link
      -> /computer_vision/fruit_pose (PoseStamped, with standoff applied)

The camera extrinsic is NOT known by this node. It comes from TF, so either
publish a measured static transform (see fruit_localization.launch.py) or add
the camera link to the real-hardware URDF once it is mounted on the arm.
"""

import json
import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

import tf2_ros
from tf2_geometry_msgs import do_transform_point


class FruitLocalizationNode(Node):
    """Locate a detected fruit in the robot's base frame."""

    def __init__(self):
        super().__init__("fruit_localization_node")

        self.detections_topic = self.declare_parameter(
            "detections_topic", "/computer_vision/detections"
        ).value
        self.pose_topic = self.declare_parameter(
            "pose_topic", "/computer_vision/fruit_pose"
        ).value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value

        # Which detection to act on.
        self.target_label = self.declare_parameter("target_label", "").value
        self.selection = self.declare_parameter(
            "selection", "highest_confidence"
        ).value
        self.minimum_confidence = float(self.declare_parameter(
            "minimum_confidence", 0.40
        ).value)

        # Depth sampling inside the bounding box.
        self.depth_sample_fraction = float(self.declare_parameter(
            "depth_sample_fraction", 0.5
        ).value)
        self.minimum_valid_depth_pixels = int(self.declare_parameter(
            "minimum_valid_depth_pixels", 20
        ).value)
        self.minimum_depth_m = float(self.declare_parameter(
            "minimum_depth_m", 0.15
        ).value)
        self.maximum_depth_m = float(self.declare_parameter(
            "maximum_depth_m", 2.0
        ).value)

        # Approach: lift the commanded target above the fruit, matching the
        # pre_final_offset idea already used by direct_esp32_moveit_node.
        self.standoff_m = float(self.declare_parameter("standoff_m", 0.03).value)

        # Orientation of the commanded pose (the fork approach attitude).
        self.orientation = [
            float(self.declare_parameter("ox", -0.548).value),
            float(self.declare_parameter("oy", 0.447).value),
            float(self.declare_parameter("oz", 0.006).value),
            float(self.declare_parameter("ow", 0.707).value),
        ]

        # Reject anything outside the reachable box, in base_link metres.
        self.workspace_min = [
            float(self.declare_parameter("workspace_min_x", -0.40).value),
            float(self.declare_parameter("workspace_min_y", -0.40).value),
            float(self.declare_parameter("workspace_min_z", 0.0).value),
        ]
        self.workspace_max = [
            float(self.declare_parameter("workspace_max_x", 0.40).value),
            float(self.declare_parameter("workspace_max_y", 0.40).value),
            float(self.declare_parameter("workspace_max_z", 0.50).value),
        ]

        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = ReliabilityPolicy.RELIABLE

        self.pose_publisher = self.create_publisher(
            PoseStamped, self.pose_topic, latched
        )
        self.summary_publisher = self.create_publisher(
            String, "/computer_vision/fruit_localization", latched
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            String, self.detections_topic, self._handle_detections, latched
        )

        self.get_logger().info("Fruit localization node ready")
        self.get_logger().info(f"  Detections in : {self.detections_topic}")
        self.get_logger().info(f"  Pose out      : {self.pose_topic}")
        self.get_logger().info(f"  Base frame    : {self.base_frame}")
        self.get_logger().info(f"  Standoff      : {self.standoff_m:.3f} m above fruit")

    # ── helpers ──────────────────────────────────────────────────────────

    def _publish_summary(self, status, detail="", payload=None):
        message = {"status": status, "detail": detail}
        if payload:
            message.update(payload)
        self.summary_publisher.publish(String(data=json.dumps(message)))

    def _fail(self, detail):
        self.get_logger().error(detail)
        self._publish_summary("error", detail)

    def _select_detection(self, detections):
        candidates = [
            detection for detection in detections
            if detection.get("confidence", 0.0) >= self.minimum_confidence
        ]
        if self.target_label:
            wanted = self.target_label.strip().lower()
            candidates = [
                detection for detection in candidates
                if str(detection.get("label", "")).lower() == wanted
            ]
        if not candidates:
            return None

        if self.selection == "largest":
            return max(
                candidates,
                key=lambda d: float(d["width"]) * float(d["height"]),
            )
        return max(candidates, key=lambda d: float(d["confidence"]))

    def _median_box_depth(self, depth_m, detection):
        """Median of valid depth pixels in the middle of the bounding box."""
        height, width = depth_m.shape[:2]

        centre_x = float(detection["center_x"])
        centre_y = float(detection["center_y"])
        half_w = float(detection["width"]) * self.depth_sample_fraction / 2.0
        half_h = float(detection["height"]) * self.depth_sample_fraction / 2.0

        x1 = max(0, int(math.floor(centre_x - half_w)))
        y1 = max(0, int(math.floor(centre_y - half_h)))
        x2 = min(width, int(math.ceil(centre_x + half_w)) + 1)
        y2 = min(height, int(math.ceil(centre_y + half_h)) + 1)
        if x2 <= x1 or y2 <= y1:
            return None, 0

        patch = depth_m[y1:y2, x1:x2]
        valid = patch[np.isfinite(patch) & (patch >= self.minimum_depth_m)
                      & (patch <= self.maximum_depth_m)]
        if valid.size == 0:
            return None, 0
        return float(np.median(valid)), int(valid.size)

    def _load_capture(self, image_path):
        directory = Path(image_path).expanduser().resolve().parent
        depth_path = directory / "latest_depth.png"
        metadata_path = directory / "latest_capture.yaml"

        if not depth_path.is_file():
            raise FileNotFoundError(f"Missing depth image: {depth_path}")
        if not metadata_path.is_file():
            raise FileNotFoundError(f"Missing capture metadata: {metadata_path}")

        raw_depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        if raw_depth is None:
            raise RuntimeError(f"Could not read {depth_path}")
        # rgbd_capture_node stores 16-bit millimetres; 0 means "no reading".
        depth_m = raw_depth.astype(np.float32) / 1000.0
        depth_m[raw_depth == 0] = np.nan

        with open(metadata_path, "r", encoding="utf-8") as handle:
            metadata = yaml.safe_load(handle)

        intrinsics = metadata.get("camera_intrinsics", {})
        fx = float(intrinsics.get("fx", 0.0))
        fy = float(intrinsics.get("fy", 0.0))
        cx = float(intrinsics.get("cx", 0.0))
        cy = float(intrinsics.get("cy", 0.0))
        if fx <= 0.0 or fy <= 0.0:
            raise ValueError(
                f"Invalid camera intrinsics in {metadata_path}: fx={fx}, fy={fy}"
            )

        camera_frame = str(
            metadata.get("capture", {}).get("rgb_frame_id", "")
        ).strip()
        if not camera_frame:
            raise ValueError(f"No rgb_frame_id recorded in {metadata_path}")

        return depth_m, (fx, fy, cx, cy), camera_frame

    def _in_workspace(self, x, y, z):
        return (
            self.workspace_min[0] <= x <= self.workspace_max[0]
            and self.workspace_min[1] <= y <= self.workspace_max[1]
            and self.workspace_min[2] <= z <= self.workspace_max[2]
        )

    # ── main callback ────────────────────────────────────────────────────

    def _handle_detections(self, message):
        try:
            result = json.loads(message.data)
        except json.JSONDecodeError as error:
            self._fail(f"Detections payload is not valid JSON: {error}")
            return

        detections = result.get("detections", [])
        if not detections:
            self._fail("Detection result contained no objects")
            return

        detection = self._select_detection(detections)
        if detection is None:
            self._fail(
                "No detection passed the confidence/label filter "
                f"(minimum_confidence={self.minimum_confidence}, "
                f"target_label='{self.target_label}')"
            )
            return

        try:
            depth_m, (fx, fy, cx, cy), camera_frame = self._load_capture(
                result["image_path"]
            )
        except Exception as error:
            self._fail(f"Could not load the matching capture: {error}")
            return

        # The bounding box must refer to the same image the depth belongs to.
        image_size = result.get("image", {})
        expected_w = int(image_size.get("width", 0))
        expected_h = int(image_size.get("height", 0))
        depth_h, depth_w = depth_m.shape[:2]
        if expected_w and expected_h and (expected_w, expected_h) != (depth_w, depth_h):
            self._fail(
                f"Detection image is {expected_w}x{expected_h} but depth is "
                f"{depth_w}x{depth_h}; bounding boxes cannot be applied to depth"
            )
            return

        depth, valid_pixels = self._median_box_depth(depth_m, detection)
        if depth is None or valid_pixels < self.minimum_valid_depth_pixels:
            self._fail(
                f"Only {valid_pixels} valid depth pixels inside the "
                f"'{detection.get('label')}' box (need "
                f"{self.minimum_valid_depth_pixels}); check that the Astra Pro "
                "depth stream and depth_registration are working"
            )
            return

        # Back-project the box centre into the camera optical frame.
        u = float(detection["center_x"])
        v = float(detection["center_y"])
        point = PointStamped()
        point.header.frame_id = camera_frame
        point.header.stamp = rclpy.time.Time().to_msg()  # latest available
        point.point.x = (u - cx) * depth / fx
        point.point.y = (v - cy) * depth / fy
        point.point.z = depth

        self.get_logger().info(
            f"'{detection['label']}' (confidence {detection['confidence']:.2f}) "
            f"at pixel ({u:.0f}, {v:.0f}), median depth {depth:.3f} m "
            f"from {valid_pixels} pixels"
        )
        self.get_logger().info(
            f"  {camera_frame}: x={point.point.x:+.4f} "
            f"y={point.point.y:+.4f} z={point.point.z:+.4f}"
        )

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
        except tf2_ros.TransformException as error:
            self._fail(
                f"No transform from '{camera_frame}' to '{self.base_frame}': "
                f"{error}. Publish the measured camera extrinsic (see "
                "fruit_localization.launch.py) before driving the arm."
            )
            return

        fruit = do_transform_point(point, transform)
        fx_base = fruit.point.x
        fy_base = fruit.point.y
        fz_base = fruit.point.z

        self.get_logger().info(
            f"  {self.base_frame}: x={fx_base:+.4f} "
            f"y={fy_base:+.4f} z={fz_base:+.4f}"
        )

        target_z = fz_base + self.standoff_m
        if not self._in_workspace(fx_base, fy_base, target_z):
            self._fail(
                f"Target ({fx_base:+.3f}, {fy_base:+.3f}, {target_z:+.3f}) is "
                "outside the configured workspace box; refusing to publish it"
            )
            return

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = fx_base
        pose.pose.position.y = fy_base
        pose.pose.position.z = target_z
        pose.pose.orientation.x = self.orientation[0]
        pose.pose.orientation.y = self.orientation[1]
        pose.pose.orientation.z = self.orientation[2]
        pose.pose.orientation.w = self.orientation[3]
        self.pose_publisher.publish(pose)

        self._publish_summary(
            "located",
            f"{detection['label']} located in {self.base_frame}",
            {
                "label": detection["label"],
                "confidence": detection["confidence"],
                "camera_frame": camera_frame,
                "depth_m": depth,
                "valid_depth_pixels": valid_pixels,
                "point_camera": [point.point.x, point.point.y, point.point.z],
                "point_base": [fx_base, fy_base, fz_base],
                "target_base": [fx_base, fy_base, target_z],
            },
        )

        self.get_logger().info(
            f"Published target on {self.pose_topic} "
            f"({self.standoff_m:.3f} m above the fruit)"
        )
        self.get_logger().info(
            "To drive the arm there, inspect the numbers first, then run:\n"
            "  ros2 launch control_arm slow_moveit_trajectory.launch.py "
            f"x:={fx_base:.4f} y:={fy_base:.4f} z:={target_z:.4f} "
            f"ox:={self.orientation[0]} oy:={self.orientation[1]} "
            f"oz:={self.orientation[2]} ow:={self.orientation[3]}"
        )


def main(args=None):
    """Run the fruit localization ROS 2 node."""
    rclpy.init(args=args)
    node = FruitLocalizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
