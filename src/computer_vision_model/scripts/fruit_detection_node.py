#!/usr/bin/env python3
"""Capture one RGB-D frame and run Roboflow object detection on its JPG."""

import json
import os
from pathlib import Path
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


class FruitDetectionNode(Node):
    """Coordinate snapshot capture and hosted Roboflow inference."""

    def __init__(self):
        super().__init__("fruit_detection_node")

        self.model_id = self.declare_parameter(
            "model_id", "sliced-fruits-and-vegetables-rnw8f/1"
        ).value
        self.api_url = self.declare_parameter(
            "api_url", "https://serverless.roboflow.com"
        ).value
        self.capture_service_name = self.declare_parameter(
            "capture_service", "/computer_vision/capture_latest"
        ).value
        self.detections_topic = self.declare_parameter(
            "detections_topic", "/computer_vision/detections"
        ).value
        self.run_on_start = self.declare_parameter(
            "run_on_start", True
        ).value
        self.startup_delay_seconds = float(self.declare_parameter(
            "startup_delay_seconds", 2.0
        ).value)
        self.minimum_confidence = float(self.declare_parameter(
            "minimum_confidence", 0.25
        ).value)

        result_qos = QoSProfile(depth=1)
        result_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        result_qos.reliability = ReliabilityPolicy.RELIABLE
        self.detection_publisher = self.create_publisher(
            String, self.detections_topic, result_qos
        )
        self.status_publisher = self.create_publisher(
            String, "/computer_vision/detection_status", result_qos
        )
        self.capture_client = self.create_client(
            Trigger, self.capture_service_name
        )
        self.run_service = self.create_service(
            Trigger,
            "/computer_vision/run_detection",
            self._handle_detection_request,
        )

        self._state_lock = threading.Lock()
        self._request_pending = False
        self._busy = False
        self._startup_request_sent = False
        self._started_at = time.monotonic()
        self._last_service_warning_at = 0.0
        self._timer = self.create_timer(0.2, self._tick)

        self.get_logger().info("Fruit detection node ready")
        self.get_logger().info(f"  Model: {self.model_id}")
        self.get_logger().info(f"  API: {self.api_url}")
        self.get_logger().info(
            "  API key source: ROBOFLOW_API_KEY environment variable"
        )
        self.get_logger().info(
            f"  Detection results: {self.detections_topic}"
        )
        if not os.environ.get("ROBOFLOW_API_KEY"):
            self.get_logger().error(
                "ROBOFLOW_API_KEY is not set; inference requests will fail"
            )

    def _publish_status(self, status, detail=""):
        message = {"status": status, "detail": detail}
        self.status_publisher.publish(String(data=json.dumps(message)))

    def _queue_detection(self):
        with self._state_lock:
            if self._busy or self._request_pending:
                return False
            self._request_pending = True
        return True

    def _handle_detection_request(self, _request, response):
        if self._queue_detection():
            response.success = True
            response.message = (
                "Detection accepted; result will be published on "
                f"{self.detections_topic}"
            )
        else:
            response.success = False
            response.message = "A capture or inference request is already active"
        return response

    def _tick(self):
        if (
            self.run_on_start
            and not self._startup_request_sent
            and time.monotonic() - self._started_at
            >= self.startup_delay_seconds
        ):
            self._startup_request_sent = True
            self._queue_detection()

        with self._state_lock:
            should_start = self._request_pending and not self._busy
        if not should_start:
            return

        if not self.capture_client.service_is_ready():
            now = time.monotonic()
            if now - self._last_service_warning_at >= 5.0:
                self.get_logger().warning(
                    f"Waiting for capture service {self.capture_service_name}"
                )
                self._publish_status("waiting_for_camera")
                self._last_service_warning_at = now
            return

        if not os.environ.get("ROBOFLOW_API_KEY"):
            with self._state_lock:
                self._request_pending = False
            self._fail("ROBOFLOW_API_KEY is not set")
            return

        with self._state_lock:
            self._request_pending = False
            self._busy = True
        self.get_logger().info("Requesting a synchronized RGB-D snapshot...")
        self._publish_status("capturing")
        future = self.capture_client.call_async(Trigger.Request())
        future.add_done_callback(self._capture_finished)

    def _capture_finished(self, future):
        try:
            response = future.result()
        except Exception as exception:  # ROS service exceptions vary by RMW.
            self._fail(f"Capture service failed: {exception}")
            return

        if response is None or not response.success:
            detail = response.message if response else "no service response"
            self._fail(f"Camera capture failed: {detail}")
            return

        image_path = Path(response.message).expanduser().resolve()
        if not image_path.is_file():
            self._fail(f"Captured image does not exist: {image_path}")
            return

        self.get_logger().info(f"Captured {image_path}; starting inference...")
        self._publish_status("inferencing", str(image_path))
        worker = threading.Thread(
            target=self._run_inference,
            args=(image_path,),
            daemon=True,
            name="roboflow_inference",
        )
        worker.start()

    @staticmethod
    def _as_dictionary(value):
        if isinstance(value, dict):
            return value
        if hasattr(value, "model_dump"):
            return value.model_dump()
        if hasattr(value, "dict"):
            return value.dict()
        raise TypeError(f"Unsupported Roboflow response type: {type(value)}")

    def _format_results(self, raw_result, image_path):
        if isinstance(raw_result, list):
            if len(raw_result) != 1:
                raise ValueError(
                    "Expected one Roboflow result for the captured image"
                )
            raw_result = raw_result[0]
        response = self._as_dictionary(raw_result)
        image_info = self._as_dictionary(response.get("image", {}))

        detections = []
        by_label = {}
        for prediction_value in response.get("predictions", []):
            prediction = self._as_dictionary(prediction_value)
            confidence = float(prediction.get("confidence", 0.0))
            if confidence < self.minimum_confidence:
                continue

            label = str(
                prediction.get("class", prediction.get("class_name", "unknown"))
            )
            center_x = float(prediction["x"])
            center_y = float(prediction["y"])
            width = float(prediction["width"])
            height = float(prediction["height"])
            detection = {
                "id": len(detections),
                "label": label,
                "confidence": confidence,
                "center_x": center_x,
                "center_y": center_y,
                "width": width,
                "height": height,
                "x_min": center_x - width / 2.0,
                "y_min": center_y - height / 2.0,
                "x_max": center_x + width / 2.0,
                "y_max": center_y + height / 2.0,
            }
            if "class_id" in prediction:
                detection["class_id"] = int(prediction["class_id"])
            detections.append(detection)
            by_label.setdefault(label, []).append(detection)

        return {
            "schema_version": 1,
            "model_id": self.model_id,
            "image_path": str(image_path),
            "image": {
                "width": int(image_info.get("width", 0)),
                "height": int(image_info.get("height", 0)),
            },
            "detection_count": len(detections),
            "detections": detections,
            "detections_by_label": by_label,
        }

    @staticmethod
    def _save_results(result, image_path):
        destination = image_path.parent / "latest_detections.json"
        temporary = destination.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, destination)
        return destination

    def _run_inference(self, image_path):
        try:
            from inference_sdk import InferenceHTTPClient

            client = InferenceHTTPClient(
                api_url=self.api_url,
                api_key=os.environ["ROBOFLOW_API_KEY"],
            )
            raw_result = client.infer(
                str(image_path), model_id=self.model_id
            )
            result = self._format_results(raw_result, image_path)
            result_path = self._save_results(result, image_path)
            self.detection_publisher.publish(
                String(data=json.dumps(result, separators=(",", ":")))
            )

            self.get_logger().info(
                f"Roboflow detected {result['detection_count']} object(s)"
            )
            for label, detections in sorted(
                result["detections_by_label"].items()
            ):
                self.get_logger().info(f"  {label}: {len(detections)}")
                for detection in detections:
                    self.get_logger().info(
                        "    center=(%.1f, %.1f), size=(%.1f x %.1f), "
                        "confidence=%.3f"
                        % (
                            detection["center_x"],
                            detection["center_y"],
                            detection["width"],
                            detection["height"],
                            detection["confidence"],
                        )
                    )
            self.get_logger().info(f"Saved detections to {result_path}")
            self.get_logger().info(
                "Results are ready; awaiting the future user-confirmation step"
            )
            self._publish_status("results_ready", str(result_path))
            with self._state_lock:
                self._busy = False
        except ModuleNotFoundError:
            self._fail(
                "inference-sdk is not installed for this Python environment"
            )
        except Exception as exception:  # Network and SDK errors vary by version.
            self._fail(f"Roboflow inference failed: {exception}")

    def _fail(self, detail):
        self.get_logger().error(detail)
        self._publish_status("error", detail)
        with self._state_lock:
            self._busy = False


def main(args=None):
    """Run the fruit detection ROS 2 node."""
    rclpy.init(args=args)
    node = FruitDetectionNode()
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
