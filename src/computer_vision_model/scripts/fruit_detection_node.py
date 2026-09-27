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
from vision_pipeline_support import phase_log


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

        self.phase_log_path = self.declare_parameter("phase_log_path", "").value
        self.one_shot = self.declare_parameter("one_shot", False).value
        self.capture_timeout = float(self.declare_parameter("capture_timeout", 45.0).value)
        self._capture_started = None
        self._completed = False
        self._next_capture_at = 0.0
        self._capture_future = None
        self._phase = "STARTUP"
        self._inference_started = None
        self._inference_active = False
        self._inference_timed_out = False
        self.inference_timeout = float(self.declare_parameter("inference_timeout", 60.0).value)

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
        phase_log(self.phase_log_path, self._phase, status, detail)

    def _queue_detection(self):
        with self._state_lock:
            if self._busy or self._inference_active or self._request_pending or (self.one_shot and self._completed):
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
            response.message = ("This one-shot run is complete; relaunch for a new capture"
                                if self.one_shot and self._completed else
                                "A capture or inference request is already active")
        return response

    def _tick(self):
        if self._inference_started is not None and time.monotonic() - self._inference_started > self.inference_timeout:
            self._inference_timed_out = True
            self._inference_started = None
            self._fail("Roboflow inference timed out; any late response will be discarded")
            return
        if self._capture_started is not None and time.monotonic() - self._capture_started > self.capture_timeout:
            if self._capture_future is not None:
                self._capture_future.cancel()
            self._capture_started = None
            with self._state_lock:
                self._request_pending = False
            self._fail("Camera capture timed out waiting for a calibrated image")
            return
        if time.monotonic() < self._next_capture_at:
            return
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

        if self._capture_started is None:
            self._capture_started = time.monotonic()
            self._phase = "CAPTURE"
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
        self.get_logger().info("Requesting a camera snapshot...")
        self._publish_status("capturing")
        future = self.capture_client.call_async(Trigger.Request())
        self._capture_future = future
        future.add_done_callback(self._capture_finished)

    def _capture_finished(self, future):
        if future.cancelled():
            return
        try:
            response = future.result()
        except Exception as exception:  # ROS service exceptions vary by RMW.
            self._fail(f"Capture service failed: {exception}")
            return

        if response is None or not response.success:
            detail = response.message if response else "no service response"
            # The driver may need several seconds to produce its first calibrated frame.
            if detail.startswith("Waiting for"):
                with self._state_lock:
                    self._busy = False
                    self._request_pending = True
                self._next_capture_at = time.monotonic() + 0.5
                return
            self._fail(f"Camera capture failed: {detail}")
            return

        image_path = Path(response.message).expanduser().resolve()
        if not image_path.is_file():
            self._fail(f"Captured image does not exist: {image_path}")
            return

        self._capture_started = None
        phase_log(self.phase_log_path, "CAPTURE", "SUCCESS",
                  "RGB image and CameraInfo saved; passing image to Roboflow.",
                  {"image_path": str(image_path), "metadata_path": str(image_path.parent / "latest_capture.yaml")})
        self._phase = "DETECTION"
        self._inference_started = time.monotonic()
        self._inference_active = True
        self._inference_timed_out = False
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
            if self._inference_timed_out:
                return
            self._inference_started = None
            result = self._format_results(raw_result, image_path)
            result_path = self._save_results(result, image_path)
            phase_log(self.phase_log_path, "DETECTION", "SUCCESS",
                      "Passing all retained detections to plane transformation (pixel units).", result)
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
                "Results published for downstream processing"
            )
            self._publish_status("results_ready", str(result_path))
            with self._state_lock:
                self._busy = False
                self._completed = True
        except ModuleNotFoundError:
            self._fail(
                "inference-sdk is not installed for this Python environment"
            )
        except Exception as exception:  # Network and SDK errors vary by version.
            if not self._inference_timed_out:
                self._fail(f"Roboflow inference failed: {exception}")
        finally:
            self._inference_active = False

    def _fail(self, detail):
        key = os.environ.get("ROBOFLOW_API_KEY", "")
        if key:
            detail = detail.replace(key, "[REDACTED]")
        self._capture_started = None
        self._inference_started = None
        self._completed = True
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
