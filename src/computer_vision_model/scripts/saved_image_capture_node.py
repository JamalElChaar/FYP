#!/usr/bin/env python3
"""Provide the existing capture service from an image and calibration on disk."""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger

from saved_image_input import prepare_saved_image
from vision_pipeline_support import phase_log


class SavedImageCaptureNode(Node):
    def __init__(self):
        super().__init__('saved_image_capture_node')
        self.image_path = self.declare_parameter('image_path', '').value
        self.calibration_path = self.declare_parameter('calibration_path', '').value
        self.output_directory = self.declare_parameter('output_directory', '/tmp/computer_vision_captures').value
        self.phase_path = self.declare_parameter('phase_log_path', '').value
        self.create_service(Trigger, '/computer_vision/capture_latest', self.capture)
        self.get_logger().info(f'Saved-image input ready: {self.image_path}')

    def capture(self, _request, response):
        try:
            path, metadata = prepare_saved_image(self.image_path, self.calibration_path, self.output_directory)
            phase_log(self.phase_path, 'IMAGE_INPUT', 'SUCCESS',
                      'Loaded image and matching calibration; passing copied RGB image to detection.', {
                          'source_image': metadata['capture']['source_image'],
                          'source_calibration': metadata['capture']['source_calibration'],
                          'image_dimensions': metadata['rgb'],
                          'camera_intrinsics': metadata['camera_intrinsics'],
                          'recorded_transform': metadata['transform_camera_to_base'],
                          'pipeline_image': str(path)})
            response.success = True
            response.message = str(path)
            self.get_logger().info(f'Prepared saved image: {path}')
        except Exception as error:
            response.success = False
            response.message = str(error)
            phase_log(self.phase_path, 'IMAGE_INPUT', 'ERROR', response.message)
            self.get_logger().error(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SavedImageCaptureNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
