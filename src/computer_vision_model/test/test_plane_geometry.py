"""Analytic checks for the coordinate conversion; no ROS or hardware needed."""
import sys
from pathlib import Path
import unittest
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from vision_pipeline_support import project_box_to_plane


class PlaneGeometryTests(unittest.TestCase):
    def setUp(self):
        self.metadata = {'rgb': {'width': 640, 'height': 480}, 'camera_intrinsics':
                         {'fx': 500., 'fy': 500., 'cx': 320., 'cy': 240.,
                          'distortion_model': 'plumb_bob', 'd': [0., 0., 0., 0., 0.]}}
        self.translation = [0.22, -0.29, 0.434]
        self.rotation = (Rotation.from_euler('xyz', [0, 50, 90], degrees=True) *
                         Rotation.from_euler('xyz', [-90, 0, -90], degrees=True))

    def project(self, pixel, rotation=None):
        return project_box_to_plane({'center_x': pixel[0], 'center_y': pixel[1]},
            self.metadata, self.translation, (rotation or self.rotation).as_quat(), -0.006)

    def test_centre_ray_at_corrected_height(self):
        result = self.project([320, 240])
        expected_y = -0.29 + 0.440 / np.tan(np.deg2rad(50))
        np.testing.assert_allclose(result['point_base_m'], [0.22, expected_y, -0.006], atol=1e-10)
        self.assertAlmostEqual(result['point_camera_m'][2], 0.440 / np.sin(np.deg2rad(50)))

    def test_known_off_centre_world_point(self):
        world = np.array([0.18, 0.12, -0.006])
        camera = self.rotation.inv().apply(world - self.translation)
        pixel = [500*camera[0]/camera[2]+320, 500*camera[1]/camera[2]+240]
        np.testing.assert_allclose(self.project(pixel)['point_base_m'], world, atol=1e-10)

    def test_lens_distortion_is_removed(self):
        world = np.array([0.12, 0.08, -0.006])
        camera = self.rotation.inv().apply(world - self.translation)
        distortion = [0.12, -0.04, 0.001, 0.002, 0.01]
        self.metadata['camera_intrinsics']['d'] = distortion
        pixel, _ = cv2.projectPoints(camera.reshape(1, 3), np.zeros(3), np.zeros(3),
            np.array([[500., 0, 320.], [0, 500., 240.], [0, 0, 1.]]), np.array(distortion))
        np.testing.assert_allclose(self.project(pixel.reshape(2))['point_base_m'], world, atol=1e-7)

    def test_parallel_ray_rejected(self):
        with self.assertRaisesRegex(ValueError, 'parallel'):
            self.project([320, 240], Rotation.from_euler('y', 90, degrees=True))

    def test_plane_behind_camera_rejected(self):
        with self.assertRaisesRegex(ValueError, 'behind'):
            self.project([320, 240], Rotation.identity())

    def test_out_of_image_rejected(self):
        with self.assertRaisesRegex(ValueError, 'outside'):
            self.project([640, 240])


if __name__ == '__main__':
    unittest.main()
