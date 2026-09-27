"""Saved-image validation and lossless input handling without camera hardware."""
from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from saved_image_input import prepare_saved_image


class SavedImageInputTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root / 'fruit.png'
        cv2.imwrite(str(self.source), np.full((48, 64, 3), 100, dtype=np.uint8))
        self.metadata = {
            'capture': {'rgb_frame_id': 'optical', 'rgb_stamp': [123, 456]},
            'rgb': {'width': 64, 'height': 48},
            'camera_intrinsics': {'fx': 50., 'fy': 50., 'cx': 32., 'cy': 24.},
            'transform_camera_to_base': {
                'parent_frame': 'base_link', 'child_frame': 'optical',
                'translation_m': [.22, -.29, .434], 'quaternion_xyzw': [1., 0., 0., 0.]}}
        self.calibration = self.source.with_suffix('.yaml')
        self.write_calibration()

    def write_calibration(self):
        self.calibration.write_text(yaml.safe_dump(self.metadata))

    def prepare(self, calibration=''):
        return prepare_saved_image(self.source, calibration, self.root / 'run')

    def test_auto_sidecar_and_source_not_modified(self):
        original = self.source.read_bytes()
        original_yaml = self.calibration.read_bytes()
        path, meta = self.prepare()
        self.assertTrue(path.is_file())
        self.assertEqual(cv2.imread(str(path)).shape, (48, 64, 3))
        self.assertEqual(meta['capture']['rgb_stamp'], [0, 0])
        self.assertEqual(meta['capture']['source_rgb_stamp'], [123, 456])
        self.assertEqual(meta['transform_camera_to_base'], self.metadata['transform_camera_to_base'])
        self.assertEqual(self.source.read_bytes(), original)
        self.assertEqual(self.calibration.read_bytes(), original_yaml)

    def test_auto_live_capture_filename(self):
        self.calibration.rename(self.root / 'latest_capture.yaml')
        _, meta = self.prepare()
        self.assertTrue(meta['capture']['source_calibration'].endswith('latest_capture.yaml'))

    def test_explicit_calibration(self):
        alternate = self.root / 'other.yaml'
        self.calibration.rename(alternate)
        self.assertTrue(self.prepare(str(alternate))[0].is_file())

    def test_no_calibration(self):
        self.calibration.unlink()
        with self.assertRaisesRegex(ValueError, 'Matching calibration'):
            self.prepare()

    def test_wrong_size(self):
        self.metadata['rgb']['width'] = 128
        self.write_calibration()
        with self.assertRaisesRegex(ValueError, 'calibration is 128x48'):
            self.prepare()

    def test_missing_extrinsics(self):
        del self.metadata['transform_camera_to_base']
        self.write_calibration()
        with self.assertRaisesRegex(ValueError, 'transform_camera_to_base'):
            self.prepare()

    def test_wrong_transform_direction(self):
        self.metadata['transform_camera_to_base']['parent_frame'] = 'optical'
        self.write_calibration()
        with self.assertRaisesRegex(ValueError, 'parent_frame'):
            self.prepare()

    def test_corrupt_image(self):
        self.source.write_text('not an image')
        with self.assertRaisesRegex(ValueError, 'Cannot decode'):
            self.prepare()

    def test_invalid_quaternion(self):
        self.metadata['transform_camera_to_base']['quaternion_xyzw'] = [0., 0., 0., 0.]
        self.write_calibration()
        with self.assertRaisesRegex(ValueError, 'Invalid recorded'):
            self.prepare()


if __name__ == '__main__':
    unittest.main()
