import unittest

from traffic_sign_perception.speed_sign_utils import (
    Detection,
    bbox_area_ratio,
    bbox_to_global,
    best_speed_detection,
    class_name_to_speed,
    is_valid_normalized_roi,
    roi_pixel_bounds,
)


class SpeedSignUtilsTest(unittest.TestCase):
    def test_roi_validation(self):
        self.assertTrue(is_valid_normalized_roi(0.45, 1.0, 0.05, 0.75))
        self.assertFalse(is_valid_normalized_roi(0.8, 0.2, 0.05, 0.75))
        self.assertFalse(is_valid_normalized_roi(-0.1, 1.0, 0.05, 0.75))

    def test_roi_pixel_coordinates(self):
        self.assertEqual(
            roi_pixel_bounds(1280, 720, 0.45, 1.0, 0.05, 0.75),
            (576, 36, 1280, 540))

    def test_bbox_coordinates_are_restored_to_full_image(self):
        self.assertEqual(bbox_to_global((10, 20, 110, 120), 576, 36),
                         (586, 56, 686, 156))

    def test_class_mapping_accepts_only_supported_classes(self):
        self.assertEqual(class_name_to_speed('speed_50'), 50)
        self.assertEqual(class_name_to_speed('SPEED_20'), 20)
        self.assertEqual(class_name_to_speed('speed_30'), 0)
        self.assertEqual(class_name_to_speed('speed_40'), 0)
        self.assertEqual(class_name_to_speed('traffic_light'), 0)

    def test_bbox_area_ratio(self):
        self.assertAlmostEqual(bbox_area_ratio((0, 0, 10, 10), 100, 100), 0.01)

    def test_best_detection_filters_class_and_small_bbox(self):
        detections = [
            Detection('traffic_light', 0.99, (0, 0, 90, 90)),
            Detection('speed_20', 0.95, (0, 0, 2, 2)),
            Detection('speed_50', 0.80, (10, 10, 40, 40)),
            Detection('speed_40', 0.70, (10, 10, 50, 50)),
        ]
        selected = best_speed_detection(detections, 100, 100, 0.01)
        self.assertIsNotNone(selected)
        self.assertEqual(selected.class_name, 'speed_50')


if __name__ == '__main__':
    unittest.main()
