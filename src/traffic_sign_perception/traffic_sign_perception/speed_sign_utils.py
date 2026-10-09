from dataclasses import dataclass
from typing import Iterable, Optional, Tuple


CLASS_TO_LIMIT = {
    'speed_20': 20, 'speed_50': 50,
    'speed limit 20': 20, 'speed limit 50': 50,
}


@dataclass(frozen=True)
class Detection:
    class_name: str
    confidence: float
    bbox: Tuple[float, float, float, float]


def is_valid_normalized_roi(x_min, x_max, y_min, y_max):
    return 0.0 <= x_min < x_max <= 1.0 and 0.0 <= y_min < y_max <= 1.0


def roi_pixel_bounds(width, height, x_min, x_max, y_min, y_max):
    return (int(width * x_min), int(height * y_min),
            int(width * x_max), int(height * y_max))


def bbox_to_global(bbox, offset_x, offset_y):
    x1, y1, x2, y2 = bbox
    return x1 + offset_x, y1 + offset_y, x2 + offset_x, y2 + offset_y


def bbox_area_ratio(bbox, image_width, image_height):
    x1, y1, x2, y2 = bbox
    area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    return area / max(1.0, float(image_width * image_height))


def class_name_to_speed(class_name):
    return CLASS_TO_LIMIT.get(str(class_name).lower(), 0)


def best_speed_detection(
        detections: Iterable[Detection], image_width: int, image_height: int,
        min_bbox_area_ratio: float = 0.0) -> Optional[Detection]:
    eligible = (
        detection for detection in detections
        if class_name_to_speed(detection.class_name) and
        bbox_area_ratio(detection.bbox, image_width, image_height) >= min_bbox_area_ratio
    )
    return max(eligible, key=lambda detection: detection.confidence, default=None)
