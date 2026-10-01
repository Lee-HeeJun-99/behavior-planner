from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int16

from traffic_sign_perception.speed_sign_utils import (
    Detection,
    bbox_area_ratio,
    bbox_to_global,
    best_speed_detection,
    class_name_to_speed,
    is_valid_normalized_roi,
    roi_pixel_bounds,
)


class SpeedSignNode(Node):
    def __init__(self):
        super().__init__('speed_sign_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('weights_path', '')
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('device', '')
        self.declare_parameter('roi_enabled', True)
        self.declare_parameter('roi_x_min', 0.45)
        self.declare_parameter('roi_x_max', 1.0)
        self.declare_parameter('roi_y_min', 0.05)
        self.declare_parameter('roi_y_max', 0.75)
        self.declare_parameter('min_bbox_area_ratio', 0.0)
        self.declare_parameter('publish_debug_image', True)

        image_topic = self.get_parameter('image_topic').value
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.device = str(self.get_parameter('device').value)
        self.roi_enabled = bool(self.get_parameter('roi_enabled').value)
        self.roi = (
            float(self.get_parameter('roi_x_min').value),
            float(self.get_parameter('roi_x_max').value),
            float(self.get_parameter('roi_y_min').value),
            float(self.get_parameter('roi_y_max').value),
        )
        self.min_bbox_area_ratio = max(
            0.0, float(self.get_parameter('min_bbox_area_ratio').value))
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        if self.roi_enabled and not is_valid_normalized_roi(*self.roi):
            self.get_logger().warning(
                f'Invalid normalized ROI {self.roi}; falling back to the full image')
            self.roi_enabled = False

        self.bridge = CvBridge()
        self.model = self._load_model(str(self.get_parameter('weights_path').value))
        self.publisher = self.create_publisher(Int16, '/Perception/speed_limit', 10)
        self.debug_publisher = None
        if self.publish_debug_image:
            self.debug_publisher = self.create_publisher(
                Image, '/Perception/speed_sign/debug_image', 2)
        self.subscription = self.create_subscription(
            Image, image_topic, self._image_callback, qos_profile_sensor_data)

    def _load_model(self, configured_path):
        if not configured_path:
            self.get_logger().warning(
                'YOLO weights_path is empty; publishing speed_limit=0 until weights are configured')
            return None
        weights_path = Path(configured_path).expanduser()
        if not weights_path.is_absolute():
            weights_path = (
                Path(get_package_share_directory('traffic_sign_perception')) /
                'weights' / weights_path)
        if not weights_path.is_file():
            self.get_logger().warning(
                f'YOLO weight file not found: {weights_path}; publishing speed_limit=0')
            return None
        try:
            from ultralytics import YOLO
            model = YOLO(str(weights_path))
            self.get_logger().info(f'Loaded speed-sign YOLO weights: {weights_path}')
            return model
        except (ImportError, RuntimeError, OSError) as error:
            self.get_logger().warning(
                f'Unable to initialize YOLO: {error}; publishing speed_limit=0')
            return None

    def _image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            height, width = image.shape[:2]
            if self.roi_enabled:
                x1, y1, x2, y2 = roi_pixel_bounds(width, height, *self.roi)
            else:
                x1, y1, x2, y2 = 0, 0, width, height
            inference_image = image[y1:y2, x1:x2]
            detections = self._infer(inference_image) if self.model is not None else []
            selected = best_speed_detection(
                detections, inference_image.shape[1], inference_image.shape[0],
                self.min_bbox_area_ratio)
            detected_limit = class_name_to_speed(selected.class_name) if selected else 0
            self.publisher.publish(Int16(data=detected_limit))
            self._publish_debug(message, image, detections, selected, (x1, y1, x2, y2))
        except Exception as error:  # Keep the node alive on conversion/backend errors.
            self.get_logger().warning(
                f'YOLO inference failed: {error}', throttle_duration_sec=2.0)
            self.publisher.publish(Int16(data=0))

    def _infer(self, image):
        predict_args = {
            'source': image,
            'conf': self.confidence_threshold,
            'verbose': False,
        }
        if self.device:
            predict_args['device'] = self.device
        results = self.model.predict(**predict_args)
        detections = []
        for result in results:
            names = result.names
            for box in result.boxes:
                class_index = int(box.cls[0].item())
                class_name = names[class_index] if isinstance(names, dict) else names[class_index]
                bbox = tuple(float(value) for value in box.xyxy[0].tolist())
                detections.append(Detection(
                    str(class_name).lower(), float(box.conf[0].item()), bbox))
        return detections

    def _publish_debug(self, source_message, image, detections, selected, roi_bounds):
        if self.debug_publisher is None:
            return
        import cv2

        debug = image.copy()
        x1, y1, x2, y2 = roi_bounds
        if self.roi_enabled:
            cv2.rectangle(debug, (x1, y1), (x2 - 1, y2 - 1), (0, 255, 255), 2)
            cv2.putText(debug, 'ROI', (x1 + 5, max(20, y1 + 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        roi_width, roi_height = x2 - x1, y2 - y1
        for detection in detections:
            if not class_name_to_speed(detection.class_name):
                continue
            if bbox_area_ratio(
                    detection.bbox, roi_width, roi_height) < self.min_bbox_area_ratio:
                continue
            gx1, gy1, gx2, gy2 = bbox_to_global(detection.bbox, x1, y1)
            color = (0, 255, 0) if detection == selected else (255, 160, 0)
            cv2.rectangle(debug, (int(gx1), int(gy1)), (int(gx2), int(gy2)), color, 2)
            label = f'{detection.class_name} {detection.confidence:.2f}'
            cv2.putText(debug, label, (int(gx1), max(20, int(gy1) - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        debug_message = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        debug_message.header = source_message.header
        self.debug_publisher.publish(debug_message)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedSignNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
