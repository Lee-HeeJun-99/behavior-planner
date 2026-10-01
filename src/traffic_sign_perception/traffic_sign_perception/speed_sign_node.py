from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int16


CLASS_TO_LIMIT = {
    'speed_30': 30,
    'speed_40': 40,
    'speed_50': 50,
}


class SpeedSignNode(Node):
    def __init__(self):
        super().__init__('speed_sign_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('weights_path', '')
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('confirmation_count', 2)
        self.declare_parameter('device', '')

        image_topic = self.get_parameter('image_topic').value
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.confirmation_count = max(1, int(self.get_parameter('confirmation_count').value))
        self.device = str(self.get_parameter('device').value)
        self.bridge = CvBridge()
        self.model = self._load_model(str(self.get_parameter('weights_path').value))
        self.candidate_limit = 0
        self.candidate_count = 0

        self.publisher = self.create_publisher(Int16, '/Perception/speed_limit', 10)
        self.subscription = self.create_subscription(
            Image, image_topic, self._image_callback, qos_profile_sensor_data)

    def _load_model(self, configured_path):
        if not configured_path:
            self.get_logger().warning(
                'YOLO weights_path is empty; publishing speed_limit=0 until weights are configured')
            return None
        weights_path = Path(configured_path).expanduser()
        if not weights_path.is_absolute():
            weights_path = Path(get_package_share_directory('traffic_sign_perception')) / 'weights' / weights_path
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
            self.get_logger().warning(f'Unable to initialize YOLO: {error}; publishing speed_limit=0')
            return None

    def _image_callback(self, message):
        if self.model is None:
            self._update_filter(0)
            return
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            predict_args = {
                'source': image,
                'conf': self.confidence_threshold,
                'verbose': False,
            }
            if self.device:
                predict_args['device'] = self.device
            results = self.model.predict(**predict_args)
            self._update_filter(self._best_limit(results))
        except Exception as error:  # Keep the ROS node alive on malformed images/backend errors.
            self.get_logger().warning(f'YOLO inference failed: {error}', throttle_duration_sec=2.0)
            self._update_filter(0)

    @staticmethod
    def _best_limit(results):
        best_limit = 0
        best_confidence = -1.0
        for result in results:
            names = result.names
            for box in result.boxes:
                class_index = int(box.cls[0].item())
                confidence = float(box.conf[0].item())
                class_name = names[class_index] if isinstance(names, dict) else names[class_index]
                limit = CLASS_TO_LIMIT.get(str(class_name).lower(), 0)
                if limit and confidence > best_confidence:
                    best_limit = limit
                    best_confidence = confidence
        return best_limit

    def _update_filter(self, detected_limit):
        if detected_limit == 0:
            self.candidate_limit = 0
            self.candidate_count = 0
            self.publisher.publish(Int16(data=0))
            return
        if detected_limit == self.candidate_limit:
            self.candidate_count += 1
        else:
            self.candidate_limit = detected_limit
            self.candidate_count = 1
        confirmed = detected_limit if self.candidate_count >= self.confirmation_count else 0
        self.publisher.publish(Int16(data=confirmed))


def main(args=None):
    rclpy.init(args=args)
    node = SpeedSignNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
