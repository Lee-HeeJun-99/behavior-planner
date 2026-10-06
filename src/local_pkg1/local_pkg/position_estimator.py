import rclpy

from local_pkg.tae_localization import GpsImuHeading


def main(args=None):
    rclpy.init(args=args)
    node = GpsImuHeading(
        node_name='position_estimator', publish_heading=False, publish_position=True)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
