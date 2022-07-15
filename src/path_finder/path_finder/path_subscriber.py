import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class PathSubscriber(Node):

    def __init__(self):
        super().__init__('path_subscriber')
        self.subscription = self.create_subscription(
            String,
            'path',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('%s' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    path_subscriber = PathSubscriber()

    rclpy.spin(path_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
