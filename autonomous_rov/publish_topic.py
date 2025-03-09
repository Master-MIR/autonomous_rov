import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class TopicPublisher(Node):  # <-- Inherit from Node
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Float64, "global_position/rel_alt", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float64()
        msg.data = 10.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%f"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = TopicPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()
