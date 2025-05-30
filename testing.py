import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class IPPublisher(Node):
    def __init__(self):
        super().__init__('ip_publisher')
        self.publisher_ = self.create_publisher(String, 'robot_imu', 10)
        self.timer = self.create_timer(1.0, self.publish_data)

    def publish_data(self):
        msg = String()
        msg.data = "Hello from ROS 2 over IP!"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = IPPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
