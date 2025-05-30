import rclpy
from rclpy.node import Node
import threading
import time
import random
from std_msgs.msg import String

class BNO055Data:
    """A shared data structure to hold IMU readings with timestamp."""
    def __init__(self):
        self.timestamp = time.time()
        self.acceleration = (0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.lock = threading.Lock()  # Thread safety

    def update(self, accel, gyro):
        """Updates IMU readings with timestamp."""
        with self.lock:
            self.timestamp = time.time()
            self.acceleration = accel
            self.gyro = gyro

    def read(self):
        """Reads IMU data safely."""
        with self.lock:
            return self.timestamp, self.acceleration, self.gyro

class IMUDataGenerator(Node):
    """ROS 2 Node that publishes IMU data at 100Hz to 'robot_imu' using a simple format."""
    def __init__(self, shared_data):
        super().__init__('robot_imu_publisher')
        self.shared_data = shared_data
        self.publisher_ = self.create_publisher(String, 'robot_imu', 10)
        self.timer = self.create_timer(0.01, self.publish_imu_data)  # 100 Hz

    def publish_imu_data(self):
        """Publishes IMU data with timestamp in custom format."""
        timestamp, accel, gyro = self.shared_data.read()

        # Custom IMU message as a formatted string
        msg = String()
        msg.data = f"{timestamp},{accel[0]},{accel[1]},{accel[2]},{gyro[0]},{gyro[1]},{gyro[2]}"
        self.publisher_.publish(msg)

        print(f"[{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(timestamp))}] Publishing IMU Data: {msg}")

def main(args=None):
    rclpy.init(args=args)
    shared_data = BNO055Data()

    # Start generator in background thread
    generator_thread = threading.Thread(target=lambda: generate_imu_data(shared_data), daemon=True)
    generator_thread.start()

    # Start ROS 2 publisher node
    node = IMUDataGenerator(shared_data)
    rclpy.spin(node)

    # Clean up
    node.destroy_node()
    rclpy.shutdown()

def generate_imu_data(shared_data):
    """Simulates continuous IMU data generation."""
    while True:
        accel = (random.uniform(-2, 2), random.uniform(-2, 2), random.uniform(-2, 2))
        gyro = (random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
        shared_data.update(accel, gyro)
        time.sleep(0.01)  # 100 Hz

if __name__ == '__main__':
    main()