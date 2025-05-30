import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import queue
import threading
import time

class IMUListener(Node):
    """ROS 2 Node that subscribes to 'robot_imu' and saves data in a FIFO queue."""
    def __init__(self, data_queue):
        super().__init__('imu_listener')
        self.data_queue = data_queue
        self.subscription = self.create_subscription(String, 'robot_imu', self.listener_callback, 10)

    def listener_callback(self, msg):
        """Stores received IMU data in the queue."""
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
        imu_data = f"[{timestamp}] {msg.data}"
        self.data_queue.put(imu_data)  # Add to FIFO queue
        #self.get_logger().info(f"Stored in queue: {imu_data}")

class QueueReader:
    """Reads and processes FIFO queue data in parallel."""
    def __init__(self, data_queue):
        self.data_queue = data_queue

    def run(self):
        """Continuously reads and prints data from the queue."""
        while True:
            if not self.data_queue.empty():
                imu_data = self.data_queue.get()  # Retrieve from FIFO queue
                print(f"Processed IMU Data: {imu_data}")
            #time.sleep(0.1)  # Avoid busy waiting

def main(args=None):
    rclpy.init(args=args)
    data_queue = queue.Queue()  # FIFO queue

    # Start listener node in a separate thread
    listener_node = IMUListener(data_queue)
    listener_thread = threading.Thread(target=rclpy.spin, args=(listener_node,), daemon=True)
    listener_thread.start()

    # Start queue reader in a separate thread
    queue_reader = QueueReader(data_queue)
    reader_thread = threading.Thread(target=queue_reader.run, daemon=True)
    reader_thread.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass

    # Cleanup
    listener_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
