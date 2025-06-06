import rclpy
from rclpy.node import Node
import can
import struct
import threading
from std_msgs.msg import Float32
import queue
import time


##############################
# URDF  <-> Encoder Conversion Factor 
# knee 
# -2.7  <--> 2.7  = 5.4 
# 0.00  <--> 7.06 = 7.06
KNEE_FACTOR = 5.4/7.06
KNEE_OFFSET = 2.7
# Hip
# -2.2  <--> 2.2    = 4.4
# 0.00  <--> -5.36  = -5.36
HIP_FACTOR  = 4.4/5.36
HIP_OFFSET  = 2.2
# ABAD
# -0.44  <--> 0.44 = .88
# -0.68  <--> 0.68 = 1.36
ABAD_FACTOR = 0.88/1.36
ABAD_OFFSET = 0
################################

class MotorPublisher(Node):
    def __init__(self, data_queue):
        super().__init__('motor_sate_publisher')
        self.pub_dict = {}  # Dictionary to store publishers dynamically
        self.parameters = ["error", "temperature", "torque", "position", "current", "batt_voltage", "velocity"]
        self.data_queue = data_queue  # Queue for incoming motor data

    def get_publisher(self, motor_id, param):
        """Creates or retrieves an existing publisher for a motor's parameter."""
        topic_name = f"/motor_state_{motor_id}/{param}"
        if topic_name not in self.pub_dict:
            self.pub_dict[topic_name] = self.create_publisher(Float32, topic_name, 10)
        return self.pub_dict[topic_name]

    def publish_from_queue(self):
        """Continuously publishes messages from the queue."""
        while rclpy.ok():
            try:
                motor_id, param, value = self.data_queue.get(timeout=1)  # Wait for data
                pub = self.get_publisher(motor_id, param)
                msg = Float32()
                msg.data = value
                #self.get_logger().info(f"Publishing {param} for Motor {motor_id}: {msg.data}")
                pub.publish(msg)
                self.data_queue.task_done()  # Mark task as done
            except queue.Empty:
                pass  # If queue is empty, continue waiting

class MotorCAN:
    def __init__(self, data_queue,bus):
        """Initialize CAN bus and define motor IDs and parameters."""
        self.params = { 
            0x09: "position",
            0x14: "current",
            0x1C: "torque",
            0x03: "error",
            0x17: "batt_voltage",
            0x15: "temperature"
        }
        self.data_queue = data_queue  # Shared queue
        self.bus = bus
        self.prev_positions = {}
        self.prev_timestamps = {}
        self.origin_time = time.monotonic() #ticks_ms()  

    def process_can_message(self, msg):
        """Extract and categorize messages by motor and parameter."""
        motor_id = msg.arbitration_id >> 5
        motor_param = self.params.get(msg.arbitration_id & 0b00011111)

        if motor_param == None:
            return

        # take only interested values
        if motor_param == "error":
            _first, _second = struct.unpack('<II', bytes(msg.data))
            value = float(_second)
            self.data_queue.put((motor_id, motor_param, value)) 
            return
        else:
            _first, _second = struct.unpack('<ff', bytes(msg.data))
            value = _second if motor_param in ["current", "torque"] else _first

        # apply factor 
        if (motor_id == 1 or motor_id == 4) and motor_param == "position":
            value = value * KNEE_FACTOR - KNEE_OFFSET
        elif (motor_id == 2 or motor_id == 5) and motor_param == "position":
            value = value * HIP_FACTOR + HIP_OFFSET
        elif (motor_id == 3 or motor_id == 6) and motor_param == "position":
            value = value * ABAD_FACTOR - ABAD_OFFSET

        # Calculate velocity
        if motor_param == "position":
            current_time = time.time()
            if motor_id in self.prev_positions:
                time_diff = current_time - self.prev_timestamps[motor_id]
                if time_diff > 0:  # Prevent division by zero
                    velocity = (value - self.prev_positions[motor_id]) / time_diff
                    self.data_queue.put((motor_id, "velocity", velocity))
            self.prev_positions[motor_id] = value
            self.prev_timestamps[motor_id] = current_time

        # Add to queue
        self.data_queue.put((motor_id, motor_param, value)) 

    def listen(self):
        """Continuously listen for incoming CAN messages."""
        print("Listening for CAN messages...")
        while True:
            msg = self.bus.recv()
            self.process_can_message(msg)

def main(args=None):

    bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
    while not (bus.recv(timeout=0) is None): pass
    
    rclpy.init(args=args)
    data_queue = queue.Queue()  # Create shared queue

    # Start ROS publisher in a separate thread
    motor_publisher = MotorPublisher(data_queue)
    publisher_thread = threading.Thread(target=motor_publisher.publish_from_queue, daemon=True)
    publisher_thread.start()

    # Start CAN listener in a separate thread
    can_listener = MotorCAN(data_queue, bus)
    can_thread = threading.Thread(target=can_listener.listen, daemon=True)
    can_thread.start()

    print(f"It has begun publishing")
    rclpy.spin(motor_publisher)  # Keep ROS node running
    motor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()