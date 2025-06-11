import rclpy
from rclpy.node import Node
import can
import struct
import threading
from std_msgs.msg import Float32
import queue
import time
import numpy as np


##############################
# URDF  <-> Encoder Conversion Factor 
# knee 
# -2.7  <--> 2.7  = 5.4   => sr
# 2.7   <--> -2.7 = 5.4   => sl
# 0.00  <--> 7.06 = 7.06  => r
KNEE_FACTOR = 5.4/7.06 # for sr. for sl *-1
KNEE_OFFSET = -2.7     # for sr and sl * -1 
# Hip
# -2.3  <--> 2.3    = 4.6  => sr
# 2.3   <--> -2.3   = 4.6  => sl
# 0.00  <--> -5.36  = 5.36 => r
HIP_FACTOR  = -4.6/5.36  # for sr, for sl *-1
HIP_OFFSET  = -2.3       # for sr, for sl *-1
# ABAD
# -0.44  <--> 0.44  = .88  => sr
# 0.44   <--> -0.44 = .88  => sl
# -0.68  <--> 0.68  = 1.36 => r
ABAD_FACTOR = 0.88/1.36 # for sr, for sl *-1
ABAD_OFFSET = 0

REAL_TO_URDF_POS_FACTOR = (-1*KNEE_FACTOR,-1*HIP_FACTOR,-1*ABAD_FACTOR,KNEE_FACTOR,HIP_FACTOR,ABAD_FACTOR)
REAL_TO_URDF_POS_OFFSET = (-1*KNEE_OFFSET,-1*HIP_OFFSET,-1*ABAD_OFFSET,KNEE_OFFSET,HIP_OFFSET,ABAD_OFFSET)
################################

class MotorPublisher(Node):
    def __init__(self, data_queue):
        super().__init__('motor_sate_publisher')
        self.pub_dict = {}  # Dictionary to store publishers dynamically
        # velocity = [rad/s]
        # position = [rad]
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


        if motor_param == "position":

            # apply factor 
            # position - value is in [rad]
            value = value * ABAD_FACTOR[motor_id-1] + ABAD_OFFSET[motor_id-1] 

            # get velocity 
            velocity = _second * 2 * np.pi # convert rev/s to rad/s
            self.data_queue.put((motor_id, "velocity", velocity))

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