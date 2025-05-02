import rclpy
from rclpy.node import Node
import can
import struct
import threading
import queue
from std_msgs.msg import String, Float32
from enum import Enum

##############################
# URDF  <-> Encoder Conversion Factor 
# knee 
# -2.7  <--> 2.7  = 5.4 
# 0.00  <--> 7.06 = 7.06
KNEE_FACTOR = 7.06/5.4
KNEE_OFFSET = 2.7
# Hip
# -2.2  <--> 2.2    = 4.4
# 0.00  <--> -5.36  = -5.36
HIP_FACTOR  = 5.36/4.4
HIP_OFFSET  = 2.2
# ABAD
# -0.44  <--> 0.44 = .88
# -0.68  <--> 0.68 = 1.36
ABAD_FACTOR = 1.36/0.88
ABAD_OFFSET = 0
################################

################################
# Motor State
class MOTOR_STATE (Enum):
    INITIALIZED = 0x01
    STOPPED     = 0x02

################################
class MotorControl:
    def __init__(self, position_queue, state_request_queue, bus):
        """Initialize motor controller."""
        self.bus = bus
        self.position_queue      = position_queue
        self.state_request_queue = state_request_queue 

        self.running = True
        self.motor_state = MOTOR_STATE.STOPPED

        # for id in range(1,4):
        #     self.set_up_motor(id)

        # Start both threads within the class
        self.motor_thread   = threading.Thread(target=self.process_positions, daemon=True)
        self.command_thread = threading.Thread(target=self.process_commands, daemon=True)

        self.motor_thread.start()
        self.command_thread.start()
        
        #print(f"Motor Active")

    def send_position(self, id, position):
        print(f"{id}:{position}")
        self.bus.send(can.Message(
            arbitration_id=(id << 5 | 0x0c),  # 0x0c: Set_Input_Pos
            data=struct.pack('<f', position),
            is_extended_id=False
        ))

    def init_motor(self,mtr_id):
        self.bus.send(can.Message(
            arbitration_id=(mtr_id << 5 | 0x07),
            data=struct.pack('<I', 8),
            is_extended_id=False
        ))

        # Wait for the axis to enter closed-loop control
        for msg in self.bus:
            if msg.arbitration_id == (mtr_id << 5 | 0x01):  # 0x01: Heartbeat
                #print(f"got here {msg.arbitration_id}")
                error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
                if state == 8:  # 8: AxisState.CLOSED_LOOP_CONTROL
                    #print(f"got here2")
                    break
                
        # self.bus.send(can.Message(
        #     arbitration_id=(mtr_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
        #     data=struct.pack('<ff', 1.0, 0.0), # 1.0: velocity, 0.0: torque feedforward
        #     is_extended_id=False
        # ))
    
    def stop_motor(self,mtr_id):
        self.bus.send(can.Message(
            arbitration_id=(mtr_id << 5 | 0x07),
            data=struct.pack('<I', 1), # disable
            is_extended_id=False
        ))
        

    def process_positions(self):
        while self.running:
            
            motor_id, position = self.position_queue.get()  # Blocks until an item is available

            if self.motor_state == MOTOR_STATE.STOPPED:
                print(f"Motors havn't been initialied yet")
                continue

            # apply factor 
            if motor_id   == 1 and -2.7 <= position <= 2.7:
                position = ( position + KNEE_OFFSET ) * KNEE_FACTOR 
            elif motor_id == 2 and -2.2 <= position <= 2.2:
                position = ( position - HIP_OFFSET ) * HIP_FACTOR
            elif motor_id == 3 and -0.44 <= position <= 0.44:
                position = ( position + ABAD_OFFSET ) * ABAD_FACTOR
            else :
                print(f"Motor_{motor_id} position {position} too large")
                continue;       

            print(f"Received command: {motor_id}:{position}")
            #self.send_position(motor_id, position)
    
    def process_commands(self):
        while self.running:
            command = self.state_request_queue.get()

            if command == "start" and self.motor_state == MOTOR_STATE.STOPPED:
                
                for id in range(1,7):
                    self.init_motor(id)

                self.motor_state = MOTOR_STATE.INITIALIZED
                print(f"Received command: {command}")
            elif command == "stop" and self.motor_state == MOTOR_STATE.INITIALIZED:

                for id in range(1,7):
                    self.stop_motor(id)

                self.motor_state = MOTOR_STATE.STOPPED
                print(f"Received command: {command}")
            else:
                print(f"Command Not Recognized: {command}")


    def stop(self):
        self.running = False
        self.motor_thread.join()
        self.command_thread.join()


class MotorListener(Node):
    """ROS 2 Node that subscribes to six motor topics and saves data in a queue."""
    
    def __init__(self, position_queue,state_request_queue):
        super().__init__('motor_cmd_listener')
        self.position_queue      = position_queue
        self.state_request_queue = state_request_queue
        self.topic_names = [
            '/motor_cmd_1/position', '/motor_cmd_2/position', '/motor_cmd_3/position',
            '/motor_cmd_4/position', '/motor_cmd_5/position', '/motor_cmd_6/position'
        ]
        self.all_motor_cmd_topic = '/motor_cmd_all/state'

        # Initialize subscriptions for motor positions (Float32 type)
        for idx, topic in enumerate(self.topic_names, start=1):
            self.create_subscription(Float32, topic, lambda msg, m_id=idx: self.listener_callback(msg, m_id), 10)

        # Subscription for /all_motor/cmd with String commands
        self.create_subscription(String, self.all_motor_cmd_topic, self.all_motor_callback, 10)

    def listener_callback(self, msg, motor_id):
        """Stores received motor position data in the queue."""
        self.position_queue.put((motor_id, msg.data))

    def all_motor_callback(self, msg):
        """Handles start/stop commands for all motors."""
        command = msg.data.strip().lower()  # Normalize command input
        self.state_request_queue.put(command)

        # debugging
        # if command in ["start", "stop"]:
        #     print(f"Received command for all motors: {command}")
        # else:
        #     print(f"Invalid command: {command}")



def main(args=None):
    bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
    
    rclpy.init(args=args)
    position_queue      = queue.Queue()
    state_request_queue = queue.Queue()

    listener_node = MotorListener(position_queue,state_request_queue)
    listener_thread = threading.Thread(target=rclpy.spin, args=(listener_node,), daemon=True)
    listener_thread.start()

    # motor_control = MotorControl(position_queue, bus)
    # motor_thread = threading.Thread(target=motor_control.run, daemon=True)
    # motor_thread.start()
    motor_control = MotorControl(position_queue, state_request_queue, bus)

    try:
        listener_thread.join()  # Keep ROS 2 listener running
    except KeyboardInterrupt:
        print("Shutting down gracefully...")
        listener_node.destroy_node()
        rclpy.shutdown()
        print("YEBE 3")
        motor_control.stop()  # Stop motor threads safely
        print("YEBE 2")
        print("YEBE 1")
        listener_thread.join()

if __name__ == '__main__':
    main()