import rclpy
from rclpy.node import Node
import can
import struct
import threading
import queue
from std_msgs.msg import String, Float32
from enum import Enum
import time

# plot
import matplotlib.pyplot as plt
from collections import deque

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
    def __init__(self, position_queue, state_request_queue,current_pos_queue,current_vel_queue, bus):
        """Initialize motor controller."""
        self.bus = bus
        self.position_queue      = position_queue
        self.state_request_queue = state_request_queue 

        self.current_pos_queue = current_pos_queue
        self.current_vel_queue = current_vel_queue

        self.running = True
        self.motor_state = MOTOR_STATE.STOPPED

        # for id in range(1,4):
        #     self.set_up_motor(id)

        # Start both threads within the class
        self.motor_thread   = threading.Thread(target=self.process_positions, daemon=True)
        self.command_thread = threading.Thread(target=self.process_commands, daemon=True)

        self.motor_thread.start()
        self.command_thread.start()


        #plot
        self.position_data = deque(maxlen=2000)  # Store last 100 positions
        self.start_time = None  # Capture start time in milliseconds
        self.time_interval = 100  # Time interval (e.g., 100 ms per update)
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'bo-', label="Motor Position")

        self.ax.set_xlabel("Time (ms)")
        self.ax.set_ylabel("Position")
        self.ax.set_title("Real-Time Knee Motor Position")
        self.ax.legend()
        self.ax.grid(True)
        #plt.ion() 
        plt.show()

        #print(f"Motor Active")

    def send_position(self, id, position):
        print(f"{id}:{position}")
        self.bus.send(can.Message(
            arbitration_id=(id << 5 | 0x0c),  # 0x0c: Set_Input_Pos
            data=struct.pack('<f', position),
            is_extended_id=False
        ))

    def init_motor(self,mtr_id):
        print("am here")
        # self.bus.send(can.Message(
        #     arbitration_id=(mtr_id << 5 | 0x07),
        #     data=struct.pack('<I', 8),
        #     is_extended_id=False
        # ))

        # # Wait for the axis to enter closed-loop control
        # for msg in self.bus:
        #     if msg.arbitration_id == (mtr_id << 5 | 0x01):  # 0x01: Heartbeat
        #         #print(f"got here {msg.arbitration_id}")
        #         error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
        #         if state == 8:  # 8: AxisState.CLOSED_LOOP_CONTROL
        #             #print(f"got here2")
        #             break
                
        # self.bus.send(can.Message(
        #     arbitration_id=(mtr_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
        #     data=struct.pack('<ff', 1.0, 0.0), # 1.0: velocity, 0.0: torque feedforward
        #     is_extended_id=False
        # ))
    
    def stop_motor(selfi,mtr_id):
        print("am here")

        # self.bus.send(can.Message(
        #     arbitration_id=(mtr_id << 5 | 0x07),
        #     data=struct.pack('<I', 1), # disable
        #     is_extended_id=False
        # ))
        

    def process_positions(self):
        while self.running:
            
            motor_id, position = self.position_queue.get()  # Blocks until an item is available

            if self.motor_state == MOTOR_STATE.STOPPED:
                print(f"Motors havn't been initialied yet")
                continue

            # apply factor 
            

            # Update stored data
            #mtr ~40ms

            if motor_id == 1 :
                current_time = time.time() * 1000  # Get current time in milliseconds
                if self.start_time is None:  
                    self.start_time = current_time  # Initialize start_time only on first data point
                elapsed_time = current_time - self.start_time  # Compute time since first position
                self.position_data.append((elapsed_time, position))
                print(f"Received command after: {motor_id}:{position}:{elapsed_time}")
                self.update_plot()



            if ( motor_id   == 1 or motor_id == 4 ) :       # knee
                position = max(-2.67, min(position, 2.67))  # Clamp within [-2.7, 2.7] # 99%
                position = ( position + KNEE_OFFSET ) * KNEE_FACTOR 
            elif ( motor_id == 2 or motor_id == 5 ):      # hip
                position = max(-2.27, min(position, 2.27))  # Clamp within [-2.3, 2.3]
                position = ( position - HIP_OFFSET ) * HIP_FACTOR
            elif ( motor_id == 3 or motor_id == 6 ):        #abad
                position = max(-0.43, min(position, 0.43))  # Clamp within [-0.44, 0.44]
                position = ( position + ABAD_OFFSET ) * ABAD_FACTOR
            else :
                print(f"Motor_{motor_id} position {position} too large")
                continue;  

            # error_pos = control_action.joint_positions - joint_pos
            # error_vel = control_action.joint_velocities - joint_vel
            # # calculate the desired joint torques
            # self.computed_effort = self.stiffness * error_pos + self.damping * error_vel + control_action.joint_efforts

            #print(f"Received command after: {motor_id}:{position}")


            # Update plot
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

    def update_plot(self):
        """Updates the live plot using real-time elapsed time."""
        if len(self.position_data) > 0:  # Ensure we have data
            time_values, position_values = zip(*self.position_data)  # Separate time & position

            self.line.set_data(time_values, position_values)

            self.ax.relim()
            self.ax.autoscale_view()
            self.ax.set_xlim(max(0, time_values[-1] - 5000), time_values[-1] + 1000)  # Show last 5 sec

            plt.draw()
            self.fig.canvas.flush_events()  # Force redraw


class MotorListener(Node):
    """ROS 2 Node that subscribes to six motor topics and saves data in a queue."""
    
    def __init__(self, position_queue,state_request_queue,current_pos_queue,current_vel_queue):
        super().__init__('motor_cmd_listener')

        self.position_queue      = position_queue
        self.state_request_queue = state_request_queue

        self.current_pos_queue = current_pos_queue
        self.current_vel_queue = current_vel_queue

        self.topic_names = [
            '/motor_cmd_1/position', '/motor_cmd_2/position', '/motor_cmd_3/position',
            '/motor_cmd_4/position', '/motor_cmd_5/position', '/motor_cmd_6/position'
        ]
        self.all_motor_cmd_topic = '/motor_cmd_all/state'

        self.motor_current_position = [
            '/motor_state_1/position', '/motor_state_2/position', '/motor_state_3/position',
            '/motor_state_4/position', '/motor_state_5/position', '/motor_state_6/position'
        ]

        self.motor_current_velocity = [
            '/motor_state_1/velocity', '/motor_state_2/velocity', '/motor_state_3/velocity',
            '/motor_state_4/velocity', '/motor_state_5/velocity', '/motor_state_6/velocity'
        ]      

        # Initialize subscriptions for motor positions (Float32 type)
        for idx, topic in enumerate(self.topic_names, start=1):
            self.create_subscription(Float32, topic, lambda msg, m_id=idx: self.listener_callback(msg, m_id), 3)

        for idx, topic in enumerate(self.motor_current_position, start=1):
            self.create_subscription(Float32, topic, lambda msg, m_id=idx: self.current_pos_callback(msg, m_id), 3)

        for idx, topic in enumerate(self.motor_current_velocity, start=1):
            self.create_subscription(Float32, topic, lambda msg, m_id=idx: self.current_vel_callback(msg, m_id), 3)

        # Subscription for /all_motor/cmd with String commands
        self.create_subscription(String, self.all_motor_cmd_topic, self.all_motor_callback, 3)

    def listener_callback(self, msg, motor_id):
        """Stores received motor position data in the queue."""
        self.position_queue.put((motor_id, msg.data))

    def current_pos_callback(self, msg, motor_id):
        self.current_pos_queue.put((motor_id, msg.data))

    def current_vel_callback(self, msg, motor_id):
        self.current_vel_queue.put((motor_id, msg.data))

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
    bus = None#can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
    
    rclpy.init(args=args)
    position_queue      = queue.Queue()
    state_request_queue = queue.Queue()
    current_pos_queue   = queue.Queue()
    current_vel_queue   = queue.Queue()

    listener_node = MotorListener(position_queue,state_request_queue,current_pos_queue,current_vel_queue)
    listener_thread = threading.Thread(target=rclpy.spin, args=(listener_node,), daemon=True)
    listener_thread.start()

    # motor_control = MotorControl(position_queue, bus)
    # motor_thread = threading.Thread(target=motor_control.run, daemon=True)
    # motor_thread.start()
    motor_control = MotorControl(position_queue, state_request_queue,current_pos_queue,current_vel_queue, bus)

    try:
        listener_thread.join()  # Keep ROS 2 listener running
    except KeyboardInterrupt:
        listener_node.destroy_node()
        rclpy.shutdown()
        motor_control.stop()  # Stop motor threads safely
        listener_thread.join()

if __name__ == '__main__':
    main()