import rclpy
from rclpy.node import Node
import can
import struct
import threading
import queue
from std_msgs.msg import String, Float32
from enum import Enum
import time
from dataclasses import dataclass

# plot
# import matplotlib.pyplot as plt
# from collections import deque

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

# position conversion 
URDF_TO_REAL_POS_FACTOR = (ABAD_FACTOR,ABAD_FACTOR,HIP_FACTOR,HIP_FACTOR,KNEE_FACTOR,KNEE_FACTOR)
URDF_TO_REAL_POS_OFFSET = (ABAD_OFFSET,ABAD_OFFSET,-1*HIP_OFFSET,-1*HIP_OFFSET,KNEE_OFFSET,KNEE_OFFSET)

# number of motots
NO_OF_MOTORS = 6
################################

################################
# NN Constants 
NN_ACTION_INTERVAL = 0.002

# actuator constants
STIFFNESS = 20
DAMPING   = 0.167

# motor link length
# TO-DO - measure actual radius 
FIXED_LINKS_LENGTH = (0.2, 0.2, 0.32, 0.32, 0.24, 0.24)

# max od torque
ODRIVE_SET_MIN_TORQUE = -1.2 
ODRIVE_SET_MAX_TORQUE = 1.2 
################################

#Joint names: ['left_ab_ad_joint', 'right_ab_ad_joint', 'left_hip_joint', 'right_hip_joint', 'left_knee_joint', 'right_knee_joint', 'left_ankle_joint', 'right_ankle_joint']

################################
# Motor State
class MOTOR_STATE (Enum):
    INITIALIZED = 0x01
    STOPPED     = 0x02

class motor_queue:

    def __init__(self, maxsize: int = 10):
        self.motor_1  = queue.Queue(maxsize=maxsize)
        self.motor_2  = queue.Queue(maxsize=maxsize)
        self.motor_3  = queue.Queue(maxsize=maxsize)
        self.motor_4  = queue.Queue(maxsize=maxsize)
        self.motor_5  = queue.Queue(maxsize=maxsize)
        self.motor_6  = queue.Queue(maxsize=maxsize)

    def save_to_queue(self, motor_id: int, data):
        try:
            queue_attr = getattr(self, f"motor_{motor_id}")
            
            if queue_attr.full():
                self.clear_queues()  # Clear all queue for sync data

            queue_attr.put(data)  # Add the new item
        except AttributeError:
            raise ValueError("Invalid motor ID")

    def wait_until_filled(self):
        while self.motor_1.empty() or self.motor_2.empty() or self.motor_3.empty() or self.motor_4.empty() or self.motor_5.empty() or self.motor_6.empty():
            pass  # Busy-waiting until both queues have data

        return self.motor_1.get(), self.motor_2.get(), self.motor_3.get(), self.motor_4.get(), self.motor_5.get(), self.motor_6.get() 

    def clear_queues(self):
        for motor_queue in [self.motor_1, self.motor_2, self.motor_3,
                            self.motor_4, self.motor_5, self.motor_6]:
            while not motor_queue.empty():
                motor_queue.get()

################################
class MotorControl:
    def __init__(self, pos_nn_q, state_request_queue,pos_rl_q,vel_rl_q, bus):
        """Initialize motor controller."""
        self.bus = bus
        self.state_request_queue = state_request_queue 

        self.pos_nn_q = pos_nn_q
        self.pos_rl_q = pos_rl_q
        self.vel_rl_q = vel_rl_q

        self.prv_target_pos = (0,0,0,0,0,0)
        self.prv_torque     = 0

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
        # self.position_data = deque(maxlen=2000)  # Store last 100 positions
        # self.start_time = None  # Capture start time in milliseconds
        # self.time_interval = 100  # Time interval (e.g., 100 ms per update)
        # self.fig, self.ax = plt.subplots()
        # self.line, = self.ax.plot([], [], 'bo-', label="Motor Position")

        # self.ax.set_xlabel("Time (ms)")
        # self.ax.set_ylabel("Position")
        # self.ax.set_title("Real-Time Knee Motor Position")
        # self.ax.legend()
        # self.ax.grid(True)
        # #plt.ion() 
        # plt.show()

        #print(f"Motor Active")

    # def send_position(self, id, position):
    #     print(f"{id}:{position}")
    #     self.bus.send(can.Message(
    #         arbitration_id=(id << 5 | 0x0c),  # 0x0c: Set_Input_Pos
    #         data=struct.pack('<f', position),
    #         is_extended_id=False
    #     ))

    # def send_torque(self, motors_torque):

    #     for index,torque in enumerate(motors_torque) :  
    #         print(f" sent torque = {index}:{torque}")      
    #         bus.send(can.Message(
    #             arbitration_id=(index << 5 | 0x0e),  # 0x0e: Set_Input_Torque
    #             data=struct.pack('<f', torque),
    #             is_extended_id=False
    #         ))

    def send_position(
        self,
        position: tuple = (0, 0, 0, 0, 0, 0),
        velocity_feedforward: tuple = (0, 0, 0, 0, 0, 0),
        torque_feedforward: tuple = (0, 0, 0, 0, 0, 0)
    ):
        for index in range(NO_OF_MOTORS):            
            try:
                self.bus.send(can.Message(
                    arbitration_id=(index << 5 | 0x0C),  # Uses correct motor ID
                    data=struct.pack('<fff', position[index], velocity_feedforward[index], torque_feedforward[index]),
                    is_extended_id=False
                ))
            except can.CanError as e:
                print(f"Failed to send CAN message for motor {index}: {e}")

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

            # note = 
            # pos_nn_q should be slower than both pos_rl_q and vel_rl_q

            # are all motor queue filled?
            # then extract all of them and save in to a variable            
            target_pos = self.pos_nn_q.wait_until_filled() # only has 1 queue size

            # motor needs to be started
            if self.motor_state == MOTOR_STATE.STOPPED:
                print(f"Motors havn't been initialied yet")
                continue

            # get current position and vel          
            cur_pos = self.pos_rl_q.wait_until_filled() # only has 1 queue size
            cur_vel = self.vel_rl_q.wait_until_filled() # only has 1 queue size

            # calculate pos error
            pos_error = target_pos - cur_pos

            # calculate vel error
            # calculate nn velocity 
            target_vel = ( target_pos - self.prv_target_pos ) / NN_ACTION_INTERVAL
            vel_error  = target_vel - cur_vel

            # then calculate torque            
            # from isaaclab -- 
            # error_pos = control_action.joint_positions  - joint_pos
            # error_vel = control_action.joint_velocities - joint_vel
            # # calculate the desired joint torques
            # self.computed_effort = self.stiffness * error_pos + self.damping * error_vel + control_action.joint_efforts
            torque = STIFFNESS*pos_error + DAMPING*vel_error + self.prv_torque

            # translate torque to real
            # torque_nn = Force * radius - from motor torque
            # rlp of motor and torque at 0.24m 
            # F1 = 7468.5*Torque_real - 71.897
            # F * Rnew/0.24 = 7468.5*Torque_real - 71.897
            force_at_24 = torque / FIXED_LINKS_LENGTH
            force_at_link = (FIXED_LINKS_LENGTH/0.24) * force_at_24
            odrive_torque = ( force_at_link + 71.897 ) / 7468.5
        
            # apply cliping to get max torque
            odrive_torque =  max(ODRIVE_SET_MIN_TORQUE, min(odrive_torque, ODRIVE_SET_MAX_TORQUE))

            # calculate pos_rl 
            # if ( motor_id   == 1 or motor_id == 4 ) :       # knee
            #     position = max(-2.67, min(position, 2.67))  # Clamp within [-2.7, 2.7] # 99%
            #     position = ( position + KNEE_OFFSET ) * KNEE_FACTOR 
            # elif ( motor_id == 2 or motor_id == 5 ):      # hip
            #     position = max(-2.27, min(position, 2.27))  # Clamp within [-2.3, 2.3]
            #     position = ( position - HIP_OFFSET ) * HIP_FACTOR
            # elif ( motor_id == 3 or motor_id == 6 ):        #abad
            #     position = max(-0.43, min(position, 0.43))  # Clamp within [-0.44, 0.44]
            #     position = ( position + ABAD_OFFSET ) * ABAD_FACTOR
            # else :
            #     print(f"Motor_{motor_id} position {position} too large")
            #     continue;
            # assuming target_pos is already limited when send?
            target_pos = ( target_pos + URDF_TO_REAL_POS_OFFSET ) * URDF_TO_REAL_POS_FACTOR

            # translate to the odrive torque and send
            #self.send_position(pos_rl,vel_rl,torque_rl)
            #self.send_position(target_pos,target_vel,odrive_torque)
            print(f"target_pos,target_vel,odrive_torque = {target_pos},{target_vel},{odrive_torque}")


            self.prv_target_pos = target_pos
            self.prv_torque     = torque

              
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

    # def update_plot(self):
    #     """Updates the live plot using real-time elapsed time."""
    #     if len(self.position_data) > 0:  # Ensure we have data
    #         time_values, position_values = zip(*self.position_data)  # Separate time & position

    #         self.line.set_data(time_values, position_values)

    #         self.ax.relim()
    #         self.ax.autoscale_view()
    #         self.ax.set_xlim(max(0, time_values[-1] - 5000), time_values[-1] + 1000)  # Show last 5 sec

    #         plt.draw()
    #         self.fig.canvas.flush_events()  # Force redraw


class MotorListener(Node):
    """ROS 2 Node that subscribes to six motor topics and saves data in a queue."""
    
    def __init__(self, pos_nn_q,state_request_queue,pos_rl_q,vel_rl_q):
        super().__init__('motor_cmd_listener')

        self.pos_nn_q = pos_nn_q
        self.pos_rl_q = pos_rl_q
        self.vel_rl_q = vel_rl_q

        self.state_request_queue = state_request_queue

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
            self.create_subscription(Float32, topic, lambda msg, m_id=idx: self.listener_callback(msg, m_id), 10)

        for idx, topic in enumerate(self.motor_current_position, start=1):
            self.create_subscription(Float32, topic, lambda msg, m_id=idx: self.current_pos_callback(msg, m_id), 10)

        for idx, topic in enumerate(self.motor_current_velocity, start=1):
            self.create_subscription(Float32, topic, lambda msg, m_id=idx: self.current_vel_callback(msg, m_id), 10)

        # Subscription for /all_motor/cmd with String commands
        self.create_subscription(String, self.all_motor_cmd_topic, self.all_motor_callback, 3)

    def listener_callback(self, msg, motor_id):
        """Stores received motor position data in the queue."""
        self.pos_nn_q.save_to_queue(motor_id, msg.data)

    def current_pos_callback(self, msg, motor_id):
        self.pos_rl_q.save_to_queue(motor_id, msg.data)

    def current_vel_callback(self, msg, motor_id):
        self.vel_rl_q.save_to_queue(motor_id, msg.data)

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

    pos_nn_q = motor_queue(maxsize=1)
    pos_rl_q = motor_queue(maxsize=1)
    vel_rl_q = motor_queue()

    state_request_queue = queue.Queue()

    listener_node = MotorListener(pos_nn_q,state_request_queue,pos_rl_q,vel_rl_q)
    listener_thread = threading.Thread(target=rclpy.spin, args=(listener_node,), daemon=True)
    listener_thread.start()

    # motor_control = MotorControl(position_queue, bus)
    # motor_thread = threading.Thread(target=motor_control.run, daemon=True)
    # motor_thread.start()
    motor_control = MotorControl(pos_nn_q, state_request_queue,pos_rl_q,vel_rl_q, bus)

    try:
        listener_thread.join()  # Keep ROS 2 listener running
    except KeyboardInterrupt:
        listener_node.destroy_node()
        rclpy.shutdown()
        motor_control.stop()  # Stop motor threads safely
        listener_thread.join()

if __name__ == '__main__':
    main()