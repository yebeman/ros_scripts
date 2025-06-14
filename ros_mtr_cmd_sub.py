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
import numpy as np

# plot
# import matplotlib.pyplot as plt
# from collections import deque

##############################
# URDF  <-> Encoder Conversion Factor 
# knee 
# -2.7  <--> 2.7  = 5.4   => sr
# 2.7   <--> -2.7 = 5.4   => sl
# 0.00  <--> 7.06 = 7.06  => r
KNEE_FACTOR = 7.06/5.4
KNEE_OFFSET = 3.53
# Hip
# -2.3  <--> 2.3    = 4.6  => sr
# 2.3   <--> -2.3   = 4.6  => sl
# 0.00  <--> -5.36  = 5.36 => r
HIP_FACTOR  = -5.36/4.6
HIP_OFFSET  = -2.68
# ABAD
# -0.44  <--> 0.44  = .88  => sr
# 0.44   <--> -0.44 = .88  => sl
# -0.68  <--> 0.68  = 1.36 => r
ABAD_FACTOR = 1.36/0.88
ABAD_OFFSET = 0

# position conversion 
URDF_TO_REAL_POS_FACTOR =  (-1*KNEE_FACTOR,-1*HIP_FACTOR,ABAD_FACTOR,KNEE_FACTOR,HIP_FACTOR,-1*ABAD_FACTOR)
URDF_TO_REAL_POS_OFFSET =  (   KNEE_OFFSET,   HIP_OFFSET,ABAD_OFFSET,KNEE_OFFSET,HIP_OFFSET,-1*ABAD_OFFSET)
# number of motots
NO_OF_MOTORS = 3
################################

################################
# NN Constants 
NN_ACTION_INTERVAL = 0.002

# actuator constants
STIFFNESS = 20
DAMPING   = 0.167

# motor link length
# TO-DO - measure actual radius 
# (0.09^2+(0.23+0.245)^2)^1/2 = 
FIXED_LINKS_LENGTH = np.array((0.23, 0.23+0.245,0.483,0.23,0.23+0.245,0.483))

# max od torque
ODRIVE_SET_MIN_TORQUE = -1.145 
ODRIVE_SET_MAX_TORQUE = 1.145

# max velocity 
ODRIVE_MAX_VEL = 7.0
ODRIVE_MIN_VEL = -7.0
################################

#Joint names: ['left_ab_ad_joint', 'right_ab_ad_joint', 'left_hip_joint', 'right_hip_joint', 'left_knee_joint', 'right_knee_joint', 'left_ankle_joint', 'right_ankle_joint']
#motor names:      motor_3       ,     motor_6        ,     motor_2     ,     motor_5      ,     motor_1      ,      motor_4      ,         NA               NA

################################
# Motor State
class MOTOR_STATE (Enum):
    INITIALIZED = 0x01
    STOPPED     = 0x02

class motor_queue:

    def __init__(self, maxsize: int = 1):
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
                return #self.clear_queues() 

            queue_attr.put(data)  # Add the new item
        except AttributeError:
            raise ValueError("Invalid motor ID")

    def wait_until_filled(self):

        start_time = time.monotonic() 

        while self.motor_1.empty() or self.motor_2.empty() or self.motor_3.empty() or self.motor_4.empty() or self.motor_5.empty() or self.motor_6.empty():

            if time.monotonic() - start_time > 0.1: 

                return None  

        return np.array((self.motor_1.get(), self.motor_2.get(), self.motor_3.get(), 
                         self.motor_4.get(), self.motor_5.get(), self.motor_6.get()))

    def is_filled(self):

        # check if all is filled - sync all motors
        if self.motor_1.empty() or self.motor_2.empty() or self.motor_3.empty() or self.motor_4.empty() or self.motor_5.empty() or self.motor_6.empty():
            return None  

        return np.array((self.motor_1.get(), self.motor_2.get(), self.motor_3.get(), 
                         self.motor_4.get(), self.motor_5.get(), self.motor_6.get()))

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

        self.prv_pos_nn  = np.array((0, 0, 0, 0, 0, 0))
        self.prv_cur_pos = np.array((0, 0, 0, 0, 0, 0))
        self.prv_cur_vel = np.array((0, 0, 0, 0, 0, 0))
        self.prv_torque  = np.array((0, 0, 0, 0, 0, 0))

        self.running = True
        self.motor_state = MOTOR_STATE.STOPPED

        # self.command_thread.start()
        self.process_commands()

    def send_torque(self, motors_torque: np.array((0, 0, 0, 0, 0, 0))):

        print(f"motors_torque={motors_torque.tolist()}")

        # try:
        #     for index,torque in enumerate(motors_torque) :  
        #         print(f" sent torque = {index}:{torque}") 
        mtr_id = 1    
        self.bus.send(can.Message(
            arbitration_id=(mtr_id << 5 | 0x0e),  # 0x0e: Set_Input_Torque
            data=struct.pack('<f', motors_torque[0]),#torque),
            is_extended_id=False
        ))
        # except OSError as e:
        #     print(f"CAN send failed: {e}")

    def send_position(
        self,
        position: np.array((0, 0, 0, 0, 0, 0)),
        velocity_feedforward: np.array((0, 0, 0, 0, 0, 0)),
        torque_feedforward: np.array((0, 0, 0, 0, 0, 0))
    ):
        #mtr_id = 1
        print(f"position={position.tolist()}, \nvelocity_feedforward={velocity_feedforward.tolist()}, \ntorque_feedforward={torque_feedforward.tolist()}")
        # try:
        #     self.bus.send(can.Message(
        #         arbitration_id=(mtr_id << 5 | 0x0C),
        #         data=struct.pack('<fhh', position[0], int(velocity_feedforward[0] * 1000), int(torque_feedforward[0] * 1000)),
        #         is_extended_id=False
        #     ))
        # except OSError as e:
        #     print(f"CAN send failed: {e}")
        # for index in range(NO_OF_MOTORS):            
        #     try:
        #         self.bus.send(can.Message(
        #             arbitration_id=((index+1) << 5 | 0x0C),
        #             data=struct.pack('<fhh', position[index], int(velocity_feedforward[index] * 1000), int(torque_feedforward[index] * 1000)),
        #             is_extended_id=False
        #         ))
        #     except can.CanError as e:
        #         print(f"Failed to send CAN message for motor {index + 1}: {e}")

    def init_motor(self,mtr_id):
        print(f"Starting motor {mtr_id}")

        if (mtr_id > 1):
            return

        self.bus.send(can.Message(
            arbitration_id=(mtr_id << 5 | 0x07),
            data=struct.pack('<I', 8),
            is_extended_id=False
        ))

    def stop_motor(self,mtr_id):
        print(f"Stopping motor {mtr_id}")

        if (mtr_id > 1):
            return

        self.bus.send(can.Message(
            arbitration_id=(mtr_id << 5 | 0x07),
            data=struct.pack('<I', 1), # disable
            is_extended_id=False
        ))

    def process_positions(self):

        print("Processing positions ...")
        #cmd_sample_time = time.monotonic()

        # max pid odrive update frequency apparently  runs at 8KHz ~ 0.125ms position/velocity/current control loops
        # starting with 5ms update frequ
        cmd_request_period = time.monotonic()
        while self.running:

            if not self.state_request_queue.empty():

                command = self.state_request_queue.get() 

                if command == "stop" and self.motor_state == MOTOR_STATE.INITIALIZED:

                    for id in range(1,7):
                        self.stop_motor(id)

                    self.motor_state = MOTOR_STATE.STOPPED
                    break
                else :
                    print("command not recognized")



            # motor needs to be started
            if self.motor_state == MOTOR_STATE.STOPPED:
                print(f"Motors havn't been initialied yet")
                break

            # note = 
            # pos_nn_q should be slower than both pos_rl_q and vel_rl_q

            # are all motor queue filled?
            # then extract all of them and save in to a variable         
            # if queue has data (without wait), take, else use previous data  and recalculate 
            pos_nn = self.pos_nn_q.is_filled() # only has 1 queue size; NN size limit ; motor 1 - 6 , [rad]
            if pos_nn is None:
                pos_nn = self.prv_pos_nn

            # get current position and vel  
            # if queue has data (without wait), take, else use previous data  and recalculate 
            cur_pos = self.pos_rl_q.is_filled() # only has 1 queue size  ; NN size limit  , [rad]
            if cur_pos is None:
                cur_pos = self.prv_cur_pos

            # if queue has data (without wait), take, else use previous data and recalculate 
            cur_vel = self.vel_rl_q.is_filled() # only has 1 queue size  ; NN size limit  , [rad/s]
            if cur_vel is None:
                cur_vel = self.prv_cur_vel

            # calculate pos error
            pos_error = pos_nn - cur_pos

            # calculate vel error
            # calculate nn velocity 
            # elapsed_time = time.monotonic() - cmd_sample_time
            # elapsed_time = np.where(elapsed_time <= 0, 1, elapsed_time)
            # target_vel = ( pos_nn - self.prv_pos_nn ) / elapsed_time
            # cmd_sample_time = time.monotonic()

            # np.clip(target_vel, ODRIVE_MIN_VEL, ODRIVE_MAX_VEL)  # clip 
            target_vel = np.array((0, 0, 0, 0, 0, 0)) # isaacsim has this to be 0
            vel_error  = target_vel - cur_vel

            # then calculate torque            
            # from isaaclab -- 
            # error_pos = control_action.joint_positions  - joint_pos
            # error_vel = control_action.joint_velocities - joint_vel
            # # calculate the desired joint torques
            # self.computed_effort = self.stiffness * error_pos + self.damping * error_vel + control_action.joint_efforts
            print(f"pos_nn = {pos_nn.tolist()}")
            print(f"cur_pos = {cur_pos.tolist()}")
            print(f"pos_error = {pos_error.tolist()}")
            print(f"cur_vel = {cur_vel.tolist()}")
            print(f"vel_error = {vel_error.tolist()}")
            torque = STIFFNESS*pos_error + DAMPING*vel_error + self.prv_torque
            print(f"torque= {torque.tolist()}")

            # translate torque to real
            # torque_nn = Force * radius - from motor torque
            # rlp of motor and torque at 0.24m 
            # F1 = 7468.5*Torque_real - 71.897
            # F * Rnew/0.24 = 7468.5*Torque_real - 71.897
            #force_at_24 = np.abs(torque) / FIXED_LINKS_LENGTH
            _torque = (FIXED_LINKS_LENGTH/FIXED_LINKS_LENGTH[0]) * np.abs(torque) # take -ve 
            _mass_in_gram = _torque/(9.98) * 1000 # kg to gram
            odrive_torque = ( _mass_in_gram + 71.897 ) / 7468.5
            odrive_torque = odrive_torque * (torque/np.where(torque == 0,1,np.abs(torque))) # apply the -ve back

            # apply cliping to get max torque
            #odrive_torque =  max(ODRIVE_SET_MIN_TORQUE, min(odrive_torque, ODRIVE_SET_MAX_TORQUE))
            odrive_torque = np.clip(odrive_torque, ODRIVE_SET_MIN_TORQUE, ODRIVE_SET_MAX_TORQUE)


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
            # print(f"pos_nn = {pos_nn.tolist()}")
            #target_pos =  URDF_TO_REAL_POS_FACTOR * pos_nn + URDF_TO_REAL_POS_OFFSET
            # print(f"target_pos = {target_pos.tolist()}")
            #print(f"cur_pos = {cur_pos.tolist()}")

            # odrive takes velocity command rev/s
            target_vel = target_vel / (2 *np.pi) 

            # translate to the odrive torque and send
            #self.send_position(pos_rl,vel_rl,torque_rl)
            #self.send_position(target_pos,target_vel,odrive_torque)
            self.send_torque(odrive_torque)
            # print(f"\nPos NN = {pos_nn.tolist()} \nTarget Pos = {target_pos.tolist()} \nTarget Vel = {target_vel.tolist()} \nODrive Torque = {odrive_torque.tolist()}")
            # print(f"cur_pos = {cur_pos.tolist()} \ncur_vel = {cur_vel.tolist()} \npos_error = {pos_error.tolist()} \nvel_error = {vel_error.tolist()}")
            # print(f"torque = {torque.tolist()} \nforce_at_link = {force_at_link.tolist()} \nelapsed_time={elapsed_time}")

            self.prv_pos_nn  = pos_nn
            self.prv_cur_vel = cur_vel
            self.prv_cur_pos = cur_pos
            self.prv_torque  = np.array((0, 0, 0, 0, 0, 0)) #isaaclab apperently sets this to 0 during calc torque

            # update at 5ms frequency 
            cmd_elapsed_period = 0.005 - (time.monotonic() - cmd_request_period) # should take 5ms 
            print(f"cmd_elapsed_period = {cmd_elapsed_period} and elapsed = {(time.monotonic() - cmd_request_period)}")
            if (cmd_elapsed_period > 0) : 
                time.sleep(cmd_elapsed_period)
            cmd_request_period = time.monotonic()
        
        # if its still running 
        if self.running:
            self.process_commands()

    def process_commands(self):

        print("Waiting for Start command ...")
        
        command = self.state_request_queue.get()

        if command == "start" and self.motor_state == MOTOR_STATE.STOPPED:
            
            for id in range(1,7):
                self.init_motor(id)

            self.motor_state = MOTOR_STATE.INITIALIZED
            
            self.process_positions() # call positions processor

    def stop(self):
        self.running = False
        self.motor_thread.join()
        self.command_thread.join()


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

        print("Listening...")

    def listener_callback(self, msg, motor_id):
        """Stores received motor position data in the queue."""
        self.pos_nn_q.save_to_queue(motor_id, msg.data)

        #for now Zero everything else
        # do it only once
        if motor_id == 1:   
            self.pos_nn_q.save_to_queue(2, 0.0)
            self.pos_nn_q.save_to_queue(3, 0.0)
            self.pos_nn_q.save_to_queue(4, 0.0)
            self.pos_nn_q.save_to_queue(5, 0.0)
            self.pos_nn_q.save_to_queue(6, 0.0)

    def current_pos_callback(self, msg, motor_id):
        self.pos_rl_q.save_to_queue(motor_id, msg.data)

        # for now Zero everything else
        # do it only once
        if motor_id == 1:   
            self.pos_rl_q.save_to_queue(2, 0.0)
            self.pos_rl_q.save_to_queue(3, 0.0)
            self.pos_rl_q.save_to_queue(4, 0.0)
            self.pos_rl_q.save_to_queue(5, 0.0)
            self.pos_rl_q.save_to_queue(6, 0.0)

    def current_vel_callback(self, msg, motor_id):
        self.vel_rl_q.save_to_queue(motor_id, msg.data)

        # for now Zero everything else
        # do it only once
        if motor_id == 1:   
            self.vel_rl_q.save_to_queue(2, 0.0)
            self.vel_rl_q.save_to_queue(3, 0.0)
            self.vel_rl_q.save_to_queue(4, 0.0)
            self.vel_rl_q.save_to_queue(5, 0.0)
            self.vel_rl_q.save_to_queue(6, 0.0)


    def all_motor_callback(self, msg):
        """Handles start/stop commands for all motors."""
        command = msg.data.strip().lower()  # Normalize command input
        self.state_request_queue.put(command)


def main(args=None):
    bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
    
    rclpy.init(args=args)

    pos_nn_q = motor_queue(maxsize=1)
    pos_rl_q = motor_queue(maxsize=1)
    vel_rl_q = motor_queue(maxsize=1)

    state_request_queue = queue.Queue(maxsize=1)

    listener_node = MotorListener(pos_nn_q,state_request_queue,pos_rl_q,vel_rl_q)
    listener_thread = threading.Thread(target=rclpy.spin, args=(listener_node,), daemon=True)
    listener_thread.start()

    motor_control = MotorControl(pos_nn_q, state_request_queue,pos_rl_q,vel_rl_q, bus)

    try:
        listener_thread.join()  # Keep ROS 2 listener running
    except KeyboardInterrupt:
        listener_node.destroy_node()
        rclpy.shutdown()
        # motor_control.stop()  # Stop motor threads safely
        listener_thread.join()

if __name__ == '__main__':
    main()