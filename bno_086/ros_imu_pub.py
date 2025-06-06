from pynq import Overlay
import threading

import math
from bno08x import *
from i2c_lib import *
import time
import numpy as np
import queue

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#
#10 -- 90ms
#20 -- 60ms
#30 -- 20ms
#40 -- consistent 20ms - max seen 0.038250 / no repeats seen
#50 -- 

class BNO086Data:
    """A shared data structure to hold IMU readings with timestamp."""
    def __init__(self):
        self.timestamp = time.time()
        self.lin_accel = (0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.gravity = (0.0, 0.0, 0.0)
        self.lock = threading.Lock()  # Thread safety

    def update(self, lin_accel, gyro, gravity):
        """Updates IMU readings with timestamp."""
        with self.lock:
            self.timestamp = time.time()
            self.lin_accel = lin_accel
            self.gyro = gyro
            self.gravity = gravity

    def read(self):
        """Reads IMU data safely."""
        with self.lock:
            return self.timestamp, self.lin_accel, self.gyro, self.gravity

class IMUDataGenerator(Node):
    """ROS 2 Node that publishes IMU data at 100Hz to separate topics."""
    def __init__(self, shared_data):
        super().__init__('imu_publisher')
        self.shared_data = shared_data

        # Create publishers for each IMU topic
        self.gyro_pub    = self.create_publisher(String, 'imu_gyro', 10)
        self.gravity_pub = self.create_publisher(String, 'imu_gravity', 10)
        self.lin_acc     = self.create_publisher(String, 'imu_lin_acc', 10)

        self.timer = self.create_timer(0.01, self.publish_imu_data)  # 100 Hz

    def publish_imu_data(self):
        """Publishes IMU data to separate topics."""
        gravity, lin_acc, gyro = self.shared_data.get()

        # normalize gravity
        gravity = gravity/np.linalg.norm(gravity)

        lin_acc_x, lin_acc_y, lin_acc_z = lin_acc
        gravity_x, gravity_y, gravity_z = gravity
        gyro_x, gyro_y, gyro_z = gyro

        self.gyro_pub.publish(String(data=f"{gyro_x},{gyro_y},{gyro_z}"))
        self.gravity_pub.publish(String(data=f"{gravity_x},{gravity_y},{gravity_z}"))
        self.lin_acc.publish(String(data=f"{lin_acc_x},{lin_acc_y},{lin_acc_z}"))

        # print("Lin Acceleration\tX: {:+.3f}\tY: {:+.3f}\tZ: {:+.3f}\tm/s²".format(accel_x, accel_y, accel_z))
        # print("Gravity\tX: {:+.3f}\tY: {:+.3f}\tZ: {:+.3f}\trads/s".format(gravity_x, gravity_y, gravity_z))
        #print("Gyroscope\tX: {:+.3f}\tY: {:+.3f}\tZ: {:+.3f}\trads/s".format(gyro_x, gyro_y, gyro_z))


def retrieve_imu(shared_data, imu):
    """Continuously updates IMU data at 100Hz."""
    origin_time = time.monotonic() #ticks_ms()  

    while True:

        start_time = time.monotonic() #ticks_ms()

        gravity, lin_acc, gyro = imu.gravity_linacc_gyro

        # Store each category in separate tuples
        lin_acc_tuple = tuple(lin_acc)  # (lin_acc_x, lin_acc_y, lin_acc_z)
        gravity_tuple = tuple(gravity)  # (gravity_x, gravity_y, gravity_z)
        gyro_tuple = tuple(gyro)        # (gyro_x, gyro_y, gyro_z)

        # Put the three tuples into the shared_data queue
        shared_data.put((lin_acc_tuple, gravity_tuple, gyro_tuple))

        print(f"every {time.monotonic() - origin_time:.6f}")
        origin_time = time.monotonic() #ticks_ms()
        elapsed_time = time.monotonic() - start_time
        sleep_time = max(0, 0.021 - elapsed_time)  #  no negative number
        time.sleep(sleep_time)

def main(args=None):


    overlay = Overlay('/home/yebe/ros2_scripts/multi_ped.bit')
    time.sleep(1)

    i2c = multi_biped_i2c(overlay)
    imu = BNO08X(i2c, debug=False, address=0x4B)

    # imu.enable_feature(BNO_REPORT_ACCELEROMETER, 50)
    imu.enable_feature(BNO_REPORT_LINEAR_ACCELERATION,50 )
    imu.enable_feature(BNO_REPORT_GYROSCOPE,50 )
    imu.enable_feature(BNO_REPORT_GRAVITY, 50)
    print("BNO08x sensors enabling : Done\n")

    rclpy.init(args=args)
    shared_data = queue.Queue() #BNO086Data()

    # Start IMU retrieval in background thread
    generator_thread = threading.Thread(target=retrieve_imu, args=(shared_data,imu), daemon=True)
    generator_thread.start()

    # Start ROS 2 publisher node
    node = IMUDataGenerator(shared_data)
    rclpy.spin(node)

    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()