from pynq import Overlay
import threading

import math
from bno08x import *
from i2c_lib import *
import time
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


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
        self.gyro_pub = self.create_publisher(String, 'imu_gyro', 10)
        self.gravity_pub = self.create_publisher(String, 'imu_gravity', 10)
        self.lin_accel_pub = self.create_publisher(String, 'imu_lin_accel', 10)

        self.timer = self.create_timer(0.01, self.publish_imu_data)  # 100 Hz

    def publish_imu_data(self):
        """Publishes IMU data to separate topics."""
        timestamp, lin_accel, gyro, gravity = self.shared_data.read()

        self.gyro_pub.publish(String(data=f"{gyro[0]},{gyro[1]},{gyro[2]}"))
        self.gravity_pub.publish(String(data=f"{gravity[0]},{gravity[1]},{gravity[2]}"))
        self.lin_accel_pub.publish(String(data=f"{lin_accel[0]},{lin_accel[1]},{lin_accel[2]}"))

        print("Gyroscope\tX: {:+.3f}\tY: {:+.3f}\tZ: {:+.3f}\trads/s".format(gyro[0], gyro[1], gyro[2]))
        #print("Gravity\tX: {:+.3f}\tY: {:+.3f}\tZ: {:+.3f}\trads/s".format(gravity[0], gravity[1], gravity[2]))


def retrieve_imu(shared_data, imu):
    """Continuously updates IMU data at 100Hz."""
    while True:
        lin_accel = imu.acc_linear


        gyro = imu.gyro

        # normalize gravity
        gravity = imu.gravity
        gravity = gravity/np.linalg.norm(gravity)


        shared_data.update(lin_accel, gyro, gravity)
        time.sleep(0.01)  # 100 Hz 

def main(args=None):


    overlay = Overlay('/home/yebe/ros2_scripts/multi_ped.bit')
    time.sleep(1)

    i2c = multi_biped_i2c(overlay)
    imu = BNO08X(i2c, debug=False, address=0x4B)

    imu.enable_feature(BNO_REPORT_ACCELEROMETER, 20)
    imu.enable_feature(BNO_REPORT_LINEAR_ACCELERATION,20 )
    imu.enable_feature(BNO_REPORT_GYROSCOPE,20 )
    imu.enable_feature(BNO_REPORT_GRAVITY, 10)
    print("BNO08x sensors enabling : Done\n")

    rclpy.init(args=args)
    shared_data = BNO086Data()

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