from pynq import Overlay
import threading

import math
from bno08x import *
from i2c_lib import *
import time

overlay = Overlay('/home/yebe/ros2_scripts/multi_ped.bit')
time.sleep(1)


i2c = multi_biped_i2c(overlay)
imu = BNO08X(i2c, debug=True, address=0x4B)

imu.enable_feature(BNO_REPORT_ACCELEROMETER, 20)
imu.enable_feature(BNO_REPORT_LINEAR_ACCELERATION,20 )
imu.enable_feature(BNO_REPORT_GYROSCOPE,20 )
imu.enable_feature(BNO_REPORT_GRAVITY, 10)
# imu.set_quaternion_euler_vector(BNO_REPORT_GAME_ROTATION_VECTOR)

print("BNO08x sensors enabling : Done\n")

cpt = 0
timer_origin = time.monotonic() #ticks_ms()
average_delay = -1

while True:
    #time.sleep(0.5)
    # cpt += 1
    # print("cpt", cpt)
 
    accel_x, accel_y, accel_z = imu.acc
    print("Acceleration\tX: {:+.3f}\tY: {:+.3f}\tZ: {:+.3f}\tm/s²".format(accel_x, accel_y, accel_z))
 
    # gyro_x, gyro_y, gyro_z = imu.gyro
    # print("Gyroscope\tX: {:+.3f}\tY: {:+.3f}\tZ: {:+.3f}\trads/s".format(gyro_x, gyro_y, gyro_z))
 
    # # lin_accel_x, lin_accel_y, lin_accel_z = imu.acc_linear
    # print("Linear Acceleration\tX: {:+.3f}\tY: {:+.3f}\tZ: {:+.3f}\tm/s²".format(lin_accel_x, lin_accel_y, lin_accel_z))
 
    # gravity_x, gravity_y, gravity_z = imu.gravity
    # print("Gravity\tX: {:+.3f}\tY: {:+.3f}\tZ: {:+.3f}\tm/s²".format(gravity_x, gravity_y, gravity_z))

    time.sleep(0.1)
    #clear_output(wait=True)  # Clears the previous output

