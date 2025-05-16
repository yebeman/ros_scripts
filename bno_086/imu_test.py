from pynq import Overlay
import threading

import math
from bno08x import *
from i2c_lib import *
import cffi
import time

overlay = Overlay('/home/yebe/ros2_scripts/multi_ped.bit')
time.sleep(1)


i2c = multi_biped_i2c(overlay)
imu = BNO08X(i2c, debug=True, address=0x4B)


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
    # # mag_x, mag_y, mag_z = imu.mag
    # print("Magnetometer\tX: {:+.3f}\tY: {:+.3f}\tZ: {:+.3f}\tuT".format(mag_x, mag_y, mag_z))
    # quat_i, quat_j, quat_k, quat_real = imu.quaternion
    # print("Rot Vect Quat\tI: {:+.3f}\tJ: {:+.3f}\tK: {:+.3f}\tReal: {:+.3f}".format(quat_i, quat_j, quat_k, quat_real))
    #R, T, P = imu.euler
    #print("Euler Angle\tX: {:+.3f}\tY: {:+.3f}\tZ: {:+.3f}".format(R, T, P))
    # print("===================================")
    # print("average delay times (ms) :", average_delay)
    # print("===================================")
    # timer = time.monotonic()
    # if cpt == 10 :
    #     imu.tare
    # if cpt % 100 == 0:
    #     average_delay = (timer - timer_origin) / cpt

    time.sleep(0.1)
    #clear_output(wait=True)  # Clears the previous output

