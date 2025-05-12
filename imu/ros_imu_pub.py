from pynq import Overlay
from pynq.lib.iic import AxiIIC
import time
import threading

import time
from bno055 import *
import cffi

import rclpy
from rclpy.node import Node

from ctypes import c_ubyte, POINTER, cast
from IPython.display import clear_output
import sys



# load fpga bitstream 
overlay = Overlay('/home/yebe/ros2_scripts/multi_ped.bit')
time.sleep(1)


#robot imu i2c interface
class multi_biped_i2c:
    def __init__(self, overlay, device_name="axi_iic_0"):
        """Initialize PYNQ I2C interface."""
        self.i2c = AxiIIC(overlay.ip_dict[device_name])
        self.lock = threading.Lock()
    
    def try_lock(self):
        """Attempt to acquire the I2C lock."""
        return self.lock.acquire(blocking=False)

    def unlock(self):
        """Release the I2C lock."""
        #self.lock.release()
        
    def writeto(self, address, data, start=0, end=None):
        """Send data using I2C communication."""
        # if self.try_lock():
        
        if (len(data) == 0) :
            data_len = 8
        else :
            data_len = len(data)
        
        
        try:
            print(address)
            print(data)
            print(data_len)
            print(f"before")
            self.i2c.send(address, data, data_len, 0)
            #self.i2c.wait()
            print(f"iam")
        finally:
            self.unlock()
        # else:
        #     print("I2C bus is busy, write operation failed.")
            
            
    def read(self, address, length):
        print(f"hi2")

        """Receive data from an I2C device."""
        received_data = []
        #if self.try_lock():
        try:
            self.i2c.receive(address, received_data, length)
            #self.i2c.wait()
        finally:
            self.unlock()
        # else:
        #     print("I2C bus is busy, read operation failed.")
        return received_data
    
    def writeto_then_readfrom(self, address, out_buffer, in_length, out_start=0, out_end=None, in_start=0, in_end=None):
        """Write data to an I2C device and immediately read the response."""
        
        # Assign default values if None is provided
        if out_end is None:
            out_end = len(out_buffer)
        if in_end is None:
            in_end = in_length
        print(f"here 1")

        # Slice output buffer according to start/end parameters
        sliced_out = list(out_buffer[out_start:out_end])
        received_data = []

        print(f"here_and")

        #if self.try_lock():
        try:
            print(f"try")
            self.iic.send(address, sliced_out, len(sliced_out), 0)
            self.iic.receive(address, received_data, in_end - in_start)                
            print(f"next")
                #self.i2c.wait()
        finally:
            self.unlock()
        # else:
        #     print("I2C bus is busy, write-read operation failed.")
        return received_data
    def readfrom_mem_into(self, address, memaddr, buf,size=1):  # memaddr = memory location within the I2C device
       
       
        # Convert bytearray to a ctypes array before passing to XIic_Send
        _ffi = cffi.FFI()
        rx_buf = _ffi.new("unsigned char [32]")
       
        self.i2c.send(address, [memaddr], 1, 1)
        self.i2c.receive(address, rx_buf,size,0)
        
        #buf = bytes(rx_buf)
        #buf[:] = rx_buf
        for i in range(size):
            buf[i] = rx_buf[i]
            
        #print(f"reading ***: {address:#04x}, memaddr: {memaddr:#04x}")
        
        # print(rx_buf[0])        
        # print(buf[0])        
        return buf  # Return the first byte of the buffer
    def writeto_mem(self, address, memaddr, buf):
        
        data = [memaddr] + list(buf)  # Ensure all data is sent
        #print(f"Writing ---: {address:#04x}, memaddr: {memaddr:#04x}")
        self.i2c.send(address, data,len(data),0)


# run
i2c = multi_biped_i2c(overlay)
imu = BNO055(i2c)

# calibrate
calibrated = False
cal_sensor_offset = bytes.fromhex('d3ff2c00f0ffffffffffffffffffffffffffffffffff')
imu.set_offsets(cal_sensor_offset)

time.sleep(0.5)



while True:
    if imu.cal_status()[0] == 3 and imu.cal_status()[1] == 3 and imu.cal_status()[2] == 3 and imu.cal_status()[3] == 3 :
        sys.stdout.write("calibrated")
        calibrated = True
        break
    
    sys.stdout.write('Calibration required: sys {} gyro {} accel {} mag {}'.format(*imu.cal_status()))

    #print(f'Gyro       x {imu.gyro()[0]:5.0f}    y {imu.gyro()[1]:5.0f}     z {imu.gyro()[2]:5.0f}')
    #print('Heading     {:4.0f} roll {:4.0f} pitch {:4.0f}'.format(*imu.euler()))
    # sys.stdout.write('Gyro      x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.gyro()))
    # sys.stdout.write('Accel     x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.accel()))
    # sys.stdout.write('Lin acc.  x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.lin_acc()))
    # sys.stdout.write('Gravity   x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.gravity()))
    # sys.stdout.write('Heading     {:4.0f} roll {:4.0f} pitch {:4.0f}'.format(*imu.euler()))
    # clear_output(wait=True)  # Clears the previous output
    time.sleep(0.01)

while True:
    if calibrated:
        print('Heading     {:4.0f} roll {:4.0f} pitch {:4.0f}'.format(*imu.euler()))
    time.sleep(0.1)
