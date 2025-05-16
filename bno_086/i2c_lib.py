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
       
        #print(f"writeto before ", {bytes(data)})


        #print(type(address), type(s_data), type(len(s_data)), type(0))

        #print(f"Writing ---: {address:#04x}, memaddr: {memaddr:#04x}")
        self.i2c.send(address, bytes(data),len(data),0)
        
        #print(f"writeto after", {bytes(data)})


    def readfrom_into(self, address, buffer, in_start=0, in_end=None):
        
        _ffi = cffi.FFI()
        rx_buf = _ffi.new("unsigned char [512]")

        #print(f"readfrom_into before ")

        
        self.i2c.receive(address, rx_buf, len(buffer),0)
        
        #print(f"readfrom_into after", {bytes(rx_buf)})
        
        for i in range(len(buffer)):
            buffer[i] = rx_buf[i]
            
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
