import threading
import time
import random

class BNO055Data:
    """A shared data structure to hold IMU readings with timestamp."""
    def __init__(self):
        self.timestamp = time.time()
        self.acceleration = (0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.lock = threading.Lock()  # Ensures thread safety

    def update(self, accel, gyro):
        """Updates IMU readings with the current timestamp."""
        with self.lock:
            self.timestamp = time.time()
            self.acceleration = accel
            self.gyro = gyro

    def read(self):
        """Reads IMU data safely."""
        with self.lock:
            return self.timestamp, self.acceleration, self.gyro

class IMUDataGenerator:
    """Generates random IMU data at 100Hz."""
    def __init__(self, shared_data):
        self.shared_data = shared_data

    def run(self):
        """Simulates continuous IMU data generation."""
        while True:
            accel = (random.uniform(-2, 2), random.uniform(-2, 2), random.uniform(-2, 2))
            gyro = (random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
            self.shared_data.update(accel, gyro)
            time.sleep(0.01)  # 100 Hz

class IMUDataReader:
    """Reads IMU data continuously and prints it."""
    def __init__(self, shared_data):
        self.shared_data = shared_data

    def run(self):
        """Continuously prints IMU data with timestamps."""
        while True:
            timestamp, accel, gyro = self.shared_data.read()
            print(f"[{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(timestamp))}] Acceleration: {accel}, Gyroscope: {gyro}")
            time.sleep(0.01)  # Matches the update rate

# Initialize shared data structure
bno055_data = BNO055Data()

# Start generator and reader threads
generator = IMUDataGenerator(bno055_data)
reader = IMUDataReader(bno055_data)

threading.Thread(target=generator.run, daemon=True).start()
threading.Thread(target=reader.run, daemon=True).start()

# Keep the main script running
while True:
    time.sleep(1)
