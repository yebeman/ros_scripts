import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import threading
import queue
import numpy as np
import time

class IMU3DVisualizer(Node):
    """ROS 2 Node that subscribes to IMU topics and moves a 3D white rectangular box with a dark border."""
    def __init__(self):
        super().__init__('imu_3d_visualizer')

        # Subscribe to IMU topics
        self.create_subscription(String, 'imu_gyro', self.gyro_callback, 10)
        self.create_subscription(String, 'imu_gravity', self.gravity_callback, 10)
        self.create_subscription(String, 'imu_lin_accel', self.accel_callback, 10)

        # Queue for storing IMU data
        self.data_queue = queue.Queue()

        # Start visualization in a separate thread
        self.visualization_thread = threading.Thread(target=self.run_visualizer, daemon=True)
        self.visualization_thread.start()

    def accel_callback(self, msg):
        """Callback for linear acceleration data."""
        data = [float(x) for x in msg.data.split(',')]
        self.data_queue.put(("accel", data[0], data[1], data[2]))

    def gyro_callback(self, msg):
        """Callback for gyroscope data."""
        data = [float(x) for x in msg.data.split(',')]
        self.data_queue.put(("gyro", data[0], data[1], data[2]))

    def gravity_callback(self, msg):
        """Callback for gravity data (not used directly)."""
        data = [float(x) for x in msg.data.split(',')]
        self.data_queue.put(("gravity", data[0], data[1], data[2]))

    def draw_box(self):
        """Draws a 3D solid white rectangular box with a dark border, painting the top face blue."""
        vertices = [
            [-2, -0.5, -0.5], [2, -0.5, -0.5], [2, 1, -0.5], [-2, 0.5, -0.5],  # Back face
            [-2, -0.5,  0.5], [2, -0.5,  0.5], [2, 1,  0.5], [-2, 0.5,  0.5]   # Front face
        ]
        faces = [
            (0, 1, 2, 3), (4, 5, 6, 7),  # Front and back
            (0, 4, 7, 3), (1, 5, 6, 2),  # Left and right
            (3, 2, 6, 7), (0, 1, 5, 4)   # **Top and bottom**
        ]
        edges = [(0, 1), (1, 2), (2, 3), (3, 0),
                 (4, 5), (5, 6), (6, 7), (7, 4),
                 (0, 4), (1, 5), (2, 6), (3, 7)]

        # Draw faces
        glBegin(GL_QUADS)
        for i, face in enumerate(faces):
            if i == 4:  # **Top face (Index 4 in the faces list)**
                glColor3f(0.0, 0.0, 1.0)  # Blue color
            else:
                glColor3f(1.0, 1.0, 1.0)  # White fill

            for vertex in face:
                glVertex3fv(vertices[vertex])
        glEnd()

        # Draw dark borders
        glBegin(GL_LINES)
        glColor3f(0.2, 0.2, 0.2)  # Dark border color
        for edge in edges:
            for vertex in edge:
                glVertex3fv(vertices[vertex])
        glEnd()


    def run_visualizer(self):
        """Runs the OpenGL loop to visualize IMU data with gravity correction."""
        pygame.init()
        WIDTH, HEIGHT = 1200, 700
        pygame.display.set_mode((WIDTH, HEIGHT), DOUBLEBUF | OPENGL)
        gluPerspective(45, (WIDTH / HEIGHT), 0.1, 50.0)
        glTranslatef(0.0, 0.0, -5)

        accel_x, accel_y, accel_z = 0, 0, 0
        gyro_x, gyro_y, gyro_z = 0, 0, 0
        gravity_x, gravity_y, gravity_z = 0, 0, 0
        prev_gravity_x, prev_gravity_y, prev_gravity_z = 0.0, 0.0, 0.0  # Initialize

        loop_time = time.time()
        dt = 0.011# ms  # ~11
        position = np.array([0.0, 0.0, 0.0])  # (x, y, z) start
        velocity = np.array([0.0, 0.0, 0.0])   # (vx, vy, vz)

        # modelview_matrix_orig = glGetFloatv(GL_MODELVIEW_MATRIX)
        # print(modelview_matrix_orig)

        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Process queued IMU data
            while not self.data_queue.empty():
                data_type, x, y, z = self.data_queue.get()
                if data_type == "accel":
                    accel_x, accel_y, accel_z = x, y, z

                    print(accel_x, accel_y, accel_z, {time.time()-loop_time})
                    loop_time = time.time()

                    lin_accel = np.array([accel_x, accel_y, accel_z])


                    velocity += lin_accel * dt  # Integrate acceleration
                    position += velocity * dt      # Integrate velocity


                    print(accel_x, accel_y, accel_z, {time.time()-loop_time}, position)
                    loop_time = time.time()

                    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
                    glLoadIdentity()

                    glTranslatef(position[0], position[1], position[2])  # Apply movement

                    self.draw_box()  # Render your object


                elif data_type == "gyro":
                    gyro_x, gyro_y, gyro_z = x, y, z

                    
                    
                    # print(gyro_x, gyro_y, gyro_z, {time.time()-loop_time})
                    # loop_time = time.time()

                    # glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

                    # Apply rotation updates based on gravity orientation
                    # glRotatef(gyro_x, 1, 0, 0)  # Rotate along X-axis --> my own reference Z
                    # glRotatef(gyro_y, 0, 1, 0)  # Rotate along Y-axis --> my own reference X
                    # glRotatef(gyro_z, 0, 0, 1)  # Rotate along Z-axis


                    # Draw the object here (assuming a function like drawMyObject())
                    # self.draw_box()

                elif data_type == "gravity":
                    gravity_x, gravity_y, gravity_z = x, y, z

                    # glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

   
                    # # # Compute pitch and roll angles using gravity orientation
                    # pitch_angle = np.arctan2(gravity_x, gravity_z) * (180.0 / np.pi)  # Rotation around X
                    # roll_angle  = np.arctan2(gravity_y, gravity_z) * (180.0 / np.pi)  # Rotation around Y
                    # # yaw_angle  = np.arctan2(gravity_y, gravity_x) * (180.0 / np.pi)  # Rotation around Z

                    # # Retrieve the original modelview matrix BEFORE transformations
                    # original_modelview_matrix = glGetFloatv(GL_MODELVIEW_MATRIX)

                    # glPushMatrix() 
                    # glLoadMatrixf(original_modelview_matrix)  # Restore original matrix

                    # # Apply rotation updates based on gravity orientation
                    # glRotatef(pitch_angle, 0, 0, 1)  # Rotate along X-axis --> my own reference Z
                    # glRotatef(roll_angle, 1, 0, 0)  # Rotate along Y-axis --> my own reference X
                    # #glRotatef(yaw_angle, 0, 0, 1)  # Rotate along Z-axis


                    # # Draw the object here (assuming a function like drawMyObject())
                    # self.draw_box()

                    # glPopMatrix()  # Restore previous matrix state

            #self.draw_box()
            pygame.display.flip()
            pygame.time.wait(10)

        pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    imu_visualizer = IMU3DVisualizer()

    # Start ROS processing in the main thread
    rclpy.spin(imu_visualizer)

    # Cleanup
    imu_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
