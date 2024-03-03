#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math
import random
from geometry_msgs.msg import Quaternion
from rclpy.duration import Duration
from project4c.disc_robot import load_disc_robot
from project4c.world_robot import load_world
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import yaml


class DifferentialDriveSimulator(Node):
    def __init__(self,robot_name,world_file_name):
        super().__init__('differential_drive_simulator')
        self.robot = load_disc_robot(robot_name)
        # Robot parameters
        self.radius = self.robot['body']['radius']
        self.wheel_distance = self.robot['wheels']['distance']
        self.error_variance_left = self.robot['wheels']['error_variance_left']
        self.error_variance_right = self.robot['wheels']['error_variance_right']
        self.error_update_rate = self.robot['wheels']['error_update_rate']
        # print(f'{self.error_update_rate = }')
        self.last_command_time = self.get_clock().now()
        self.load_world = load_world(world_file_name)
        # Pose
        self.x = self.load_world['initial_pose'][0]
        self.y = self.load_world['initial_pose'][1]
        self.theta = self.load_world['initial_pose'][2]

        #laser

        self.laser_rate = self.robot['laser']['rate']
        self.laser_count = self.robot['laser']['count']
        self.angle_min = self.robot['laser']['angle_min']
        self.angle_max = self.robot['laser']['angle_max']
        self.range_min = self.robot['laser']['range_min']
        self.range_max = self.robot['laser']['range_max']
        self.laser_error_variance = self.robot['laser']['error_variance']
        self.laser_fail_probability = self.robot['laser']['fail_probability']
        self.laser_publish_rate = 20

        
        # Velocity
        self.vl = 0.0
        self.vr = 0.0
        
        # Error
        self.left_error = 1.0
        self.right_error = 1.0
        self.last_error_update_time = self.get_clock().now()

        # New publisher for the OccupancyGrid
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        # load the map and publish once
        self.map_msg = self.read_world_file(world_file_name)
        
        # Created a timer to periodically publish the map
        self.map_timer = self.create_timer(2.0, self.publish_map)

        
        self.laser_scan_publisher = self.create_publisher(LaserScan, '/scan', 10)

        self.laser_scan_timer = self.create_timer(1.0 / self.laser_publish_rate, self.publish_laser_scan)

        # Subscribers
        self.subscription_vl = self.create_subscription(Float64, '/vl', self.vl_callback, 10)
        self.subscription_vr = self.create_subscription(Float64, '/vr', self.vr_callback, 10)

        # Transform broadcaster
        self.br = TransformBroadcaster(self)
        
        # Timer
        self.timer = self.create_timer(0.1, self.update_pose)

    def publish_map(self):
        # This function will be called periodically by the timer
        self.map_publisher.publish(self.map_msg)

    def vl_callback(self, msg):
        self.vl = msg.data
        self.last_command_time = self.get_clock().now()  # Update the timestamp when a command is received

    def vr_callback(self, msg):
        self.vr = msg.data
        self.last_command_time = self.get_clock().now()  # Update the timestamp when a command is received
        
    def update_error(self):
        current_time = self.get_clock().now()
        # Check if more than one second has passed since the last command
        if (current_time - self.last_command_time) > Duration(seconds=1.0):
            self.vl = 0.0
            self.vr = 0.0

        if current_time - self.last_error_update_time > Duration(seconds=self.error_update_rate):
            self.left_error = np.random.normal(1, math.sqrt(self.error_variance_left))
            self.right_error = np.random.normal(1, math.sqrt(self.error_variance_right))
            self.last_error_update_time = current_time
    
    def update_pose(self):
        self.update_error()
        
        vl = self.vl * self.left_error
        vr = self.vr * self.right_error
        
        v = (vl + vr) / 2
        omega = (vr - vl) / self.wheel_distance
        
        delta_t = 0.1  # update interval
        delta_x = v * math.cos(self.theta) * delta_t
        delta_y = v * math.sin(self.theta) * delta_t
        delta_theta = omega * delta_t
        
        # Calculate next position
        next_x = self.x + delta_x
        next_y = self.y + delta_y

        # Validate next position
        if math.isnan(next_x) or math.isnan(next_y):
            # self.get_logger().error(f"Invalid pose calculation: next_x={next_x}, next_y={next_y}")
            return  # Skip updating pose

        # Check for collision
        if not self.check_collision(next_x, next_y):
            self.x = next_x
            self.y = next_y
        else:
            # Stop the robot at the obstacle boundary
            self.vl = 0.0
            self.vr = 0.0

        self.theta += delta_theta
        
        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.euler_to_quaternion(0, 0, self.theta)
        
        self.br.sendTransform(t)


    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx,y=qy,z=qz,w=qw)


    def read_world_file(self,filename):
        with open(filename) as file:
            world = yaml.safe_load(file)
        
        resolution = world['resolution']
        # print(f'the {resolution = }')
        # robot_pose = world['robot_pose']
        map_data = world['map']
        # Split the map string into rows
        map_rows = map_data.split('\n')
        # print(f'the {map_data = }')
         # Strip out newline characters and create a 2D list of map cells

        map_array = [list(row) for row in map_rows]

        # print(f'the {len(map_array) = }')
        origin_x = 0.0
        origin_y = 0.0
        #creating the occupancy grid message

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'world'
        grid.info.resolution = resolution
        grid.info.width = len(map_array[0])
        grid.info.height = len(map_array)
        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y
        grid.info.origin.orientation.w = 1.0 # No rotation

        # Flattening the map data and converting it to occupancy values

        flat_map = []
        for row in reversed(map_array): # reversed to match the occupancy grid coordinate system
            # print(f'the {row = }')
            for char in row:
                if char == '#':
                    flat_map.append(100)
                elif char =='.':
                    flat_map.append(0)
                elif char == '\n':
                    continue

        grid.data = flat_map
        # print(f'the {flat_map = }')
        # print(f'the {grid.data = }')
        return grid

    def calculate_laser_distances(self):
        laser_distances = []
        for i in range(self.laser_count):
            angle = self.theta + self.angle_min + i * (self.angle_max - self.angle_min)/self.laser_count
            distance = self.ray_cast(angle)
            laser_distances.append(distance)
        return laser_distances
    
    def ray_cast(self, angle):

        if random.random() < self.laser_fail_probability:
            return float('nan')  # Return NaN for a failed measurement
    # Start at the robot's location
        x = self.x
        y = self.y

        # Calculate the ray's direction in the world frame
        world_angle = self.theta + angle
        dx = math.cos(world_angle)
        dy = math.sin(world_angle)

        # Step size for the ray increments, depending on the map resolution
        step_size = self.map_msg.info.resolution / 2.0
        max_distance = self.range_max
        distance = 0

        # Incrementally step along the ray
        while distance < max_distance:
            # Check the occupancy grid cell at the current ray position
            map_x = int((x - self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
            map_y = int((y - self.map_msg.info.origin.position.y) / self.map_msg.info.resolution)

            # Check if out of bounds
            if map_x < 0 or map_y < 0 or map_x >= self.map_msg.info.width or map_y >= self.map_msg.info.height:
                return max_distance  # Return max range if out of bounds

            # Calculate the index in the occupancy grid data array
            index = map_y * self.map_msg.info.width + map_x

            # Check if the occupancy grid cell is occupied
            if self.map_msg.data[index] == 100:
                # Add some noise to the measurement
                noise = np.random.normal(0, math.sqrt(self.laser_error_variance))
                measured_distance = distance + noise
                return min(max(measured_distance, self.range_min), max_distance)

            # Move to the next position along the ray
            x += dx * step_size
            y += dy * step_size
            distance += step_size

        # If no obstacle was hit, return the max range
        return max_distance

    def publish_laser_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = (scan.angle_max - scan.angle_min) / self.laser_count
        scan.time_increment = 0.0
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        scan.ranges = [self.ray_cast(angle) for angle in np.linspace(scan.angle_min, scan.angle_max, self.laser_count)]

        self.laser_scan_publisher.publish(scan)    


    def check_collision(self, next_x, next_y):
        map_resolution = self.map_msg.info.resolution
        map_origin = self.map_msg.info.origin.position

        # Calculate the bounds of the robot considering its radius
        bounds = [
            (next_x + math.cos(angle) * self.radius, next_y + math.sin(angle) * self.radius)
            for angle in np.linspace(0, 2 * math.pi, num=60)  # Check around the circumference
        ]

        for bx, by in bounds:
            # Convert each bound point to map coordinates
            map_x = int((bx - map_origin.x) / map_resolution)
            map_y = int((by - map_origin.y) / map_resolution)

            # Check if the cell is within map bounds
            if (map_x >= self.map_msg.info.width or map_x < 0 or
                map_y >= self.map_msg.info.height or map_y < 0):
                return True  # Collision with boundary

            # Check if the cell is occupied
            index = map_y * self.map_msg.info.width + map_x
            if index < len(self.map_msg.data) and self.map_msg.data[index] == 100:
                return True  # Collision with obstacle

        return False  # No collision






def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('temp_node_for_robot_name_retrieval')
    node.declare_parameter("robot_name",'/home/lab2004/project_ws/src/project4a/project4a/normal.robot')
    robot_name = node.get_parameter("robot_name").value
    # print(f'in diff drive node the robot name is {robot_name}')
    node.destroy_node()

    node = rclpy.create_node('temp_node_for_world_name_retrieval')
    node.declare_parameter("world_name",'/home/lab2004/project_ws/src/project4c/project4c/windy.world')
    world_file_name = node.get_parameter("world_name").value
    node.destroy_node()
    


    differential_drive_simulator = DifferentialDriveSimulator(robot_name,world_file_name)
    rclpy.spin(differential_drive_simulator)
    differential_drive_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
