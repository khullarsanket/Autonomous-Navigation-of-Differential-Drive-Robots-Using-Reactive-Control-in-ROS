#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller_2')

        # Subscriber to the LaserScan data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # Publisher for the robot's velocity commands
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

    def scan_callback(self, msg):
        # Calculated the size of each region by dividing the total readings by 5
        region_size = len(msg.ranges) // 5
        
        #  Dividing the laser scan data into distinct regions (front, front-left, front-right, left, right).
        regions = {
            'right':  min(min(msg.ranges[0:region_size]), 40.0),  # Closest obstacle in the right slice
            'fright': min(min(msg.ranges[region_size:2*region_size]), 40.0),  # Closest obstacle in the front-right slice
            'front':  min(min(msg.ranges[2*region_size:3*region_size]), 40.0),  # Closest obstacle in the front slice
            'fleft':  min(min(msg.ranges[3*region_size:4*region_size]), 40.0),  # Closest obstacle in the front-left slice
            'left':   min(min(msg.ranges[4*region_size:]), 40.0),  # Closest obstacle in the left slice
        }

        self.take_action(regions)

    def take_action(self,regions):
        msg = Twist()
        linear_x = 0.0
        angular_z = 0.0

        # Calculates the difference between left and right distances
        side_diff = regions['left'] - regions['right']
        
        # Proportional control factor for angular velocity
        angular_control_factor = 0.2  # Adjusted based on robot's responsiveness

        # Minimum safe distance from any obstacle
        safe_distance = 0.16 # Adjusted based on  requirements 
        state = ''
        # Scaling factor for proportional control
        scaling_factor = 2.0  # adjusted this based on testing

        min_speed_threshold = 1.0 # Minimum speed threshold
        narrow_corridor_threshold = 1.0  # Threshold to detect narrow corridors

        # Check if the robot is in a narrow corridor
        in_narrow_corridor = all(region < narrow_corridor_threshold for region in regions.values())

        ## Implementing the reactive controller

        if regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] > 1:
            state = 'case 1 - nothing'
            linear_x = scaling_factor/regions['front']
            angular_z = 0.0
        elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] > 1:
            state = 'case 2 - front'
            linear_x = 0.0
            angular_z = scaling_factor/regions['front']
            # angular_z = 0.5

        elif regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] < 1:
            state = 'case 3 - front_right'
            linear_x = 0.0
            angular_z = scaling_factor/regions['fright']
            # angular_z = 0.5

        elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] > 1:
            state = 'case 4 - fornt_left'
            linear_x = 0.0
            angular_z = -scaling_factor/regions['fleft']
            # angular_z = -0.5

        elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] < 1:
            state = 'case 5 - front and fornt_right'
            linear_x = 0.0
            angular_z = scaling_factor/regions['fright']
            # angular_z = 0.5

        elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] > 1:
            state = 'case 6 - front and front_left'
            linear_x = 0.1  # Small forward motion
            angular_z = -angular_control_factor * side_diff
            # angular_z = -0.5

        elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] < 1:
            state = 'case 7 - front and front_left and front_right'
            # Small forward motion, adjust angular_z based on side_diff
            linear_x = 0.1
            angular_z = angular_control_factor * side_diff

        elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] < 1:
            state = 'case 8 - front_left and front_right'
            # Adjust linear_x and angular_z based on side_diff
            linear_x = scaling_factor/regions['front']
            angular_z = 0.0
        else:
            state = 'unknown case'

        # Increase speed in narrow corridors
        if in_narrow_corridor:
            linear_x = max(linear_x, min_speed_threshold)

        # To get the state of the robot
        # self.get_logger().info(state) 

        ## Ensure the robot is not too close to any side
        if regions['left'] < safe_distance or regions['right'] < safe_distance:
            angular_z = angular_control_factor * side_diff

        # Ensure angular velocity is within some limits
        max_angular_speed = 1.0  # Define a max angular speed
        angular_z = max(min(angular_z, max_angular_speed), -max_angular_speed)
        
        max_linear_speed = 1.0
        linear_x = min(linear_x, max_linear_speed)

        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    navigation_controller = NavigationController()
    rclpy.spin(navigation_controller)
    navigation_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
