#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from project4b.disc_robot import load_disc_robot

class VelocityTranslator(Node):
    def __init__(self,robot_name):
        super().__init__('velocity_translator')
        
        # Robot wheel distance
        self.robot = load_disc_robot(robot_name)
        self.wheel_distance = self.robot['wheels']['distance']
        # print(self.wheel_distance)
        
        # Subscribers
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Publishers
        self.publisher_vl = self.create_publisher(Float64, '/vl', 10)
        self.publisher_vr = self.create_publisher(Float64, '/vr', 10)

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocity
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        
        # Calculate wheel velocities
        vl = linear_velocity - angular_velocity * self.wheel_distance / 2.0
        vr = linear_velocity + angular_velocity * self.wheel_distance / 2.0
        
        # Publish wheel velocities
        vl_msg = Float64()
        vr_msg = Float64()
        vl_msg.data = vl
        vr_msg.data = vr
        self.publisher_vl.publish(vl_msg)
        self.publisher_vr.publish(vr_msg)

def main(args=None):
    rclpy.init(args=args)
    ## To take in the num_turtles as arguments 

    node = rclpy.create_node('temp_node_for_param_retrieval')
    node.declare_parameter("robot_name",'/home/lab2004/project_ws/src/project4a/project4a/normal.robot')
    robot_name = node.get_parameter("robot_name").value
    node.destroy_node()


    velocity_translator = VelocityTranslator(robot_name)
    rclpy.spin(velocity_translator)
    velocity_translator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
