#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from time import time

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # Timer to call the publish_joint_state method at 1Hz
        self.timer = self.create_timer(1.0, self.publish_joint_state)
        
        self.joint_name = 'Revolute 2'
        self.joint_angle = 0.0  # Starting at 0 radians

    def publish_joint_state(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()  # Current time as timestamp
        joint_state_msg.header.frame_id = 'base_link'  # Set the frame_id
        joint_state_msg.name = [self.joint_name]
        joint_state_msg.position = [self.joint_angle]
        joint_state_msg.velocity = [0.0]  # Assuming no velocity for simplicity
        joint_state_msg.effort = [0.0]    # Assuming no effort for simplicity
        self.get_logger().info(f'Publishing: {joint_state_msg}')
        self.publisher_.publish(joint_state_msg)
        self.joint_angle += 0.1  # Change the angle by 0.1 radians each time
        if self.joint_angle > 3.14:  # Keep the angle within a reasonable range (e.g., π)
            self.joint_angle = 0.0  # Reset to 0 if it exceeds π (or your max angle)

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
