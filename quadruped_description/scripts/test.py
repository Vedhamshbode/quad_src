#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher = self.create_publisher(JointTrajectory, '/arm_cont/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.publish_trajectory)

    def publish_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = ['base_link1_joint', 'link1_link2_joint', 'link2_link3_joint']  # List of joint names
        point = JointTrajectoryPoint()
        point.positions = [-0.89, -1.45, 0.78]  # List of joint positions
        point.time_from_start = Duration()  # Time duration
        point.time_from_start.sec = 1
        msg.points.append(point)
        self.publisher.publish(msg)
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = TrajectoryPublisher()
    rclpy.spin(trajectory_publisher)
    trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

