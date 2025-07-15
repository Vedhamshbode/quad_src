#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher = self.create_publisher(JointTrajectory, '/lb_cont/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.publish_trajectory)

    def publish_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = ['lb1_base', 'lb1_2', 'lb2_3']  # List of joint names
        point = JointTrajectoryPoint()
        point.positions = [-1.55, 0.0, 0.0]  # List of joint positions
        point.time_from_start = Duration()  # Time duration
        point.time_from_start.sec = 3
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

