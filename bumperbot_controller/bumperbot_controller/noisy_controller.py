#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import numpy as np
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from nav_msgs.msg import Odometry
import math
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

class noisyController(Node):
    def __init__(self):
        super().__init__("simple_controller")

        self.declare_parameter("wheel_radius",0.033)
        self.declare_parameter("wheel_separation",0.17) #params that can be modified as per need

        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value #read the value passed and store it
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value #read the value passed and store it

        self.get_logger().info("Using wheel_radius %f" % self.wheel_radius)
        self.get_logger().info("Using wheel_separation %f" % self.wheel_separation)

        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0 
        self.prev_time_ = self.get_clock().now()

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        self.joint_sub_  = self.create_subscription(JointState,"joint_states",self.joint_callback,10) #wheel encoder data!!!!
        self.odom_pub_ = self.create_publisher(Odometry, 'bumperbot_controller/odom_noisy',10)

      
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom" #fixed frame wrt it robot expresses its coordinates
        self.odom_msg_.child_frame_id = "base_footprint_ekf"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        self.br = TransformBroadcaster(self)
        self.transform_stamped = TransformStamped()
        self.transform_stamped.header.frame_id = "odom"
        self.transform_stamped.child_frame_id = "base_footprint_noisy" #this displays in rviz

    # calculate linear and ang velocity of robot from position encoder data 
    def joint_callback(self,msg):
        wheel_encoder_left = msg.position[1] + np.random.normal(0,0.005) #adding gaussian noise!!!
        wheel_encoder_right = msg.position[0] + np.random.normal(0,0.005)
        dp_left = wheel_encoder_left - self.left_wheel_prev_pos_
        dp_right = wheel_encoder_right - self.right_wheel_prev_pos_
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_ #convert to ros2 time object to subtract time
        # update the values for next ieteration!!!!
        self.left_wheel_prev_pos_ = msg.position[1]
        self.right_wheel_prev_pos_ = msg.position[0]
        self.prev_time_ = Time.from_msg(msg.header.stamp)
        # ang velocities
        fi_left = dp_left / (dt.nanoseconds/S_TO_NS)
        fi_right = dp_right / (dt.nanoseconds/ S_TO_NS)

        linear = (self.wheel_radius*fi_right + self.wheel_radius*fi_left)/2
        angular = (self.wheel_radius*fi_right - self.wheel_radius*fi_left)/self.wheel_separation

        ds = (self.wheel_radius*dp_right + self.wheel_radius*dp_left)/2
        d_theta = (self.wheel_radius*dp_right - self.wheel_radius*dp_left)/self.wheel_separation
        # calculate pos and orientation of robot!!!
        self.theta_ += d_theta
        self.x_ += ds*math.cos(self.theta_)
        self.y_ += ds*math.sin(self.theta_)
        
        q = quaternion_from_euler(0,0,self.theta_)
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular

        self.transform_stamped.transform.translation.x = self.x_
        self.transform_stamped.transform.translation.y = self.y_
        self.transform_stamped.transform.rotation.x = q[0]
        self.transform_stamped.transform.rotation.y = q[1]
        self.transform_stamped.transform.rotation.z = q[2]
        self.transform_stamped.transform.rotation.w = q[3]
        self.transform_stamped.header.stamp = self.get_clock().now().to_msg()

        self.br.sendTransform(self.transform_stamped)
        self.odom_pub_.publish(self.odom_msg_)
# publish the odometry message!!

        self.get_logger().info("linear: %f, angular: %f"%(linear,angular))
        self.get_logger().info("x: %f, y: %f theta: %f"%(self.x_,self.y_,self.theta_))

def main():
    rclpy.init()
    noisy_controller = noisyController()
    rclpy.spin(noisy_controller)
    noisy_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
