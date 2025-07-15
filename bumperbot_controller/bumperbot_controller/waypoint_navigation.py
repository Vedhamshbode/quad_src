#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import math

class WaypointController(Node):
    def __init__(self):
        super().__init__("waypoint_controller")

        # Declare parameters with default values
        self.declare_parameter("waypoint_1_x", 2.0)
        self.declare_parameter("waypoint_1_y", 1.0)
        self.declare_parameter("waypoint_2_x", 3.0)
        self.declare_parameter("waypoint_2_y", 4.0)
        self.declare_parameter("kp", 0.5)
        self.declare_parameter("ki", 0.0002)
        self.declare_parameter("kd", 0.0)

        # Retrieve parameters
        self.waypoint_1_x = self.get_parameter("waypoint_1_x").value
        self.waypoint_1_y = self.get_parameter("waypoint_1_y").value
        self.waypoint_2_x = self.get_parameter("waypoint_2_x").value
        self.waypoint_2_y = self.get_parameter("waypoint_2_y").value
        self.kp = self.get_parameter("kp").value
        self.ki = self.get_parameter("ki").value
        self.kd = self.get_parameter("kd").value

        self.odom_sub = self.create_subscription(Odometry,'/odom', self.odom_callback,10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel',10)
        self.goal1_flag = False
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.rx = 0.0
        self.ry = 0.0
        self.lin_err_sum = 0.0
        self.control_timer = self.create_timer(0.1,self.waypoint_controller)

        # Print parameter values
        self.get_logger().info(f"Waypoint 1: ({self.waypoint_1_x}, {self.waypoint_1_y})")
        self.get_logger().info(f"Waypoint 2: ({self.waypoint_2_x}, {self.waypoint_2_y})")
        self.get_logger().info(f"PID Gains: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x 
        self.y = msg.pose.pose.position.y
        q = []
        q.append(msg.pose.pose.orientation.x)
        q.append(msg.pose.pose.orientation.y)
        q.append(msg.pose.pose.orientation.z)
        q.append(msg.pose.pose.orientation.w)
        r,p,self.theta = euler_from_quaternion(q)
        # self.theta = math.pi/2 - self.theta
        # print("theta ",theta)
    
    def waypoint_controller(self):
        if (self.goal1_flag==False):
            # theta_d = math.acos(self.waypoint_1_x/math.sqrt(self.waypoint_1_x*self.waypoint_1_x + self.waypoint_1_y*self.waypoint_1_y))\
            theta_d = math.atan(self.waypoint_1_y/self.waypoint_1_x)
            ang_err = theta_d - self.theta
            vel_msg = Twist()
            vel_msg.angular.z = ang_err*self.kp
            print("ang1", ang_err)
            self.vel_pub.publish(vel_msg)
            self.rx = self.waypoint_1_x - self.x
            self.ry = self.waypoint_1_y - self.y

            if(abs(ang_err)<0.001):
                lin_err = math.sqrt(self.rx*self.rx + self.ry*self.ry)
                self.lin_err_sum = lin_err + self.lin_err_sum
                # vx = self.kp*math.sqrt(rx*rx + ry*ry)
                vx = self.kp*lin_err + self.ki*self.lin_err_sum
                vel_msg.linear.x = vx
                vel_msg.linear.y = 0.0
                vel_msg.angular.z = 0.0
            
                print("linear error 1", lin_err)
                if(lin_err<0.0023):
                    self.get_logger().info("Goal 1 reached!")
                    vel_msg.linear.x = 0.0
                    vel_msg.angular.z = 0.0
                    vel_msg.linear.y = 0.0
                    self.goal1_flag = True
                self.vel_pub.publish(vel_msg)
            
        if self.goal1_flag:
            vel_msg = Twist()

            # theta_d = math.acos(self.waypoint_2_x/math.sqrt(self.waypoint_2_x*self.waypoint_2_x + self.waypoint_2_y*self.waypoint_2_y))
            # self.get_logger().info("Going towards Goal 2...")

            self.rx = self.waypoint_2_x - self.x
            self.ry = self.waypoint_2_y - self.y
            theta_d = math.atan(self.waypoint_2_y/math.sqrt(self.waypoint_2_x))
            ang_err = theta_d - self.theta
            vel_msg = Twist()
            vel_msg.angular.z = ang_err*self.kp
            print("ang err ",ang_err)
            self.vel_pub.publish(vel_msg)

            if(abs(ang_err)<0.001):
                lin_err = math.sqrt(self.rx*self.rx + self.ry*self.ry)
                self.lin_err_sum = lin_err + self.lin_err_sum
                # vx = self.kp*math.sqrt(rx*rx + ry*ry)
                vx = self.kp*lin_err + self.ki*self.lin_err_sum
                # vx = self.kp*lin_err
                vel_msg.linear.x = vx
                vel_msg.linear.y = 0.0
                vel_msg.angular.z = 0.0
            
                print("linear error 2", lin_err)
                if(lin_err<0.28):
                    self.get_logger().info("Goal 2 reached!")
                    vel_msg.linear.x = 0.0
                    goal2_flag = True
                self.vel_pub.publish(vel_msg)
            








def main(args=None):
    rclpy.init(args=args)
    node = WaypointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
