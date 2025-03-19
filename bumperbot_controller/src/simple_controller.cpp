#include "simple_controller.hpp"
#include <eigen3/Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;

SimpleController::SimpleController(const std::string & name)
    : Node(name)
    , left_wheel_prev_pos(0.0)
    , right_wheel_prev_pos(0.0)
{
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheelbase", 0.17);
    declare_parameter("max_rpm", 1000.0);


    wheel_radius = get_parameter("wheel_radius").as_double();
    wheel_separation = get_parameter("wheelbase").as_double();
    max_rpm = get_parameter("max_rpm").as_double();


    RCLCPP_INFO_STREAM(get_logger(), "Using_wheel_radius"<<wheel_radius);
    RCLCPP_INFO_STREAM(get_logger(), "Using_wheel_separation"<<wheel_separation);

    prev_time = get_clock()->now();

    wheel_cmd_publisher = create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands",10);
    left_wheel_publisher = create_publisher<std_msgs::msg::Float64>("/left_wheel_rpm",10);
    right_wheel_publisher = create_publisher<std_msgs::msg::Float64>("/right_wheel_rpm",10);

    vel_sub = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,
        std::bind(&SimpleController::vel_callback, this,_1));

    joint_sub = create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,
        std::bind(&SimpleController::joint_callback, this,_1));

    odom_pub = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    speed_conversion << wheel_radius/2, wheel_radius/2, wheel_radius/wheel_separation, -wheel_radius/wheel_separation;

    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;

    transform_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped.header.frame_id = "odom";
    transform_stamped.child_frame_id = "base_footprint"; 

    geometry_msgs::msg::TransformStamped transform_stamped;

    RCLCPP_INFO_STREAM(get_logger(), "the conversion matrix is \n"<< speed_conversion);
    
}
void SimpleController::vel_callback(const geometry_msgs::msg::Twist & msg){
    Eigen::Vector2d robot_speed(msg.linear.x, msg.angular.z);
    Eigen::Vector2d wheel_speed = speed_conversion.inverse() * robot_speed;
    std_msgs::msg::Float64MultiArray wheel_speed_msg;

    if(wheel_speed.coeff(1)<max_rpm && wheel_speed.coeff(0)<max_rpm){
        wheel_speed_msg.data.push_back(wheel_speed.coeff(1));
        wheel_speed_msg.data.push_back(wheel_speed.coeff(0));

        wheel_cmd_publisher->publish(wheel_speed_msg);
    }
    else{
        RCLCPP_INFO_STREAM(get_logger(), "the rpm value is beyond limit \n");
 
    }
}
void SimpleController::joint_callback(const sensor_msgs::msg::JointState & msg){
    std_msgs::msg::Float64 left_wheel_speed;
    std_msgs::msg::Float64 right_wheel_speed;

    double dp_left = msg.position.at(1)- left_wheel_prev_pos;
    double dp_right = msg.position.at(0)- right_wheel_prev_pos;

    rclcpp::Time msg_time = msg.header.stamp;
    rclcpp::Duration dt = msg_time - prev_time;
    
    left_wheel_prev_pos =  msg.position.at(1);
    right_wheel_prev_pos = msg.position.at(0);
    prev_time = msg_time;

    double fi_left = dp_left/dt.seconds();
    double fi_right = dp_right/dt.seconds();

    double linear = (wheel_radius * fi_left + wheel_radius* fi_right)/2;
    double angular = (wheel_radius*fi_right - wheel_radius*fi_left)/wheel_separation;

    double ds = (wheel_radius*dp_left + wheel_radius*dp_right)/2;
    double d_theta = (wheel_radius*dp_right - wheel_radius*dp_left)/wheel_separation;

    theta += d_theta;
    x += ds*cos(theta);
    y += ds*sin(theta);

    tf2::Quaternion q;
    q.setRPY(0,0,theta);
    odom_msg.pose.pose.orientation.x= q.x();
    odom_msg.pose.pose.orientation.y= q.y();
    odom_msg.pose.pose.orientation.z= q.z();
    odom_msg.pose.pose.orientation.w= q.w();
    odom_msg.header.stamp = get_clock()->now();
    odom_msg.pose.pose.position.x  = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.twist.twist.linear.x = linear;
    odom_msg.twist.twist.angular.z = angular;

    transform_stamped.transform.translation.x = x;
    transform_stamped.transform.translation.y = y;
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();
    transform_stamped.header.stamp = get_clock()->now();

    left_wheel_speed.data = fi_left*60;
    right_wheel_speed.data = fi_right*60;
    
    left_wheel_publisher->publish(left_wheel_speed);
    right_wheel_publisher->publish(right_wheel_speed);
    odom_pub->publish(odom_msg);

    transform_broadcaster->sendTransform(transform_stamped);
    RCLCPP_INFO_STREAM(get_logger(), "Linear: "<<linear<<" Angular velocity "<<angular);


}


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleController>("controller_cpp");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
