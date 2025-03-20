#include "rclcpp/rclcpp.hpp"
#include "custom_service/srv/ik_sw.hpp"
#include <memory>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <array>
#include <iterator>
#include <iostream>
#include <cmath>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define PI 3.14159265358979323846

using namespace std::chrono_literals;
using namespace std;
using Matrix4x4 = std::array<std::array<double, 4>, 4>;
using Vector3 = std::array<double, 3>;
using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;

class TrajBezier : public rclcpp::Node
{
public:
struct DHParameter {
    double a;
    double alpha;
    double d;
    double theta;
};

struct Cart_Pose {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

struct Cart_Vel {
    double x_dot;
    double y_dot;
    double z_dot;
    double roll_dot;
    double pitch_dot;
    double yaw_dot;
};

struct Joint_Pose
{
    double Theta_1;
    double Theta_2;
    double Theta_3;

};

    TrajBezier() : Node("traj_node")
    {
        std_msgs::msg::Float32MultiArray transformation;

        joint_traj_pub_rb = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/rb_cont/joint_trajectory", 10);
        joint_traj_pub_lf = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/lf_cont/joint_trajectory", 10);
        joint_traj_pub_lb = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/lb_cont/joint_trajectory", 10);
        joint_traj_pub_rf = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/rf_cont/joint_trajectory", 10);
        vel_sub = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,std::bind(&TrajBezier::vel_callback, this,std::placeholders::_1));
        client_ = this->create_client<custom_service::srv::IkSw>("ik_service");
        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
        // timer_ = this->create_wall_timer(
        //     std::chrono::seconds(1), std::bind(&TrajectoryPublisher::publish_trajectory, this));

        d = 0.08;
        h = 0.03;
        theta = 0.0;  // Example angle in radians (45 degrees)

        Eigen::Vector3d P3(0.025, -0.054, -0.25);
        // Eigen::Vector3d P3(0.15, -0.054, 0.03);
        // marker_pub(B, "dh_ref_lf");
        grounded();

        // trot_gait(T, num_points, beta_u,cycles, d, h, theta);
        // grounded();

        // std::vector<double> vel = {0.5, 0.4, 0.1};
    //     std::vector<double> sol;
    //    sol= vel_ik(vel);
    // while(flag==0){
    //     RCLCPP_INFO(this->get_logger(), "waiting for data... \n");
    // }
        
    }
    void vel_callback(const geometry_msgs::msg::Twist & msg){
        // flag = 1;
        robot_vel = {0.0, 0.0, 0.0};

        RCLCPP_INFO(this->get_logger(), "velocities received..: vx, vy, wz : %f, %f, %f", msg.linear.x, msg.linear.y, msg.angular.z);
    
        if(msg.linear.x<=max_v && msg.linear.y<=max_v && msg.angular.z<=max_w){
            robot_vel[0] = (msg.linear.x);
            robot_vel[1] = (msg.linear.y);
            robot_vel[2] = (msg.angular.z);
        }
        else{
            RCLCPP_INFO(this->get_logger(), "the rpm value is beyond limit \n");
        }

        // if(robot_vel[0] == 0.0 && robot_vel[1]==0.0 && robot_vel[2]==0.0){
        //     is_initial=0;
        //     grounded();
        // }

        teleop(robot_vel);        

    }
    std::vector<double> Inverse_position_kinematics(Vector3 loc, string leg_name){
        //****************/ robot parameters**********
        double d1 = 0.0255;
        double d2 = 0.054;
        double a2 = 0.154;
        double a3 = 0.125;

        double alpha, beta = 0.0;
        double reach = a2 + a3;



        double th11,th1 =0;
        double th13,th14 =0;
        double th2 =0, th21=0, th22=0;
        double th3 =0, th31 = 0, th32=0;
        double max_it1, max_it2 =0;

        Vector3 p3 = {loc[0],loc[1],loc[2]}; // COORDINATES OF EE

        // ************Calculating  th1**************
        // atan(val) -> has range (-pi/2,pi/2)
        // ************While atan2(y,x) has range (-pi,pi) and takes into account sign of x,y!******************
        double psi = atan(p3[1]/p3[0]);
        th1 = asin(d2/sqrt(p3[0]*p3[0] + p3[1]*p3[1])) + psi;
        // cout<<"th1 "<<th1<<endl;

        Vector3 p = {p3[0]-d2*sin(th1), p3[1] + d2*cos(th1), p3[2]};
        // cout<<"xp "<<p[0]<<" yp "<<p[1]<<endl;

    //    ***************************th3*********************

        th31 = acos(((p[0]*p[0]) + (p[1]*p[1]) + ((p[2] - d1)*(p[2] - d1)) - (a2*a2) - (a3*a3))/(2*a2*a3));
        th32 = -acos(((p[0]*p[0]) + (p[1]*p[1]) + ((p[2] - d1)*(p[2] - d1)) - (a2*a2) - (a3*a3))/(2*a2*a3));



        // ************************th21*********************
        alpha = cos(th31);
        double phi = atan((a3* sin(th31))/(a2+(a3*cos(th31))));
        th21 = asin((p[2] - d1)/sqrt((a2+(a3*cos(th31)))*(a2+(a3*cos(th31))) + ((a3*sin(th31))*(a3*sin(th31))) )) - phi;
        // cout<<"th21 "<<th21<<endl;

        // ************************th22*********************
        alpha = cos(th32);
        phi = atan((a3* sin(th32))/(a2+(a3*cos(th32))));
        th22 = asin((p[2] - d1)/sqrt((a2+(a3*cos(th32)))*(a2+(a3*cos(th32))) + ((a3*sin(th32))*(a3*sin(th32))) )) - phi;
    

        if(leg_name == "right"){
            th2 = th21;
            th3 = th31;
        }
        else{
            th2 = th22;
            th3 = th32;
        }


    // **************************************NUMERICAL****************
        double distPerUpdate = 0.001 * reach;
        double norm = 1000;
        double max_it = 1000;

        while (max_it>0.001){
            std::vector<DHParameter> dhParams = {
                {0,     PI / 2,    0.0255,    th1},
                {0.154,    0,         0.054,      th2},
                {0.125,   0,    0,   th3}
            };

            Cart_Pose pose = forwardKinematics(dhParams);
            std::vector<double> evec;
            evec.push_back(abs(pose.x - p3[0]));
            evec.push_back(abs(pose.y - p3[1]));
            evec.push_back(abs(pose.z - p3[2]));

            max_it = *(std::max_element(evec.begin(), evec.end()));

            norm = sqrt((pose.x - p3[0])*(pose.x - p3[0]) + (pose.y - p3[1])*(pose.y - p3[1]) + (pose.z - p3[2])*(pose.z - p3[2]));
            Vector dr = {-distPerUpdate*(pose.x - p3[0])/(norm+0.0001), -distPerUpdate*(pose.y - p3[1])/(norm+0.0001), -distPerUpdate*(pose.z - p3[2])/(norm+0.0001)};
            Matrix J = calculateJacobian(dhParams);

            Matrix J_i = pseudoInverse(J);
            
            Vector dtheta = multiplyMatrixVector(J_i,dr);
                    // cout<<"final sol1 "<<th1<<" "<<th2<<" "<<th3<<endl;

            if (abs(th1)>PI/2 || abs(th2)>PI/2 || abs(th3)>PI/2){

                break;
            }
            th1 = th1 + dtheta[0];
            th2 = th2 + dtheta[1];
            th3 = th3 + dtheta[2];
         
        }
        // cout<<"norm "<<norm<<endl;
        cout<<"final sol "<<th1<<" "<<th2<<" "<<th3<<endl;
        std::vector<double> j = {th1, th2, th3};
        return j;
}
 
    std::vector<double> ik_r( std::vector<std::array<double, 3>> B, int current_index)
    {
       
        // ********************************* right back leg data ****************************

        
        Vector3 transformation= {-B[current_index][2], B[current_index][1], B[current_index][0]};
        
        // ********************************* right leg data request ****************************
            std::vector<double> ang;    
            ang = Inverse_position_kinematics(transformation, "right");

            cout<<"ang "<<ang[0]<<" "<<ang[1]<<" "<<ang[2]<<endl;
            return ang;

    }

    std::vector<double> ik_l( std::vector<std::array<double, 3>> B, int current_index)
    {
        
        // ********************************* left leg data ****************************
       
        Vector3 transformation = {-B[current_index][2], B[current_index][1], B[current_index][0]};

            std::vector<double> ang;    
            ang = Inverse_position_kinematics(transformation, "left");

            return ang;
    }

    void pubjangles(std::vector<double> jangles, std::vector<std::string> joint_names_, double T, double num_points, string leg_name){
            auto msg = trajectory_msgs::msg::JointTrajectory();
        
            // List of joint names
            msg.joint_names = joint_names_;
    
            auto point = trajectory_msgs::msg::JointTrajectoryPoint();
            point.positions = {jangles[0], jangles[1], jangles[2]};  // Target joint positions
            
            // Set the duration to 1 second
            builtin_interfaces::msg::Duration duration;
            duration.sec = 0;         // Full seconds part
            duration.nanosec = T/num_points * 1000000000;  // 0.5 seconds = 500 million nanoseconds
            point.time_from_start = duration;
            msg.points.push_back(point);
            

            // Publish the trajectory
            if (leg_name == "rb"){
                joint_traj_pub_rb->publish(msg);
            }

            else if (leg_name == "lf"){
                joint_traj_pub_lf->publish(msg);
            }
            
            RCLCPP_INFO(this->get_logger(), "Published joint trajectory");
           
            double time_to_wait = (T / num_points) * 1000;  // Convert to milliseconds
            rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(time_to_wait)));
    }

    void marker_pub(std::vector<std::array<double, 3>> pts, string frame_id ){
        // ******************* marker code ******************
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker marker;
        for (int i =0; i<pts.size(); i++){
            marker.header.frame_id = frame_id;  // Change to "odom" or "base_link" if needed
            marker.header.stamp = this->now();
            marker.ns = "marker_points";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
    
            marker.pose.position.x = -pts[i][2];
            marker.pose.position.y = pts[i][1];
            marker.pose.position.z = -pts[i][0];
    
            marker.scale.x = 0.009;  // Marker size
            marker.scale.y = 0.009;
            marker.scale.z = 0.009;
    
            marker.color.a = 1.0;  // Fully visible
            marker.color.r = 1.0;  // Red color
            marker.color.g = 0.0;
            marker.color.b = 0.0;
    
            marker.lifetime = rclcpp::Duration::from_seconds(0);  // Infinite lifetime
    
            marker_array.markers.push_back(marker);
            publisher_->publish(marker_array);
        }
    }

    void trot_gait(double T, double num_points, double beta_u, int cycles , double d, double h, double theta){

        Eigen::Vector3d P3r(0.0, -0.054, -0.25);
        Eigen::Vector3d P3l(0.045, -0.054, -0.25);
        // Eigen::Vector3d P3(0.025, -0.054, -0.25);

        Br = trajplanner(d, h, theta, P3r, beta_u, num_points);
        Bl = trajplanner(d, h, theta, P3l, beta_u, num_points);

        sol_r.clear();
        sol_l.clear();
        for (int i =0; i<Br.size();i++){

            sol_r.push_back(ik_r(Br,i));
            sol_l.push_back(ik_l(Bl,Br.size() - i -1));
            cout<<i<<endl;
        } 
        int count = 0, c= 0;
        // double shift_time = (T- beta_u*T);

            //#################### initialization of RB ###################################
            auto msg_rb = trajectory_msgs::msg::JointTrajectory();
            msg_rb.joint_names = {"rb1_base", "rb1_2", "rb2_3"};
    
            auto point_rb = trajectory_msgs::msg::JointTrajectoryPoint();
            point_rb.positions = {sol_r[0][0], sol_r[0][1], sol_r[0][2]};  // Target joint positions
            
            builtin_interfaces::msg::Duration duration;
            duration.sec = 0;         // Full seconds part
            duration.nanosec = T/num_points * 1000000000;  // 0.5 seconds = 500 million nanoseconds
            point_rb.time_from_start = duration;
            msg_rb.points.push_back(point_rb);
            
            joint_traj_pub_rb->publish(msg_rb);

            //#################### initialization of LF ###################################
            auto msg_lf = trajectory_msgs::msg::JointTrajectory();
            msg_lf.joint_names = {"lf1_base", "lf1_2", "lf2_3"};
    
            auto point_lf = trajectory_msgs::msg::JointTrajectoryPoint();
            point_lf.positions = {sol_l[0][0], sol_l[0][1], sol_l[0][2]}; // Target joint positions
            
            point_lf.time_from_start = duration;
            msg_lf.points.push_back(point_lf);

            joint_traj_pub_lf->publish(msg_lf);

            //#################### initialization of RF ###################################
            auto msg_rf = trajectory_msgs::msg::JointTrajectory();
            msg_rf.joint_names = {"rf1_base", "rf1_2", "rf2_3"};
    
            auto point_rf = trajectory_msgs::msg::JointTrajectoryPoint();
            point_rf.positions = {sol_r[10][0], sol_r[10][1], sol_r[10][2]};  // Target joint positions
            
            point_rf.time_from_start = duration;
            msg_rf.points.push_back(point_rf);
            
            joint_traj_pub_rf->publish(msg_rf);
            
            //#################### initialization of LB ###################################
            auto msg_lb = trajectory_msgs::msg::JointTrajectory();
            msg_lb.joint_names = {"lb1_base", "lb1_2", "lb2_3"};
    
            auto point_lb = trajectory_msgs::msg::JointTrajectoryPoint();
            point_lb.positions = {sol_l[10][0], sol_l[10][1], sol_l[10][2]};  // Target joint positions
            
            point_lb.time_from_start = duration;
            msg_lb.points.push_back(point_lb);
            
            joint_traj_pub_lb->publish(msg_lb);
            
            double time_to_wait = (T / num_points) * 1000;  // Convert to milliseconds
            rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(time_to_wait)));

        for (int j = 0; j<cycles; j++){

            for (int i = 0; i<sol_r.size(); i++)
            {      
                c = 10+i;

                // #########################cycles###  RB ######################
                auto msg_rb = trajectory_msgs::msg::JointTrajectory();
                msg_rb.joint_names = {"rb1_base", "rb1_2", "rb2_3"};
        
                auto point_rb = trajectory_msgs::msg::JointTrajectoryPoint();
                point_rb.positions = {sol_r[i][0], sol_r[i][1], sol_r[i][2]};  // Target joint positions

                builtin_interfaces::msg::Duration duration;
                duration.sec = 0;         // Full seconds part
                duration.nanosec = T/num_points * 1000000000;  // 0.5 seconds = 500 million nanoseconds
                point_rb.time_from_start = duration;
                msg_rb.points.push_back(point_rb);
                
                joint_traj_pub_rb->publish(msg_rb);

                // ############################  LF ######################
                auto msg_lf = trajectory_msgs::msg::JointTrajectory();
                msg_lf.joint_names = {"lf1_base", "lf1_2", "lf2_3"};
        
                auto point_lf = trajectory_msgs::msg::JointTrajectoryPoint();
                point_lf.positions = {sol_l[i][0], sol_l[i][1], sol_l[i][2]}; // Target joint positions
                
                point_lf.time_from_start = duration;
                msg_lf.points.push_back(point_lf);

                joint_traj_pub_lf->publish(msg_lf);


                if (c>19){
                    c = c-19;
                }
                // ############################  RF ######################

                auto msg_rf = trajectory_msgs::msg::JointTrajectory();
                msg_rf.joint_names = {"rf1_base", "rf1_2", "rf2_3"};
        
                auto point_rf = trajectory_msgs::msg::JointTrajectoryPoint();
                point_rf.positions = {sol_r[c][0], sol_r[c][1], sol_r[c][2]};  // Target joint positions
                
                point_rf.time_from_start = duration;
                msg_rf.points.push_back(point_rf);
                
                joint_traj_pub_rf->publish(msg_rf);

                // ############################  LB ######################
                auto msg_lb = trajectory_msgs::msg::JointTrajectory();
                msg_lb.joint_names = {"lb1_base", "lb1_2", "lb2_3"};
        
                auto point_lb = trajectory_msgs::msg::JointTrajectoryPoint();
                point_lb.positions = {sol_l[c][0], sol_l[c][1], sol_l[c][2]};  // Target joint positions
                
                point_lb.time_from_start = duration;
                msg_lb.points.push_back(point_lb);
                
                joint_traj_pub_lb->publish(msg_lb);
                
                RCLCPP_INFO(this->get_logger(), "Published joint trajectory");
            
                double time_to_wait = (T / num_points) * 1000;  // Convert to milliseconds
                rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(time_to_wait)));
            }
        }

    }

    void rot_gait(double T, double num_points, double beta_u, int cycles, double d, double h, double theta){
        
        Eigen::Vector3d P3(0.025, -0.054, -0.25);
        B = trajplanner(d, h, theta, P3, beta_u, num_points);
        sol_r.clear();
        sol_l.clear();
        for (int i =0; i<B.size();i++){

            sol_r.push_back(ik_r(B,i));
            sol_l.push_back(ik_l(B,B.size() - i -1));
            cout<<i<<endl;
        }
        int count = 0, c= 0;
        // double shift_time = (T- beta_u*T);

            //#################### initialization of RB ###################################
            auto msg_rb = trajectory_msgs::msg::JointTrajectory();
            msg_rb.joint_names = {"rb1_base", "rb1_2", "rb2_3"};
    
            auto point_rb = trajectory_msgs::msg::JointTrajectoryPoint();
            point_rb.positions = {sol_r[0][0], sol_r[0][1], sol_r[0][2]};  // Target joint positions
            
            builtin_interfaces::msg::Duration duration;
            duration.sec = 0;         // Full seconds part
            duration.nanosec = T/num_points * 1000000000;  // 0.5 seconds = 500 million nanoseconds
            point_rb.time_from_start = duration;
            msg_rb.points.push_back(point_rb);
            
            joint_traj_pub_rb->publish(msg_rb);

            //#################### initialization of LF ###################################
            auto msg_lf = trajectory_msgs::msg::JointTrajectory();
            msg_lf.joint_names = {"lf1_base", "lf1_2", "lf2_3"};
    
            auto point_lf = trajectory_msgs::msg::JointTrajectoryPoint();
            point_lf.positions = {sol_l[0][0], sol_l[0][1], sol_l[0][2]}; // Target joint positions
            
            point_lf.time_from_start = duration;
            msg_lf.points.push_back(point_lf);

            joint_traj_pub_lf->publish(msg_lf);

            //#################### initialization of RF ###################################
            auto msg_rf = trajectory_msgs::msg::JointTrajectory();
            msg_rf.joint_names = {"rf1_base", "rf1_2", "rf2_3"};
    
            auto point_rf = trajectory_msgs::msg::JointTrajectoryPoint();
            point_rf.positions = {sol_r[10][0], sol_r[10][1], sol_r[10][2]};  // Target joint positions
            
            point_rf.time_from_start = duration;
            msg_rf.points.push_back(point_rf);
            
            joint_traj_pub_rf->publish(msg_rf);
            
            //#################### initialization of LB ###################################
            auto msg_lb = trajectory_msgs::msg::JointTrajectory();
            msg_lb.joint_names = {"lb1_base", "lb1_2", "lb2_3"};
    
            auto point_lb = trajectory_msgs::msg::JointTrajectoryPoint();
            point_lb.positions = {sol_l[10][0], sol_l[10][1], sol_l[10][2]};  // Target joint positions
            
            point_lb.time_from_start = duration;
            msg_lb.points.push_back(point_lb);
            
            joint_traj_pub_lb->publish(msg_lb);
            
            double time_to_wait = (T / num_points) * 1000;  // Convert to milliseconds
            rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(time_to_wait)));

        for (int j = 0; j<cycles; j++){

            for (int i = 0; i<sol_r.size(); i++)
            {      
                c = 10+i;

                // ############################  RB ######################
                auto msg_rb = trajectory_msgs::msg::JointTrajectory();
                msg_rb.joint_names = {"rb1_base", "rb1_2", "rb2_3"};
        
                auto point_rb = trajectory_msgs::msg::JointTrajectoryPoint();
                point_rb.positions = {sol_r[i][0], sol_r[i][1], sol_r[i][2]};  // Target joint positions

                builtin_interfaces::msg::Duration duration;
                duration.sec = 0;         // Full seconds part
                duration.nanosec = T/num_points * 1000000000;  // 0.5 seconds = 500 million nanoseconds
                point_rb.time_from_start = duration;
                msg_rb.points.push_back(point_rb);
                
                joint_traj_pub_rb->publish(msg_rb);

                // ############################  LF ######################
                auto msg_lf = trajectory_msgs::msg::JointTrajectory();
                msg_lf.joint_names = {"lf1_base", "lf1_2", "lf2_3"};
        
                auto point_lf = trajectory_msgs::msg::JointTrajectoryPoint();
                point_lf.positions = {sol_l[i][0], sol_l[i][1], sol_l[i][2]}; // Target joint positions
                
                point_lf.time_from_start = duration;
                msg_lf.points.push_back(point_lf);

                joint_traj_pub_lf->publish(msg_lf);


                if (c>19){
                    c = c-19;
                }
                // ############################  RF ######################

                auto msg_rf = trajectory_msgs::msg::JointTrajectory();
                msg_rf.joint_names = {"rf1_base", "rf1_2", "rf2_3"};
        
                auto point_rf = trajectory_msgs::msg::JointTrajectoryPoint();
                point_rf.positions = {sol_r[c][0], sol_r[c][1], sol_r[c][2]};  // Target joint positions
                
                point_rf.time_from_start = duration;
                msg_rf.points.push_back(point_rf);
                
                joint_traj_pub_rf->publish(msg_rf);

                // ############################  LB ######################
                auto msg_lb = trajectory_msgs::msg::JointTrajectory();
                msg_lb.joint_names = {"lb1_base", "lb1_2", "lb2_3"};
        
                auto point_lb = trajectory_msgs::msg::JointTrajectoryPoint();
                point_lb.positions = {sol_l[c][0], sol_l[c][1], sol_l[c][2]};  // Target joint positions
                
                point_lb.time_from_start = duration;
                msg_lb.points.push_back(point_lb);
                
                joint_traj_pub_lb->publish(msg_lb);
                
                RCLCPP_INFO(this->get_logger(), "Published joint trajectory");
            
                double time_to_wait = (T / num_points) * 1000;  // Convert to milliseconds
                rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(time_to_wait)));
            }
        } 

    }

    std::vector<double> vel_ik(std::vector<double> &req_vel){
        RCLCPP_INFO(this->get_logger(), "vel_ik called..");
        double vrx = req_vel[0];
        double vry = req_vel[1];
        double wr = req_vel[2];

        Eigen::Matrix<double, 4, 3> conv_mat;
        conv_mat << 1, 0, l/2,
             1, 0, -l/2,
             0, 1, 0,
             0, 1, -c;
    
        // Define a 3x1 vector
        Eigen::Matrix<double, 3, 1> robot_vel;
        robot_vel << vrx,
             vry,
             wr;
    
        // Perform matrix multiplication
        Eigen::Matrix<double, 4, 1> lat_vel = conv_mat * robot_vel;
        //op is in {vd, ve, vf, vt}

        double vd = lat_vel(0,0);
        double ve = lat_vel(1,0);
        double vf = lat_vel(2,0);
        double vt = lat_vel(3,0);
        double thrf = atan2(vf,(vd+0.0001));
        double thrb = atan2(vt,(vd+0.0001));
        double thlb = atan2(vt,(ve+0.0001));
        double thlf = atan2(vf,(ve+0.0001));

        std::vector<double> tip_vel = {sqrt(vf*vf + vd*vd), sqrt(vt*vt + vd*vd), sqrt(vt*vt + ve*ve), sqrt(vf*vf + ve*ve), thrf, thrb, thlb, thlf}; //rf, rb, lb, lf
    
        // Print the result
        // std::cout << "lateral_velociies:\n" << lat_vel << std::endl;
        for (int i=0; i<tip_vel.size(); i++){
            std::cout<<tip_vel[i]<<" ";
        }

        return tip_vel;
    }

    std::vector<double> bezier_params(double vel, double cp_max, double freq_min, double freq_max){
        int a=1;
        double cp = cp_max;
        double freq = freq_max;

        if (vel>freq_max*cp_max){
            cp = cp_max;
            freq = freq_max;
        }

        else{
            while(a==1){
                if (vel>=cp*(freq - freq_min) && vel<=cp*freq){
                    freq = freq+ 0.001;
                    cp = vel/freq;
                    a=0;
                } 
                else if(freq<=freq_min){
                    cp=cp+0.005;
                    freq= freq_max;
                    if(cp>cp_max){
                        cp=0;
                    }
                }
                else{
                    freq = freq - 0.001;
                }
            }
        }
        std::vector<double> params = {cp, freq};
        return params;
    }


    std::vector<double> freq_cp(double vel, double cp_max1, double freq_min1, double freq_max1){
     
        std::vector<double> params;
        double cp_c = 0, freq_c = freq_min1;

        if (vel>freq_max1*cp_max1){
            params.push_back(cp_max1);
            params.push_back(freq_max1);
            return params;
        }
        else{
            for (; freq_c < freq_max1; freq_c = freq_c + 0.01){
                cp_c = vel/freq_c;
                if (cp_c<= cp_max1 && freq_c<=freq_max1){
                    break;
                }
            }
        }
        params.push_back(cp_c);
        params.push_back(freq_c);
        RCLCPP_INFO(this->get_logger(), "frequency : %f, stride_length : %f", freq_c, cp_c);
        return params;
    }

    void teleop(std::vector<double> body_vel){

        RCLCPP_INFO(this->get_logger(), "teleop_called..");
        // Eigen::Vector3d P3(0.025, -0.054, -0.25);
        Eigen::Vector3d P3r(-0.015, -0.054, -0.23);
        Eigen::Vector3d P3l(0.06, -0.054, -0.23);
        std::vector<double> tip_vel = vel_ik(body_vel); //get tip velocities and directions {thetas!}
        //find max tip velocity!
        double max_vel = 0;
        int index=0;
        for (int i=0;i<4;i++){
            if(max_vel<tip_vel[i]){
                max_vel = tip_vel[i];
                index = i;
            }
        }
        
        std::vector<double> leg_bezier = freq_cp(tip_vel[index], cp_max, freq_min, freq_max);

        // std::vector<double> leg_bezier = bezier_params(tip_vel[index], cp_max, freq_min, freq_max); //get freq and cp for highest leg velocity!
        // std::vector<string> legs = {"rf", "rb", "lb", "lf"}//
        double d_rf = tip_vel[0]/leg_bezier[1];
        double d_rb = tip_vel[1]/leg_bezier[1];
        double d_lb = tip_vel[2]/leg_bezier[1];
        double d_lf = tip_vel[3]/leg_bezier[1];
        if(index==0){
            //rf is of mx velocity!
            // i know its step length and freq
            d_rf = leg_bezier[0];
        }
        else if(index==1){
            //rb is of mx velocity!
            // i know its step length and freq
            d_rb = leg_bezier[0];
        }
        else if(index==2){
            //lb is of mx velocity!
            // i know its step length and freq
            d_lb = leg_bezier[0];
        }
        else if(index==3){
            //lf is of mx velocity!
            // i know its step length and freq
            d_lf = leg_bezier[0];
        }

        B_rf = trajplanner(d_rf, h, tip_vel[4], P3r, beta_u, num_points);
        B_rb = trajplanner(d_rb, h, tip_vel[5], P3r, beta_u, num_points);
        B_lb = trajplanner(d_lb, h, tip_vel[6], P3l, beta_u, num_points);
        B_lf = trajplanner(d_lf, h, tip_vel[7], P3l, beta_u, num_points);

        RCLCPP_INFO(this->get_logger(), "Frequency : %f ", leg_bezier[1]);
        T = 1/leg_bezier[1]; //time period from frequency!

        sol_rf.clear();
        sol_rb.clear();
        sol_lb.clear();
        sol_lf.clear();

        //get ik solutions for each leg
        for (int i =0; i<B_rf.size();i++){

            sol_rf.push_back(ik_r(B_rf,i));
            sol_rb.push_back(ik_r(B_rb,i));

            sol_lb.push_back(ik_l(B_lb,B_rb.size() - i -1));
            sol_lf.push_back(ik_l(B_lf,B_rb.size() - i -1));
            // cout<<i<<endl;
        }

        RCLCPP_INFO(this->get_logger(), "got_ik_solutions ");
        int count = 0, c= 0;

        if (is_initial == 0){
            initialization(sol_rf, sol_rb, sol_lb, sol_lf, T, num_points);
            is_initial = 1;
        }
        RCLCPP_INFO(this->get_logger(), "Initialization_done ");

        for (int i = 0; i<sol_rf.size(); i++)
        {      
            c = 10+i;

            // #########################cycles###  RB ######################
            auto msg_rb = trajectory_msgs::msg::JointTrajectory();
            msg_rb.joint_names = {"rb1_base", "rb1_2", "rb2_3"};
    
            auto point_rb = trajectory_msgs::msg::JointTrajectoryPoint();
            point_rb.positions = {sol_rb[i][0], sol_rb[i][1], sol_rb[i][2]};  // Target joint positions

            builtin_interfaces::msg::Duration duration;
            duration.sec = 0;         // Full seconds part
            duration.nanosec = T/num_points * 1000000000;  // 0.5 seconds = 500 million nanoseconds
            point_rb.time_from_start = duration;
            msg_rb.points.push_back(point_rb);
            
            joint_traj_pub_rb->publish(msg_rb);

            // ############################  LF ######################
            auto msg_lf = trajectory_msgs::msg::JointTrajectory();
            msg_lf.joint_names = {"lf1_base", "lf1_2", "lf2_3"};
    
            auto point_lf = trajectory_msgs::msg::JointTrajectoryPoint();
            point_lf.positions = {sol_lf[i][0], sol_lf[i][1], sol_lf[i][2]}; // Target joint positions
            
            point_lf.time_from_start = duration;
            msg_lf.points.push_back(point_lf);

            joint_traj_pub_lf->publish(msg_lf);


            if (c>19){
                c = c-19;
            }
            // ############################  RF ######################

            auto msg_rf = trajectory_msgs::msg::JointTrajectory();
            msg_rf.joint_names = {"rf1_base", "rf1_2", "rf2_3"};
    
            auto point_rf = trajectory_msgs::msg::JointTrajectoryPoint();
            point_rf.positions = {sol_rf[c][0], sol_rf[c][1], sol_rf[c][2]};  // Target joint positions
            
            point_rf.time_from_start = duration;
            msg_rf.points.push_back(point_rf);
            
            joint_traj_pub_rf->publish(msg_rf);

            // ############################  LB ######################
            auto msg_lb = trajectory_msgs::msg::JointTrajectory();
            msg_lb.joint_names = {"lb1_base", "lb1_2", "lb2_3"};
    
            auto point_lb = trajectory_msgs::msg::JointTrajectoryPoint();
            point_lb.positions = {sol_lb[c][0], sol_lb[c][1], sol_lb[c][2]};  // Target joint positions
            
            point_lb.time_from_start = duration;
            msg_lb.points.push_back(point_lb);
            
            joint_traj_pub_lb->publish(msg_lb);
            
            RCLCPP_INFO(this->get_logger(), "Published joint trajectory");
        
            double time_to_wait = (T / num_points) * 1000;  // Convert to milliseconds
            rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(time_to_wait)));
        }
        // }
    }

    void initialization(std::vector<std::vector<double>> sol_rf, std::vector<std::vector<double>> sol_rb, std::vector<std::vector<double>> sol_lb, std::vector<std::vector<double>> sol_lf, double T, int numpoints ){
        //#################### initialization of RB ###################################
        auto msg_rb = trajectory_msgs::msg::JointTrajectory();
        msg_rb.joint_names = {"rb1_base", "rb1_2", "rb2_3"};

        auto point_rb = trajectory_msgs::msg::JointTrajectoryPoint();
        point_rb.positions = {sol_rb[0][0], sol_rb[0][1], sol_rb[0][2]};  // Target joint positions
        
        builtin_interfaces::msg::Duration duration;
        duration.sec = 0;         // Full seconds part
        duration.nanosec = T/num_points * 1000000000;  // 0.5 seconds = 500 million nanoseconds
        point_rb.time_from_start = duration;
        msg_rb.points.push_back(point_rb);
        
        joint_traj_pub_rb->publish(msg_rb);

        //#################### initialization of LF ###################################
        auto msg_lf = trajectory_msgs::msg::JointTrajectory();
        msg_lf.joint_names = {"lf1_base", "lf1_2", "lf2_3"};

        auto point_lf = trajectory_msgs::msg::JointTrajectoryPoint();
        point_lf.positions = {sol_lf[0][0], sol_lf[0][1], sol_lf[0][2]}; // Target joint positions
        
        point_lf.time_from_start = duration;
        msg_lf.points.push_back(point_lf);

        joint_traj_pub_lf->publish(msg_lf);

        //#################### initialization of RF ###################################
        auto msg_rf = trajectory_msgs::msg::JointTrajectory();
        msg_rf.joint_names = {"rf1_base", "rf1_2", "rf2_3"};

        auto point_rf = trajectory_msgs::msg::JointTrajectoryPoint();
        point_rf.positions = {sol_rf[10][0], sol_rf[10][1], sol_rf[10][2]};  // Target joint positions
        
        point_rf.time_from_start = duration;
        msg_rf.points.push_back(point_rf);
        
        joint_traj_pub_rf->publish(msg_rf);
        
        //#################### initialization of LB ###################################
        auto msg_lb = trajectory_msgs::msg::JointTrajectory();
        msg_lb.joint_names = {"lb1_base", "lb1_2", "lb2_3"};

        auto point_lb = trajectory_msgs::msg::JointTrajectoryPoint();
        point_lb.positions = {sol_lb[10][0], sol_lb[10][1], sol_lb[10][2]};  // Target joint positions
        
        point_lb.time_from_start = duration;
        msg_lb.points.push_back(point_lb);
        
        joint_traj_pub_lb->publish(msg_lb);
        
        double time_to_wait = (T / num_points) * 1000;  // Convert to milliseconds
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(time_to_wait)));
    }
    
    void grounded(){

        is_initial = 0;

        Eigen::Vector3d P3r(-0.015, -0.054, -0.23);
        Eigen::Vector3d P3l(0.06, -0.054, -0.23);


        std::vector<std::array<double, 3>> Pr = {{P3r[0], P3r[1], P3r[2]}}; 
        std::vector<std::array<double, 3>> Pl = {{P3l[0], P3l[1], P3l[2]}}; 

        // std::vector<std::array<double, 3>> Pl = {{-P3[0], P3[1], P3[2]}}; 

        std::vector<double> solution_r = ik_r(Pr, 0);
        std::vector<double> solution_l = ik_l(Pl, 0);

        //#################### initialization of RB ###################################
        auto msg_rb = trajectory_msgs::msg::JointTrajectory();
        msg_rb.joint_names = {"rb1_base", "rb1_2", "rb2_3"};

        auto point_rb = trajectory_msgs::msg::JointTrajectoryPoint();
        point_rb.positions = {solution_r[0], solution_r[1], solution_r[2]};  // Target joint positions
        
        builtin_interfaces::msg::Duration duration;
        duration.sec = 0;         // Full seconds part
        duration.nanosec = 0.2 * 1000000000;  // 0.5 seconds = 500 million nanoseconds
        point_rb.time_from_start = duration;
        msg_rb.points.push_back(point_rb);
        
        joint_traj_pub_rb->publish(msg_rb);

        //#################### initialization of LF ###################################
        auto msg_lf = trajectory_msgs::msg::JointTrajectory();
        msg_lf.joint_names = {"lf1_base", "lf1_2", "lf2_3"};

        auto point_lf = trajectory_msgs::msg::JointTrajectoryPoint();
        point_lf.positions = {solution_l[0], solution_l[1], solution_l[2]}; // Target joint positions
        
        point_lf.time_from_start = duration;
        msg_lf.points.push_back(point_lf);

        joint_traj_pub_lf->publish(msg_lf);

        //#################### initialization of RF ###################################
        auto msg_rf = trajectory_msgs::msg::JointTrajectory();
        msg_rf.joint_names = {"rf1_base", "rf1_2", "rf2_3"};

        auto point_rf = trajectory_msgs::msg::JointTrajectoryPoint();
        point_rf.positions = {solution_r[0], solution_r[1], solution_r[2]};  // Target joint positions
        
        point_rf.time_from_start = duration;
        msg_rf.points.push_back(point_rf);
        
        joint_traj_pub_rf->publish(msg_rf);
        
        //#################### initialization of LB ###################################
        auto msg_lb = trajectory_msgs::msg::JointTrajectory();
        msg_lb.joint_names = {"lb1_base", "lb1_2", "lb2_3"};

        auto point_lb = trajectory_msgs::msg::JointTrajectoryPoint();
        point_lb.positions = {solution_l[0], solution_l[1], solution_l[2]};  // Target joint positions
        
        point_lb.time_from_start = duration;
        msg_lb.points.push_back(point_lb);
        
        joint_traj_pub_lb->publish(msg_lb);
        
        double time_to_wait = (0.2) * 1000;  // Convert to milliseconds
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(time_to_wait)));
    }

    Matrix transposeMatrix(const Matrix& mat) {
        Matrix transposed(mat[0].size(), std::vector<double>(mat.size()));
        for (size_t i = 0; i < mat.size(); ++i) {
            for (size_t j = 0; j < mat[0].size(); ++j) {
                transposed[j][i] = mat[i][j];
            }
        }
        return transposed;
    }



    Matrix multiplyMatrices(const Matrix& A, const Matrix& B) {
        Matrix result(A.size(), std::vector<double>(B[0].size(), 0.0));
        for (size_t i = 0; i < A.size(); ++i) {
            for (size_t j = 0; j < B[0].size(); ++j) {
                for (size_t k = 0; k < A[0].size(); ++k) {
                    result[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return result;
    }


    Vector multiplyMatrixVector(const Matrix& mat, const Vector& vec) {
        Vector result(mat.size(), 0.0);
        for (size_t i = 0; i < mat.size(); ++i) {
            for (size_t j = 0; j < vec.size(); ++j) {
                result[i] += mat[i][j] * vec[j];
            }
        }
        return result;
    }


    Matrix dhTransform(double a, double alpha, double d, double theta) {
        Matrix T =  {{
            {cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a * cos(theta)},
            {sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)},
            {0,           sin(alpha),               cos(alpha),               d},
            {0,           0,                        0,                        1}
        }};
        return T;
    }

    Vector3 crossProduct(const Vector3& a, const Vector3& b) {
        return {
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        };
    }

    Vector3 subtractVectors(const Vector3& a, const Vector3& b) {
        return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    Matrix forwardKinematicsTransform(const std::vector<DHParameter>& dhParams) {
        Matrix T = {{
            {1, 0, 0, 0},
            {0, 1, 0, 0},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
        }};
        
        for (const auto& param : dhParams) {
            T = multiplyMatrices(T, dhTransform(param.a, param.alpha, param.d, param.theta));
        }
        return T; 
    }

        
    Cart_Pose forwardKinematics(const std::vector<DHParameter>& dhParams) {
        Matrix T = forwardKinematicsTransform(dhParams);

        Cart_Pose pose;
        pose.x = T[0][3];
        pose.y = T[1][3];
        pose.z = T[2][3];

        // Extracting roll, pitch, yaw from rotation matrix T
        pose.roll = atan2(T[2][1], T[2][2]);
        pose.pitch = atan2(-T[2][0], sqrt(T[2][1] * T[2][1] + T[2][2] * T[2][2]));//0
        pose.yaw = atan2(T[1][0], T[0][0]);
        return pose;
    }

    Matrix Inverse_Matrix(Matrix&T){
        Matrix T_inv = {{
            {1, 0, 0, 0},
            {0, 1, 0, 0},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
        }};
        Eigen::MatrixXd ans(4,4);
        for(int i =0; i<4;i++){
            for(int j=0;j<4;j++){
                ans(i,j)=T[i][j];
            }
        }
        // Code for pseudo-inverse calculation 
        Eigen::MatrixXd inverse = ans.inverse();
        for(int i =0; i<4;i++){
            for(int j=0;j<4;j++){
                T_inv[i][j]=inverse(i,j);
            }
        }
        return T_inv;
    }


    Matrix calculateJacobian(const std::vector<DHParameter>& dhParams) {
        std::vector<Matrix> transforms;
        Matrix T = {{
            {1, 0, 0, 0},
            {0, 1, 0, 0},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
        }};
        transforms.push_back(T);
        for (const auto& param : dhParams) {
            T = multiplyMatrices(T, dhTransform(param.a, param.alpha, param.d, param.theta));
            transforms.push_back(T);
        }
        transforms.pop_back();


        Vector3 p_e = {T[0][3], T[1][3], T[2][3]};
        std::vector<std::vector<double>> J(3, std::vector<double>(3, 0.0));


        for (int i = 0; i < 3; ++i) {
            Vector3 z_i = {transforms[i][0][2], transforms[i][1][2], transforms[i][2][2]};
            Vector3 p_i = {transforms[i][0][3], transforms[i][1][3], transforms[i][2][3]};
            Vector3 p_e_minus_p_i = subtractVectors(p_e, p_i);
            Vector3 J_v = crossProduct(z_i, p_e_minus_p_i);
                            // cout<<"hi"<<endl;///


            J[0][i] = J_v[0];
            J[1][i] = J_v[1];
            J[2][i] = J_v[2];

            // J[3][i] = z_i[0];
            // J[4][i] = z_i[1];
            // J[5][i] = z_i[2];
        }


        return J;
    }
    Matrix pseudoInverse(const Matrix& mat) {
        // pseudo inverse method is computationally time consuming so for real time applications like otg or cartesian jogging it has to be avoided so a new method is required
        Matrix U, V;
        Vector S;
        Matrix transposed = transposeMatrix(mat);
        Matrix ans1(mat.size(),std::vector<double>(mat.size()));
        Matrix ans2(mat.size(),std::vector<double>(mat.size()));
        Matrix result(mat[0].size(), Vector(mat.size()));

        // Pseudo-inverse calculation
    //    ans1 =  multiplyMatrices(mat,transposed);
        ans1 = mat;
        Eigen::MatrixXd ans(ans1.size(),ans1.size());
        for(int i =0; i<ans1.size();i++){
            for(int j=0;j<ans1.size();j++){
                ans(i,j)=ans1[i][j];
            }
        }
        // Code for pseudo-inverse calculation 
        Eigen::MatrixXd inverse = ans.inverse();
        for(int i =0; i<ans2.size();i++){
            for(int j=0;j<ans2.size();j++){
                ans1[i][j]=inverse(i,j);
            }
        }
        // result = multiplyMatrices(transposed,ans1);
        result = ans1;
        return result;
    }

    std::vector<double> linspace(double start, double end, size_t num_points)
    {
        std::vector<double> result(num_points);
        double step = (end - start) / (num_points - 1);

        for (size_t i = 0; i < num_points; ++i) {
            result[i] = start + i * step;
        }

        return result;
    }

    double factorial(int n) {
        if (n == 0 || n == 1) return 1;
        return n * factorial(n - 1);
    }

   int nCr(int n, int k) {
        if (k == 0 || k == n) return 1;
        int res = 1;
        for (int i = 1; i <= k; i++) {
            res *= (n - (k - i));
            res /= i;
        }
        return res;
    }


    std::vector<std::array<double, 3>> trajplanner(double d, double h, double theta, Eigen::Vector3d P3, double beta_u, double num_points){
        RCLCPP_INFO(this->get_logger(), "trajplanning started");

        // % Compute step displacement components
        double dx = d * 0.7 * cos(theta);
        double dy = d * 0.7 * sin(theta);


        // Define control points
        Eigen::Vector3d P0 = {P3(0), P3(1), P3(2) + h};
        Eigen::Vector3d P6 = P0;
        Eigen::Vector3d P1 = {P3(0) + (4.0 / 5.0) * dx, P3(1) + (4.0 / 5.0) * dy, P3(2) + (3.0 / 5.0) * h};
        Eigen::Vector3d P2 = {P3(0) + (5.0 / 5.0) * dx, P3(1) + (5.0 / 5.0) * dy, P3(2) + (1.0 / 5.0) * h};
        Eigen::Vector3d P4 = {P3(0) - (5.0 / 5.0) * dx, P3(1) - (5.0 / 5.0) * dy, P3(2) + (1.0 / 5.0) * h};
        Eigen::Vector3d P5 = {P3(0) - (4.0 / 5.0) * dx, P3(1) - (4.0 / 5.0) * dy, P3(2) + (3.0 / 5.0) * h};

        // std::vector<double> u_values = linspace(0.0, 1.0, 7); beta_u = 0.5
        std::vector<double> u_values = {0.0, (1-beta_u)/4, (1-beta_u)/2, 0.5, (1+beta_u)/2, (3+beta_u)/4, 1.0};

        Eigen::MatrixXd V = Eigen::MatrixXd::Zero(7, 7);


        for (int j = 0; j < 7; j++) {  // j corresponds to MATLAB row index
            double u = u_values[j];
            for (int i = 0; i <= 6; i++) {  // i corresponds to MATLAB column index
                V(j, i) = nCr(6, i) * pow((1 - u), (6 - i)) * pow(u, i);
            }
        }

        // Print the matrix
        // std::cout << "V Matrix:\n" << V << std::endl;    

        Eigen::MatrixXd P(7, 3);
        P << P0.transpose(),
            P1.transpose(),
            P2.transpose(),
            P3.transpose(),
            P4.transpose(),
            P5.transpose(),
            P6.transpose();

        Eigen::MatrixXd W = (V.transpose() * V).colPivHouseholderQr().solve(V.transpose() * P);
        // std::cout << "W Matrix:\n" << W << std::endl;
        // cout<<" in to the func"<<endl;

        return bezier_curve_3d(W, num_points);

    }

    std::vector<std::array<double, 3>> bezier_curve_3d(Eigen::MatrixXd W, int num_points){
        std::vector<double> u_vals = linspace(0.0, 1.0, num_points);
        std::vector<std::array<double, 3>> B(num_points);
        for (int i =0; i<num_points;i++){
            double u = u_vals[i];
            Vector3 B_point = {0.0, 0.0, 0.0};
            for (int j = 0; j<7; j++){
                double bern_coef = nCr(6, j) * pow((1-u), (6 - j)) * pow(u,j);
                // cout<<W[5][0]<<" W50"<<endl;
                // cout<<W[5][0]<<" W50"<<endl;
                // break;
                Vector3 W_row = { bern_coef*W(j,0), bern_coef*W(j,1), bern_coef*W(j,2) };
                B_point[0] = B_point[0] + W_row[0];
                B_point[1] = B_point[1] + W_row[1];
                B_point[2] = B_point[2] + W_row[2];
                // cout<< j <<  " loop no"<<endl;
            }
            B[i][0] = B_point[0];
            B[i][1] = B_point[1];
            B[i][2] = B_point[2];

            cout<< B[i][0] << " " << B[i][1] << " " << B[i][2] <<endl;
            // cout<< "func completed"<<endl;
        }

        return B;        
    }

private:
    
    // Member variables
    double l =0.26, c = 0.32;
    double num_points = 20;
    double beta_u = 0.6;
    double T = 1.0;
    int cycles = 15;
    double d, h, theta, cp_max=0.10, freq_max=1.66, freq_min=0.16; 
    double max_v = 0.16, max_w = 0.15; //change the values
    int flag = 0;
    // Eigen::Vector3d P3(0.025, -0.054, -0.25);
    std::vector<std::vector<double>> sol_r;
    std::vector<std::vector<double>> sol_l;
    std::vector<std::vector<double>> sol_rf;
    std::vector<std::vector<double>> sol_rb;
    std::vector<std::vector<double>> sol_lb;
    std::vector<std::vector<double>> sol_lf;

    std::vector<std::thread> threads_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub_rb;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub_lf;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub_lb;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub_rf;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub;

    rclcpp::Client<custom_service::srv::IkSw>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t current_index;
    std::vector<std::array<double, 3>> B;
    std::vector<std::array<double, 3>> Br;
    std::vector<std::array<double, 3>> Bl;
    std::vector<std::array<double, 3>> B_rb;
    std::vector<std::array<double, 3>> B_rf;
    std::vector<std::array<double, 3>> B_lb;
    std::vector<std::array<double, 3>> B_lf;

    std::vector<double> robot_vel;
    int is_initial = 0;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  

};

int main(int argc, char** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create an instance of the TrajBezier node
    auto node = std::make_shared<TrajBezier>();

    // Spin to keep the node alive
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
