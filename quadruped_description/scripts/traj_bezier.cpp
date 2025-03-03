#include "rclcpp/rclcpp.hpp"
#include "custom_service/srv/ik_sw.hpp"
#include <memory>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;
using namespace std;
using Matrix4x4 = std::array<std::array<double, 4>, 4>;
using Vector3 = std::array<double, 3>;
using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;

class TrajBezier : public rclcpp::Node
{
public:
    TrajBezier() : Node("traj_node")
    {
        std_msgs::msg::Float32MultiArray transformation;

        joint_traj_pub_rb = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/rb_cont/joint_trajectory", 10);
        joint_traj_pub_lf = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/lf_cont/joint_trajectory", 10);
        joint_traj_pub_lb = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/lb_cont/joint_trajectory", 10);
        joint_traj_pub_rf = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/rf_cont/joint_trajectory", 10);

        client_ = this->create_client<custom_service::srv::IkSw>("ik_service");
        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_marker_array", 10);
        // timer_ = this->create_wall_timer(
        //     std::chrono::seconds(1), std::bind(&TrajectoryPublisher::publish_trajectory, this));

        double d = 0.12;
        double h = 0.05;
        double theta = 0.0;  // Example angle in radians (45 degrees)
        double num_of_points = 20;
        double T = 4.0;
        double beta_u = 0.6;
        int cycles = 6;
        Eigen::Vector3d P3(0.025, -0.054, -0.25);
        // Eigen::Vector3d P3(0.15, -0.054, 0.03);
        B = trajplanner(d, h, theta, P3, beta_u, num_of_points); 
        std::vector<std::vector<double>> sol_r;
        std::vector<std::vector<double>> sol_l;
        marker_pub(B, "dh_ref_lf");

        for (int i =0; i<B.size();i++){

            sol_r.push_back(ik_r(B,i));
            sol_l.push_back(ik_l(B,B.size() - i -1));
            // pubjangles(sol_r[i], {"rb1_base", "rb1_2", "rb2_3"}, T, num_of_points, "rb");
            cout<<i<<endl;
        }
        trot_gait(sol_r, sol_l, T, num_of_points, beta_u,cycles);
        
    }
    std::vector<double> ik_r( std::vector<std::array<double, 3>> B, int current_index)
    {
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service. Exiting.");
                return {0.0,0.0,0.0};
                
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to be available...");
        }
        // ********************************* right back leg data ****************************

        std_msgs::msg::Float32MultiArray transformation;
        transformation.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        transformation.layout.dim[0].label = "dim1";  // Label for the dimension
        transformation.layout.dim[0].size = 3;      // Size of the dimension
        transformation.layout.dim[0].stride = 1;     // Stride for the dimension
        transformation.layout.data_offset = 0;  
        transformation.data = {-B[current_index][2], B[current_index][1], B[current_index][0]};
        
        // ********************************* right back leg data request ****************************

        auto request = std::make_shared<custom_service::srv::IkSw::Request>();
        request->transformation = transformation;
        request->leg = "right";
        auto future = client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Result: ");
            auto res = future.get()->jangles;
            std::vector<std::vector<double>> theta_vec;
            std::vector<double> ang;    

            for (int i = 0;i<3;i++){
                ang.push_back(res.data[i]);
            }
            theta_vec.push_back(ang);
            cout<<"ang "<<ang[0]<<" "<<ang[1]<<" "<<ang[2]<<endl;
            return ang;
        }
         else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

    std::vector<double> ik_l( std::vector<std::array<double, 3>> B, int current_index)
    {
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service. Exiting.");
                return {0.0,0.0,0.0};
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to be available...");
        }
        // ********************************* right back leg data ****************************

        std_msgs::msg::Float32MultiArray transformation;
        transformation.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        transformation.layout.dim[0].label = "dim1";  // Label for the dimension
        transformation.layout.dim[0].size = 3;      // Size of the dimension
        transformation.layout.dim[0].stride = 1;     // Stride for the dimension
        transformation.layout.data_offset = 0;  
        transformation.data = {-B[current_index][2], B[current_index][1], B[current_index][0]};
        // std::vector<double> pts = {-B[current_index][2], -B[current_index][1], -B[current_index][0]};
        // ********************************* right back leg data request ****************************

        auto request = std::make_shared<custom_service::srv::IkSw::Request>();
        request->transformation = transformation;
        request->leg = "left";
        auto future = client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Result: ");
            auto res = future.get()->jangles;
            std::vector<std::vector<double>> theta_vec;
            std::vector<double> ang;    

            for (int i = 0;i<3;i++){
                ang.push_back(res.data[i]);
            }
            theta_vec.push_back(ang);
            // cout<<"ang "<<ang[0]<<" "<<ang[1]<<" "<<ang[2]<<endl;
            return ang;
        }
         else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
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

    void trot_gait(std::vector<std::vector<double>> sol_r, std::vector<std::vector<double>> sol_l, double T, double num_points, double beta_u, int cycles){
        auto start_time = std::chrono::high_resolution_clock::now();
        int count = 0, c= 0;
        // double shift_time = (T- beta_u*T);
        double shift_time = 2.0;

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

                auto end_time = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed_time = end_time - start_time;


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


    Matrix transposeMatrix(const Matrix& mat) {
        Matrix transposed(mat[0].size(), std::vector<double>(mat.size()));
        for (size_t i = 0; i < mat.size(); ++i) {
            for (size_t j = 0; j < mat[0].size(); ++j) {
                transposed[j][i] = mat[i][j];
            }
        }
        return transposed;
    }

    Matrix Inverse_Matrix(Matrix&T){
        Matrix T_inv = {{
            {1, 0, 0, 0, 0, 0, 0},
            {0, 1, 0, 0, 0, 0, 0},
            {0, 0, 1, 0, 0, 0, 0},
            {0, 0, 0, 1, 0, 0, 0},
            {0, 0, 0, 0, 1, 0, 0},
            {0, 0, 0, 0, 0, 1, 0},
            {0, 0, 0, 0, 0, 0, 1}
        }};
        Eigen::MatrixXd ans(7,7);
        for(int i =0; i<7;i++){
            for(int j=0;j<7;j++){
                ans(i,j)=T[i][j];
            }
        }
        // Code for pseudo-inverse calculation 
        Eigen::MatrixXd inverse = ans.completeOrthogonalDecomposition().pseudoInverse();
        for(int i =0; i<7;i++){
            for(int j=0;j<7;j++){
                T_inv[i][j]=inverse(i,j);
            }
        }
        return T_inv;
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

    // void CallIkService(std_msgs::msg::Float32MultiArray transformation)
    // {
    //     auto request = std::make_shared<custom_service::srv::IkSw::Request>();
    //     request->transformation = transformation;
    //     auto future = client_->async_send_request(request);
    //     cout<<"before try"<<endl;

    //     try {
    //         auto response = future.get();
    //         std_msgs::msg::Float32MultiArray jpose = response->jangles;
    //         cout<<"inside try"<<endl;

    //         // std::cout << "jpose: " << jpose.data[0] << " " << jpose.data[1] << " " << jpose.data[2] << std::endl;
    //         auto joint_traj = trajectory_msgs::msg::JointTrajectory();
    //         joint_traj.header.stamp = this->get_clock()->now();
            
    //         joint_traj.joint_names = {"base_link1_joint", "link1_link2_joint", "link2_link3_joint"};

    //         trajectory_msgs::msg::JointTrajectoryPoint point;
    //         point.positions = {jpose.data[0], jpose.data[1], jpose.data[2]};
    //         point.velocities = {0.0, 0.0, 0.0};
    //         point.accelerations = {0.0, 0.0, 0.0};

    //         // Correct way to set time_from_start
    //         point.time_from_start.sec = 1;
    //         point.time_from_start.nanosec = 0;

    //         joint_traj.points.push_back(point);

    //         // Publish the trajectory
    //         joint_traj_pub_->publish(joint_traj);
    //         RCLCPP_INFO(this->get_logger(), "Published joint trajectory");

    //         // Wait for motion to complete
    //         // rclcpp::Time start_time = this->get_clock()->now();
    //         // rclcpp::Time end_time = start_time + rclcpp::Duration::from_seconds(1.0);

    //         // while (rclcpp::ok() && this->get_clock()->now() < end_time) {
    //         //     rclcpp::spin_some(*this);
    //         // }
    //         std::this_thread::sleep_for(std::chrono::seconds(1));

    //     } catch (const std::exception &e) {
    //         RCLCPP_ERROR(this->get_logger(), "Service call failed");
    //     }
    // }

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

        // Matrix P = {{P0(0), P0(1), P0(2)}, 
        //             {P1(0), P1(1), P1(2)},
        //             {P2(0), P2(1), P2(2)}, 
        //             {P3(0), P3(1), P3(2)}, 
        //             {P4(0), P4(1), P4(2)}, 
        //             {P5(0), P5(1), P5(2)}, 
        //             {P6(0), P6(1), P6(2)}};

        // std::vector<double> u_values = linspace(0.0, 1.0, 7); beta_u = 0.5
        std::vector<double> u_values = {0.0, (1-beta_u)/4, (1-beta_u)/2, 0.5, (1+beta_u)/2, (3+beta_u)/4, 1.0};

        // Matrix V = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
        //             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
        //             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
        //             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
        //             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
        //             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
        //             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
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
    double d, h, theta, P3; 
    std::vector<std::thread> threads_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub_rb;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub_lf;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub_lb;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub_rf;

    rclcpp::Client<custom_service::srv::IkSw>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t current_index;
    std::vector<std::array<double, 3>> B;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;

};

int main(int argc, char** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create an instance of the TrajBezier node
    auto node = std::make_shared<TrajBezier>();

    // Spin to keep the node alive
    // rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}