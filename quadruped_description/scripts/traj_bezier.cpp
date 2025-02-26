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

        joint_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_cont/joint_trajectory", 10);
        client_ = this->create_client<custom_service::srv::IkSw>("ik_service");
        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_marker_array", 10);
        // timer_ = this->create_wall_timer(
        //     std::chrono::seconds(1), std::bind(&TrajectoryPublisher::publish_trajectory, this));

        double d = 0.12;
        double h = 0.06;
        double theta = 0.0;  // Example angle in radians (45 degrees)
        Eigen::Vector3d P3(0.025, -0.054, -0.25);
        // Eigen::Vector3d P3(0.15, -0.054, 0.03);
        B = trajplanner(d, h, theta, P3); 

        for (int i =0; i<B.size();i++){

            init(B,i);
            cout<<i<<endl;
        }
    }
    void init( std::vector<std::array<double, 3>> B, int current_index)
    {
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to be available...");
        }

        std_msgs::msg::Float32MultiArray transformation;
            transformation.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
            transformation.layout.dim[0].label = "dim1";  // Label for the dimension
            transformation.layout.dim[0].size = 3;      // Size of the dimension
            transformation.layout.dim[0].stride = 1;     // Stride for the dimension
            transformation.layout.data_offset = 0;  
        transformation.data = {-B[current_index][2], B[current_index][1], B[current_index][0]};

        auto request = std::make_shared<custom_service::srv::IkSw::Request>();
        request->transformation = transformation;
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

            // pub_for moving
            // for ( int i=0; i<theta_vec.size();i++){
            //     auto joint_traj = trajectory_msgs::msg::JointTrajectory();
            //     joint_traj.header.stamp = this->get_clock()->now();
                
            //     joint_traj.joint_names = {"base_link1_joint", "link1_link2_joint", "link2_link3_joint"};

            //     trajectory_msgs::msg::JointTrajectoryPoint point;
            //     point.positions = {0.2, 0.3, (0.1+((double) i)/10)};
            //     point.velocities = {0.0, 0.0, 0.0};
            //     point.accelerations = {0.0, 0.0, 0.0};

            //     // Correct way to set time_from_start
            //     point.time_from_start.sec = 1;
            //     point.time_from_start.nanosec = 0;

            //     joint_traj.points.push_back(point);

            //     // Publish the trajectory
            //     joint_traj_pub_->publish(joint_traj);
            //     RCLCPP_INFO(this->get_logger(), "Published joint trajectory");
            //     RCLCPP_INFO(this->get_logger(), "Sleeping for 1 seconds...");
            //     rclcpp::sleep_for(std::chrono::seconds(1));  // Sleep for 2 seconds
            //     RCLCPP_INFO(this->get_logger(), "Resuming execution after sleep.");

            // }
            // RCLCPP_INFO(this->get_logger(), "Resuming execution after sleep.");
            auto msg = trajectory_msgs::msg::JointTrajectory();
        
            // List of joint names
            msg.joint_names = {"base_link1_joint", "link1_link2_joint", "link2_link3_joint"};
    
            auto point = trajectory_msgs::msg::JointTrajectoryPoint();
            point.positions = {ang[0], ang[1], ang[2]};  // Target joint positions
            
            // Set the duration to 1 second
            builtin_interfaces::msg::Duration duration;
            duration.sec = 0;         // Full seconds part
            duration.nanosec = 200000000;  // 0.5 seconds = 500 million nanoseconds
            point.time_from_start = duration;
            
            msg.points.push_back(point);
            // ******************* marker code ******************
            visualization_msgs::msg::MarkerArray marker_array;
            std::vector<double> points = { -B[current_index][2], B[current_index][1], B[current_index][0]};

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "dh_ref_base";  // Change to "odom" or "base_link" if needed
            marker.header.stamp = this->now();
            marker.ns = "marker_points";
            marker.id = current_index;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = points[0];
            marker.pose.position.y = points[1];
            marker.pose.position.z = points[2];

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
            RCLCPP_INFO(this->get_logger(), "Published %ld marker points", points.size());

            // Publish the trajectory
            joint_traj_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published joint trajectory");
           
            // RCLCPP_INFO(this->get_logger(), "Sle     eping for 1 seconds...");
            rclcpp::sleep_for(std::chrono::milliseconds(200));  // 500 milliseconds = 0.5 seconds

        }
         else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

    void pub(std::vector<std::vector<double>> theta_vec){
        
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

    std::vector<std::array<double, 3>> trajplanner(double d, double h, double theta, Eigen::Vector3d P3){
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

        std::vector<double> u_values = linspace(0.0, 1.0, 7);

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
        int num_points = 20;
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
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub_;
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