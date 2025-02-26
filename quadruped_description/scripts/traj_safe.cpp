#include "rclcpp/rclcpp.hpp"
#include "custom_service/srv/ik_sw.hpp"
#include <memory>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "builtin_interfaces/msg/duration.hpp"

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
        // Create the transformation message
        std_msgs::msg::Float32MultiArray transformation;

        joint_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_cont/joint_trajectory", 10);

        client = this->create_client<custom_service::srv::IkSw>("ik_service");

        while (!client->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for the service to start");
        }

        double d = 0.15;
        double h = 0.07;
        double theta = 0.0;  // Example angle in radians (45 degrees)
        Eigen::Vector3d P3(0.195, -0.054, 0.0254);
        std::vector<std::array<double, 3>> B = trajplanner(d, h, theta, P3); 

        for (int i =0; i<B.size();i++){

            cout<<i<<endl;
            transformation.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
            transformation.layout.dim[0].label = "dim1";  // Label for the dimension
            transformation.layout.dim[0].size = 12;      // Size of the dimension
            transformation.layout.dim[0].stride = 1;     // Stride for the dimension
            transformation.layout.data_offset = 0;  
            double theta = 0.348;
            transformation.data = {sin(theta), -cos(theta), 0.0, B[i][0], 0.0, 0.0, -1.0, B[i][1], cos(theta), sin(theta), 0.0, B[i][2]};
            threads_.push_back(std::thread(&TrajBezier::CallIkService, this, transformation));
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

    void CallIkService(std_msgs::msg::Float32MultiArray transformation)
    {
        auto request = std::make_shared<custom_service::srv::IkSw::Request>();
        request->transformation = transformation;
        auto future = client->async_send_request(request);
        cout<<"before try"<<endl;

        try {
            auto response = future.get();
            std_msgs::msg::Float32MultiArray jpose = response->jangles;
            cout<<"inside try"<<endl;

            // std::cout << "jpose: " << jpose.data[0] << " " << jpose.data[1] << " " << jpose.data[2] << std::endl;
            auto joint_traj = trajectory_msgs::msg::JointTrajectory();
            joint_traj.header.stamp = this->get_clock()->now();
            
            joint_traj.joint_names = {"base_link1_joint", "link1_link2_joint", "link2_link3_joint"};

            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = {jpose.data[0], jpose.data[1], jpose.data[2]};
            point.velocities = {0.0, 0.0, 0.0};
            point.accelerations = {0.0, 0.0, 0.0};

            // Correct way to set time_from_start
            point.time_from_start.sec = 1;
            point.time_from_start.nanosec = 0;

            joint_traj.points.push_back(point);

            // Publish the trajectory
            joint_traj_pub_->publish(joint_traj);
            RCLCPP_INFO(this->get_logger(), "Published joint trajectory");

            // Wait for motion to complete
            // rclcpp::Time start_time = this->get_clock()->now();
            // rclcpp::Time end_time = start_time + rclcpp::Duration::from_seconds(1.0);

            // while (rclcpp::ok() && this->get_clock()->now() < end_time) {
            //     rclcpp::spin_some(*this);
            // }
            std::this_thread::sleep_for(std::chrono::seconds(1));

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

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
        std::cout << "V Matrix:\n" << V << std::endl;

        Eigen::MatrixXd P(7, 3);
        P << P0.transpose(),
            P1.transpose(),
            P2.transpose(),
            P3.transpose(),
            P4.transpose(),
            P5.transpose(),
            P6.transpose();

        Eigen::MatrixXd W = (V.transpose() * V).colPivHouseholderQr().solve(V.transpose() * P);
        std::cout << "W Matrix:\n" << W << std::endl;
        int num_points = 10;
        cout<<" in to the func"<<endl;
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
            cout<< "func completed"<<endl;
        }
        return B;        
    }

private:
    std::vector<std::thread> threads_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub_;
    rclcpp::Client<custom_service::srv::IkSw>::SharedPtr client;
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