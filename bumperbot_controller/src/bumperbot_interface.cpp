#include "bumperbot_controller/bumperbot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace bumperbot_controller
{
    RobotInterface::RobotInterface() //constructor name of class::name of function
    {

    }
    RobotInterface::~RobotInterface()
    {
        if(arduino_.IsOpen()){
            try{
                arduino_.Close();
            }
            catch(...){
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotInterface"), "Something went wrong while closing the connection"<<port_);

            }
        }
    }
    CallbackReturn RobotInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
    {
        // lets call the on_init function of the base class hardware-interface
        // it receives hardware_info and returns result of type callbackreturn
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);

        if (result != CallbackReturn::SUCCESS)
        {
            return result;
            // result fails
        }
        // if base class was succesful then we can initialize serial comm of arduino!
        try{
            port_ = info_.hardware_parameters.at("port");
            // info will come from base class and contains the parameter!
        }
        catch(const std::out_of_range &e){
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotInterface"), "No serial port provided");
            return CallbackReturn::FAILURE;

        }
        position_commands_.reserve(info_.joints.size());
        RCLCPP_INFO(rclcpp::get_logger("robotInterface"), "joint_size: %d", info_.joints.size());
        position_states_.reserve(info_.joints.size());
        prev_position_commands_.reserve(info_.joints.size());

        return CallbackReturn::SUCCESS;
    }
    std::vector<hardware_interface::StateInterface> RobotInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces; 
        // used to communicate the current status of motors
        for(size_t i =0; i<info_.joints.size(); i++){
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RobotInterface::export_command_interfaces(){
        // sending commands to robot motors
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for(size_t i =0; i<info_.joints.size(); i++){
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
        }
        return command_interfaces;
    }
    CallbackReturn RobotInterface::on_activate(const rclcpp_lifecycle::State & previous_state) {
        RCLCPP_INFO(rclcpp::get_logger("robotInterface"), "starting robot hardware...");
        position_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        prev_position_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,};
        position_states_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // assuming initial cmds are 0

        try{
            arduino_.Open(port_);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        catch(...){
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotInterface"), "something went wrong while opening port"<<port_);
            return CallbackReturn::FAILURE;
        }
        RCLCPP_INFO(rclcpp::get_logger("robotInterface"), "hardware started, ready to take cmds!");
        return CallbackReturn::SUCCESS;

    }
    CallbackReturn RobotInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
        RCLCPP_INFO(rclcpp::get_logger("robotInterface"), "Stopping robot hardware !");
        if(arduino_.IsOpen()){
            try{
                arduino_.Close();
            }
            catch(...){
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotInterface"), "something went wrong while closing port"<<port_);
                return CallbackReturn::FAILURE;
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("robotInterface"), "hardware stopped!");
        return CallbackReturn::SUCCESS;
    }
    hardware_interface::return_type RobotInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // reading current position of joints and making it available in ros2

        position_states_  = position_commands_;

        // as there is no position feedback so lets assume motor goes at whats commanded!
        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type RobotInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period) {
        // send cmds to arduino!
        // check for repetitive same cmds!
        if (position_commands_ == prev_position_commands_){
            return hardware_interface::return_type::OK;
        }
        // if its a new cmd
        std::string msg;
        /////// --- LB ---  ///////////
        int th9 = static_cast<int>((position_commands_.at(0))*180)/3.1415;
        int th10 = static_cast<int>((position_commands_.at(1))*180)/3.1415;
        int th11 = static_cast<int>((position_commands_.at(2))*180)/3.1415;

        ///////// --- LF ---  ///////////
        int th3 = static_cast<int>((position_commands_.at(3))*180)/3.1415;
        int th4 = static_cast<int>((position_commands_.at(4))*180)/3.1415;
        int th5 = static_cast<int>((position_commands_.at(5))*180)/3.1415;

        ///////// --- RB ---  ///////////
        int th6 = static_cast<int>((position_commands_.at(6))*180)/3.1415;
        int th7 = static_cast<int>((position_commands_.at(7))*180)/3.1415;
        int th8 = static_cast<int>((position_commands_.at(8))*180)/3.1415;

        ///////// --- RF ---  ///////////
        int th0 = static_cast<int>((position_commands_.at(9))*180)/3.1415;
        int th1 = static_cast<int>((position_commands_.at(10))*180)/3.1415;
        int th2 = static_cast<int>((position_commands_.at(11))*180)/3.1415;

        RCLCPP_INFO(rclcpp::get_logger("robotInterface"), "th9 th10 th11: %d %d %d , th3 th4 th5: %d %d %d , th6 th7 th8 %d %d %d , th0 th1 th2: %d %d %d",th9, th10, th11, th3, th4, th5, th6, th7, th8, th0, th1, th2);
        msg.append("mo_");
        msg.append(std::to_string(th9));
        msg.append("_");
        msg.append(std::to_string(th10));
        msg.append("_");
        msg.append(std::to_string(th11));
        msg.append("_");
        msg.append(std::to_string(th3));
        msg.append("_");
        msg.append(std::to_string(th4));
        msg.append("_");
        msg.append(std::to_string(th5));
        msg.append("_");
        msg.append(std::to_string(th6));
        msg.append("_");
        msg.append(std::to_string(th7));
        msg.append("_");
        msg.append(std::to_string(th8));
        msg.append("_");
        msg.append(std::to_string(th0));
        msg.append("_");
        msg.append(std::to_string(th1));
        msg.append("_");
        msg.append(std::to_string(th2));
        msg.append("\r");
        // RCLCPP_INFO(rclcpp::get_logger("robotInterface"), "msg is going");
        try{
            arduino_.FlushIOBuffers();
            arduino_.Write(msg);
        }
        catch(...){
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotInterface"), "something went wrong while sending the message"<<msg);
            return hardware_interface::return_type::ERROR;
        }
        prev_position_commands_ = position_commands_;
        return hardware_interface::return_type::OK;
    }

}
PLUGINLIB_EXPORT_CLASS(bumperbot_controller::RobotInterface, hardware_interface::SystemInterface);
// you need to mention class and base class inherited both
// convert to plugin!