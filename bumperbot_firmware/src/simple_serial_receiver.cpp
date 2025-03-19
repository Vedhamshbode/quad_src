#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <libserial/SerialPort.h>

using namespace std::chrono_literals;

class SimpleSerialReceiver : public rclcpp::Node
{
    public:
        SimpleSerialReceiver() : Node("simple_serial_receiver") //Constructor
        {
            declare_parameter<std::string>("port", "/dev/ttyACM1");
            std::string port_ = get_parameter("port").as_string();
            pub_ =  create_publisher<std_msgs::msg::String>("serial_receiver",10); //create publisher and assign to shared pointer
            timer_ = create_wall_timer(0.01s, std::bind(&SimpleSerialReceiver::timer_callback,this)); //create timer callback 
            arduino_.Open(port_);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        ~SimpleSerialReceiver(){
            arduino_.Close();
        }

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_; //create a shared pointer of the given data type for publisher
        rclcpp::TimerBase::SharedPtr timer_;  //create a shared pointer for the timer
        LibSerial::SerialPort arduino_;

        void timer_callback(){
            auto message = std_msgs::msg::String();

            if (rclcpp::ok() && arduino_.IsDataAvailable()){
                arduino_.ReadLine(message.data);
            }
            pub_->publish(message);    // publishing the data through shared pointer of publisher
        }
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleSerialReceiver>();  //create a shared pointer of class obj
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}