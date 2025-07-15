#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SimpleSerialTransmitter(Node):
    def __init__(self):
        super().__init__("simple_serial_transmitter")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", "115200")

        self.port_ = self.get_parameter("port").value
        self.baud_rate_ = self.get_parameter("baud_rate").value

        self.subscriber_ = self.create_subscription(String, "serial_transmitter", self.data_callback,10)
        self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baud_rate_, timeout=0.1)

    def data_callback(self,msg):
        data = msg.data
        self.arduino_.write(msg.data.encode("utf-8"))
    
def main():
    rclpy.init()
    node = SimpleSerialTransmitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()