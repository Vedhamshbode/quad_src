#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros

class StaticTf(Node):
    def __init__(self):
        super().__init__('static_tf')
        self.br = tf2_ros.StaticTransformBroadcaster(self)   

        b_tf = TransformStamped()
        b_tf.header.stamp = self.get_clock().now().to_msg()
        b_tf.header.frame_id = "link1_1"
        b_tf.child_frame_id = "dh_ref_base"
        b_tf.transform.translation.x = 0.0
        b_tf.transform.translation.y = 0.0
        b_tf.transform.translation.z = 0.0
        b_tf.transform.rotation.x = 0.0
        b_tf.transform.rotation.y = 0.0
        b_tf.transform.rotation.z = 1.0  
        b_tf.transform.rotation.w = 0.0
        self.br.sendTransform(b_tf) 

def main(args=None):
    rclpy.init(args=args)
    static_tf = StaticTf()
    # rclpy.spin(static_tf)
    static_tf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

