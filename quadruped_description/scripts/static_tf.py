#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf_transformations


class StaticTf(Node):
    def __init__(self):
        super().__init__('static_tf')
        self.br_rb = tf2_ros.StaticTransformBroadcaster(self)   
        self.br_rf = tf2_ros.StaticTransformBroadcaster(self)   
        self.br_lb = tf2_ros.StaticTransformBroadcaster(self)   
        self.br_lf = tf2_ros.StaticTransformBroadcaster(self)   

        q_rb = tf_transformations.quaternion_from_euler(3.1416, 1.5708, 0.0)


        rb_tf = TransformStamped()
        rb_tf.header.stamp = self.get_clock().now().to_msg()
        rb_tf.header.frame_id = "base_link"
        rb_tf.child_frame_id = "dh_ref_rb"
        rb_tf.transform.translation.x = 0.159
        rb_tf.transform.translation.y = 0.131
        rb_tf.transform.translation.z = 0.359
        rb_tf.transform.rotation.x = q_rb[0]
        rb_tf.transform.rotation.y = q_rb[1]
        rb_tf.transform.rotation.z = q_rb[2]  
        rb_tf.transform.rotation.w = q_rb[3]
        # *****************************************

        q_rf = tf_transformations.quaternion_from_euler(3.1416, 1.5708, 0.0)


        rf_tf = TransformStamped()
        rf_tf.header.stamp = self.get_clock().now().to_msg()
        rf_tf.header.frame_id = "base_link"
        rf_tf.child_frame_id = "dh_ref_rf"
        rf_tf.transform.translation.x = -0.0859
        rf_tf.transform.translation.y = 0.131
        rf_tf.transform.translation.z = 0.359
        rf_tf.transform.rotation.x = q_rf[0]
        rf_tf.transform.rotation.y = q_rf[1]
        rf_tf.transform.rotation.z = q_rf[2]  
        rf_tf.transform.rotation.w = q_rf[3]
        # *****************************************

        q_lb = tf_transformations.quaternion_from_euler(0.0, 1.5708, 0.0)


        lb_tf = TransformStamped()
        lb_tf.header.stamp = self.get_clock().now().to_msg()
        lb_tf.header.frame_id = "base_link"
        lb_tf.child_frame_id = "dh_ref_lb"
        lb_tf.transform.translation.x = 0.159
        lb_tf.transform.translation.y = -0.138
        lb_tf.transform.translation.z = 0.359
        lb_tf.transform.rotation.x = q_lb[0]
        lb_tf.transform.rotation.y = q_lb[1]
        lb_tf.transform.rotation.z = q_lb[2]  
        lb_tf.transform.rotation.w = q_lb[3]
        # *****************************************
        q_lf = tf_transformations.quaternion_from_euler(0.0, 1.5708, 0.0)


        lf_tf = TransformStamped()
        lf_tf.header.stamp = self.get_clock().now().to_msg()
        lf_tf.header.frame_id = "base_link"
        lf_tf.child_frame_id = "dh_ref_lf"
        lf_tf.transform.translation.x = -0.0859
        lf_tf.transform.translation.y = -0.138
        lf_tf.transform.translation.z = 0.359
        lf_tf.transform.rotation.x = q_lf[0]
        lf_tf.transform.rotation.y = q_lf[1]
        lf_tf.transform.rotation.z = q_lf[2]  
        lf_tf.transform.rotation.w = q_lf[3]

        self.br_rf.sendTransform(rf_tf)
        self.br_rb.sendTransform(rb_tf)
        self.br_lb.sendTransform(lb_tf)
        self.br_lf.sendTransform(lf_tf)




def main(args=None):
    rclpy.init(args=args)
    static_tf = StaticTf()
    # rclpy.spin(static_tf)
    static_tf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

