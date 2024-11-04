#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool

class SampleApp(Node):

   def __init__(self):
      super().__init__('sample_app')
      self.joy_sub = self.create_subscription(Joy, '/device/mavros/rc_joy/joy', self.joy_callback, 10)# subscriberの宣言
      self.last_joy = None

      self.move_twist_pub = self.create_publisher(TwistStamped,'/device/mavros/setpoint_velocity/cmd_vel', 1)
      self.last_velocity_x = 0.0
      self.turret_twist_pub = self.create_publisher(TwistStamped,'/device/head_turret/cmd_rate', 1)

      self.mode_client = self.create_client(SetBool, '/device/mavros/setup/request_guided')
      self.mode_future = None
      self.arm_client = self.create_client(SetBool, '/device/mavros/setup/request_arming')
      self.arm_future = None

      self.timer = self.create_timer(0.2, self.timer_callback)
      self.request_setup_status = False

   def joy_callback(self, msg):
      last_is_off = self.last_joy and 1 <= len(self.last_joy.buttons) and self.last_joy.buttons[0] == 0
      current_is_on = 1 <= len(msg.buttons) and msg.buttons[0] != 0
      if last_is_off and current_is_on:
         self.get_logger().info('request arm')
         if self.mode_client.wait_for_service(timeout_sec=0.5):
            req = SetBool.Request()
            req.data = True
            self.mode_future = self.mode_client.call_async(req)

      if 6 <= len(msg.axes):
         target_velocity_x = 0.5 * msg.axes[4]
         rate = 0.1
         velocity_x = rate * target_velocity_x + (1 - rate) * self.last_velocity_x
         self.last_velocity_x = velocity_x

         twist = TwistStamped()
         twist.header.frame_id = 'base_link'
         twist.header.stamp = msg.header.stamp
         twist.twist.linear.x = velocity_x
         twist.twist.angular.z = 1.5 * msg.axes[3]
         self.move_twist_pub.publish(twist)

      if 4 <= len(msg.axes):
         twist = TwistStamped()
         twist.header.frame_id = 'base_turret_link'
         twist.header.stamp = msg.header.stamp
         twist.twist.angular.y = -1.0 * msg.axes[1]
         twist.twist.angular.z = 1.0 * msg.axes[0]
         self.turret_twist_pub.publish(twist)

      self.last_joy = msg

   def timer_callback(self):
      if self.request_setup_status:
         self.request_setup_status = False

      if self.mode_future and self.mode_future.done():
         response = self.mode_future.result()
         self.mode_future = None

         if self.arm_client.wait_for_service(timeout_sec=0.5):
            req = SetBool.Request()
            req.data = True
            self.arm_future = self.arm_client.call_async(req)

      if self.arm_future and self.arm_future.done():
         self.arm_future = None
         self.get_logger().info('arm done')


def main():
   rclpy.init()
   node = SampleApp()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()