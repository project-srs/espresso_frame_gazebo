#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import uuid

# ROS2
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.serialization import serialize_message
from ros2topic.api import get_msg_class
import rosbag2_py
from visualization_msgs.msg import MarkerArray, Marker
from s7_msgs.msg import LocalObjectList, LocalObject, LocalObjectList
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import Point, PointStamped, PoseWithCovarianceStamped, Pose2D

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, TwistStamped, Transform, TransformStamped, Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool

from tf2_ros import TransformBroadcaster
import math

class DummyMove(Node):
    def __init__(self):
        super().__init__('mavros_move')

        self.declare_parameter('initial_pose.x', 0.0)
        self.declare_parameter('initial_pose.y', 0.0)
        self.declare_parameter('initial_pose.theta', 0.0)

        # mavros(move)
        self.odometry_pub = self.create_publisher(Odometry, "mavros/velocity_position/odom", 1)
        self.armed_pub = self.create_publisher(Bool, "mavros/setup/armed", 1)
        self.mode_pub = self.create_publisher(String, "mavros/setup/mode", 1)

        self.cmd_vel_sub = self.create_subscription(TwistStamped, "mavros/setpoint_velocity/cmd_vel", self.cmd_vel_callback, 1)
        self.arming_srv = self.create_service(SetBool, "mavros/setup/request_arming", self.set_arming_callback)
        self.mode_srv = self.create_service(SetBool, "mavros/setup/request_guided", self.set_guided_callback)
        self.last_cmd_vel = TwistStamped()
        self.last_armed = False
        self.last_is_guided = False

        # mavros(other)
        self.battery_pub = self.create_publisher(BatteryState, "mavros/battery", 1)

        # map
        self.map_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/perception/slam_toolbox/pose", 1)

        # battery

        # other
        self.dt = 0.05
        self.interval_timer = self.create_timer(self.dt, self.timer_callback)
        self.last_twist = Twist()
        self.last_local_position = Pose2D()
        self.tf_broadcaster = TransformBroadcaster(self)

    def cmd_vel_callback(self, msg):
        self.last_cmd_vel = msg

    def set_arming_callback(self, request, response):
        self.get_logger().info(f'armed {self.last_armed} -> {request.data}')
        self.last_armed = request.data
        response.success = True
        return response

    def set_guided_callback(self, request, response):
        self.get_logger().info(f'armed {self.last_is_guided} -> {request.data}')
        self.last_is_guided = request.data
        response.success = True
        return response

    def timer_callback(self):
        ros_now = self.get_clock().now()

        # check arming state
        cmd_twist = Twist()
        if self.last_armed and self.last_is_guided:
            cmd_twist = self.last_cmd_vel.twist

        # update velocity
        rate = 2.0 * self.dt
        self.last_twist.linear.x += (cmd_twist.linear.x - self.last_twist.linear.x) * rate
        self.last_twist.angular.z += (cmd_twist.angular.z - self.last_twist.angular.z) * rate

        # update position
        th = self.last_local_position.theta + self.last_twist.angular.z * self.dt / 2.0
        self.last_local_position.x += math.cos(th) * self.last_twist.linear.x * self.dt
        self.last_local_position.y += math.sin(th) * self.last_twist.linear.x * self.dt
        self.last_local_position.theta += self.last_twist.angular.z * self.dt
        local_pose = Pose()
        local_pose.position.x = self.last_local_position.x
        local_pose.position.y = self.last_local_position.y
        local_pose.position.z = 0.072
        local_pose.orientation.z = math.sin(self.last_local_position.theta / 2.0)
        local_pose.orientation.w = math.cos(self.last_local_position.theta / 2.0)

        # output odom
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.header.stamp = ros_now.to_msg()
        odom.child_frame_id = "base_link"
        odom.twist.twist = self.last_twist
        odom.pose.pose = local_pose
        self.odometry_pub.publish(odom)

        # output odom_tf
        odom_tf = TransformStamped()
        odom_tf.header.frame_id = "odom"
        odom_tf.header.stamp = ros_now.to_msg()
        odom_tf.child_frame_id = "base_link"
        odom_tf.transform = pose_to_transform(local_pose)
        self.tf_broadcaster.sendTransform(odom_tf)

        # update map_pose
        init_x = self.get_parameter('initial_pose.x').get_parameter_value().double_value
        init_y = self.get_parameter('initial_pose.y').get_parameter_value().double_value
        init_theta = self.get_parameter('initial_pose.theta').get_parameter_value().double_value

        map_x = init_x + math.cos(init_theta) * self.last_local_position.x - math.sin(init_theta) * self.last_local_position.y
        map_y = init_y + math.sin(init_theta) * self.last_local_position.x + math.cos(init_theta) * self.last_local_position.y
        map_theta = init_theta + self.last_local_position.theta
        last_map_pose = Pose()
        last_map_pose.position.x = map_x
        last_map_pose.position.y = map_y
        last_map_pose.position.z = 0.0
        last_map_pose.orientation.z = math.sin(map_theta / 2.0)
        last_map_pose.orientation.w = math.cos(map_theta / 2.0)

        # output map_pose
        map_pose = PoseWithCovarianceStamped()
        map_pose.header.frame_id = "map"
        map_pose.header.stamp = ros_now.to_msg()
        map_pose.pose.pose = last_map_pose
        self.map_pose_pub.publish(map_pose)

        # output map_odom_tf
        map_odom_tf = TransformStamped()
        map_odom_tf.header.frame_id = "map"
        map_odom_tf.header.stamp = ros_now.to_msg()
        map_odom_tf.child_frame_id = "odom"
        map_odom_tf.transform.translation.x = init_x
        map_odom_tf.transform.translation.y = init_y
        map_odom_tf.transform.translation.z = 0.0
        map_odom_tf.transform.rotation.z = math.sin(init_theta / 2.0)
        map_odom_tf.transform.rotation.w = math.cos(init_theta / 2.0)
        self.tf_broadcaster.sendTransform(map_odom_tf)

        # output status
        armed_msg = Bool()
        armed_msg.data = self.last_armed
        self.armed_pub.publish(armed_msg)

        mode_msg = String()
        mode_msg.data = "GUIDED" if self.last_is_guided else "HOLD"
        self.mode_pub.publish(mode_msg)

        battery_msg = BatteryState()
        battery_msg.header.stamp = ros_now.to_msg()
        battery_msg.voltage = 19.0
        battery_msg.current = -0.9
        battery_msg.percentage = 0.97
        battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery_msg.present = True
        battery_msg.cell_voltage.append(19.0)
        self.battery_pub.publish(battery_msg)

def pose_to_transform(pose):
    transform = Transform()
    transform.translation.x = pose.position.x
    transform.translation.y = pose.position.y
    transform.translation.z = pose.position.z
    transform.rotation = pose.orientation
    return transform

def main(argv=sys.argv):
    try:
        rclpy.init()
        sbr = DummyMove()
        rclpy.spin(sbr)
        rclpy.shutdown()
    except Exception as e:
        print(e)
        pass
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main(sys.argv)
