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
from geometry_msgs.msg import Point, PointStamped, PoseWithCovarianceStamped, Pose2D, Quaternion

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, TwistStamped, Transform, TransformStamped, Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool

from tf2_ros import TransformBroadcaster
import math

class DummyTurret(Node):
    def __init__(self):
        super().__init__('mavros_move')

        # turret
        self.odometry_pub = self.create_publisher(Odometry, "/device/head_turret/odom", 1)
        self.cmd_vel_sub = self.create_subscription(TwistStamped, "/device/head_turret/cmd_rate", self.cmd_rate_callback, 1)
        self.last_cmd_rate = TwistStamped()

        # other
        self.dt = 1.0 / 30
        self.interval_timer = self.create_timer(self.dt, self.timer_callback)
        self.last_twist = Twist()
        self.last_pitch = 0.0
        self.last_yaw = 0.0
        self.tf_broadcaster = TransformBroadcaster(self)

    def cmd_rate_callback(self, msg):
        self.last_cmd_rate = msg

    def timer_callback(self):
        ros_now = self.get_clock().now()

        # update velocity
        rate = 2.0 * self.dt
        self.last_twist.angular.y += (self.last_cmd_rate.twist.angular.y - self.last_twist.angular.y) * rate
        self.last_twist.angular.z += (self.last_cmd_rate.twist.angular.z - self.last_twist.angular.z) * rate

        # update position
        self.last_pitch = min(max(self.last_pitch + self.last_twist.angular.y * self.dt, -math.pi / 4), math.pi / 2)
        self.last_yaw = min(max(self.last_yaw + self.last_twist.angular.z * self.dt, -math.pi / 2), math.pi / 2)

        # output odom
        odom = Odometry()
        odom.header.frame_id = "turret_base_link"
        odom.header.stamp = ros_now.to_msg()
        odom.child_frame_id = "turret_tip_link"
        odom.twist.twist = self.last_twist
        odom.pose.pose.orientation = generate_quaternion_from_euler(0, self.last_pitch, self.last_yaw)
        self.odometry_pub.publish(odom)

        # output odom_tf
        odom_tf = TransformStamped()
        odom_tf.header.frame_id = "turret_base_link"
        odom_tf.header.stamp = ros_now.to_msg()
        odom_tf.child_frame_id = "turret_tip_link"
        odom_tf.transform = pose_to_transform(odom.pose.pose)
        self.tf_broadcaster.sendTransform(odom_tf)

def pose_to_transform(pose):
    transform = Transform()
    transform.translation.x = pose.position.x
    transform.translation.y = pose.position.y
    transform.translation.z = pose.position.z
    transform.rotation = pose.orientation
    return transform

def generate_quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def main(argv=sys.argv):
    try:
        rclpy.init()
        sbr = DummyTurret()
        rclpy.spin(sbr)
        rclpy.shutdown()
    except Exception as e:
        print(e)
        pass
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main(sys.argv)
