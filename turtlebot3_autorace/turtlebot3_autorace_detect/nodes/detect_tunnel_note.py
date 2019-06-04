#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
import tf
from enum import Enum
from std_msgs.msg import UInt8, Float64, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionResult


class DetectTunnel():
    def __init__(self):
        self.sub_tunnel_order = rospy.Subscriber('/detect/tunnel_order', UInt8, self.cbTunnelOrder, queue_size=1)
        self.sub_arrival_status = rospy.Subscriber("/move_base/result", MoveBaseActionResult,
                                                   self.cbGetNavigationResult, queue_size=1)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom, queue_size=1)

        self.pub_tunnel_return = rospy.Publisher('/detect/tunnel_stamped', UInt8, queue_size=1)
        self.pub_goal_pose_stamped = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.pub_init_pose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('/control/cmd_vel', Twist, queue_size=1)
        self.pub_max_vel = rospy.Publisher('/control/max_vel', Float64, queue_size=1)

        self.StepOfTunnel = Enum('StepOfTunnel',
                                 'searching_tunnel_sign '
                                 'go_in_to_tunnel '
                                 'navigation '
                                 'go_out_from_tunnel '
                                 'exit')

        self.is_navigation_finished = False
        self.is_tunnel_finished = False

        self.last_current_theta = 0.0

    def cbGetNavigationResult(self, msg_nav_result):
        if msg_nav_result.status.status == 3:
            rospy.loginfo("Reached")
            self.is_navigation_finished = True

    def cbTunnelOrder(self, order):
        pub_tunnel_return = UInt8()

        if order.data == self.StepOfTunnel.searching_tunnel_sign.value:
            rospy.loginfo("Now lane_following")

            pub_tunnel_return.data = self.StepOfTunnel.searching_tunnel_sign.value


        elif order.data == self.StepOfTunnel.go_in_to_tunnel.value:
            rospy.loginfo("Now go_in_to_tunnel")

            self.lastError = 0.0
            self.start_pos_x = self.current_pos_x
            self.start_pos_y = self.current_pos_y

            while True:
                error = self.fnStraight(0.55)

                if math.fabs(error) < 0.005:
                    break

            self.fnStop()

            rospy.loginfo("go_in_to_tunnel finished")

            pub_tunnel_return.data = self.StepOfTunnel.go_in_to_tunnel.value


        elif order.data == self.StepOfTunnel.navigation.value:
            rospy.loginfo("Now navigation")
            initialPose = PoseWithCovarianceStamped()
            initialPose.header.frame_id = "map"
            initialPose.header.stamp = rospy.Time.now()
            initialPose.pose.pose = self.odom_msg.pose.pose

            self.pub_init_pose.publish(initialPose)

            self.fnPubGoalPose()

            while True:
                if self.is_navigation_finished == True:
                    break
                else:
                    pass

            pub_tunnel_return.data = self.StepOfTunnel.navigation.value


        elif order.data == self.StepOfTunnel.go_out_from_tunnel.value:
            rospy.loginfo("Now go_out_from_tunnel")

            self.lastError = 0.0
            self.start_pos_x = self.current_pos_x
            self.start_pos_y = self.current_pos_y

            while True:
                error = self.fnStraight(0.25)

                if math.fabs(error) < 0.005:
                    break

            self.fnStop()

            pub_tunnel_return.data = self.StepOfTunnel.go_out_from_tunnel.value

        elif order.data == self.StepOfTunnel.exit.value:
            rospy.loginfo("Now exit")

            pub_tunnel_return.data = self.StepOfTunnel.exit.value

        self.pub_tunnel_return.publish(pub_tunnel_return)

    def cbOdom(self, odom_msg):
        #将里程计数据转换成位置的四元数坐标，并将四元数转化成殴拉角作为当前角度
        #根据当前角度与上一时刻的角度差值的大小设定当前时刻和上一时刻的角度值
        quaternion = (odom_msg.pose.pose.orientation.x,
                      odom_msg.pose.pose.orientation.y,
                      odom_msg.pose.pose.orientation.z,
                      odom_msg.pose.pose.orientation.w)
        self.current_theta = self.euler_from_quaternion(quaternion)
        self.odom_msg = odom_msg
        if (self.current_theta - self.last_current_theta) < -math.pi:
            self.current_theta = 2. * math.pi + self.current_theta
            self.last_current_theta = math.pi
        elif (self.current_theta - self.last_current_theta) > math.pi:
            self.current_theta = -2. * math.pi + self.current_theta
            self.last_current_theta = -math.pi
        else:
            self.last_current_theta = self.current_theta

        #将采集到的里程计数据（位置坐标）作为当前位置坐标
        self.current_pos_x = odom_msg.pose.pose.position.x
        self.current_pos_y = odom_msg.pose.pose.position.y

    def euler_from_quaternion(self, quaternion):
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        return theta

    def fnPubGoalPose(self):
        #利用导航功能，监听move_base/goal节点信息，得到导航终点的坐标信息
        goalPoseStamped = PoseStamped()

        goalPoseStamped.header.frame_id = "map"
        goalPoseStamped.header.stamp = rospy.Time.now()

        goalPoseStamped.pose.position.x = 0.15
        goalPoseStamped.pose.position.y = -1.76
        goalPoseStamped.pose.position.z = 0.0

        goalPoseStamped.pose.orientation.x = 0.0
        goalPoseStamped.pose.orientation.y = 0.0
        goalPoseStamped.pose.orientation.z = 0.0
        goalPoseStamped.pose.orientation.w = 1.0

        self.pub_goal_pose_stamped.publish(goalPoseStamped)

    def fnStraight(self, desired_dist):
        #计算当前位置与起始位置的位置偏差，建立比例微分控制器，发布速度控制命令
        err_pos = math.sqrt(
            (self.current_pos_x - self.start_pos_x) ** 2 + (self.current_pos_y - self.start_pos_y) ** 2) - desired_dist

        rospy.loginfo("Tunnel_Straight")
        rospy.loginfo("err_pos  desired_dist : %f  %f  %f", err_pos, desired_dist, self.lastError)

        Kp = 0.4
        Kd = 0.05

        angular_z = Kp * err_pos + Kd * (err_pos - self.lastError)
        self.lastError = err_pos

        twist = Twist()
        twist.linear.x = 0.07
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

        return err_pos

    def fnStop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

    def cbTunnelFinished(self, tunnel_finished_msg):
        self.is_tunnel_finished = True

    def main(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('detect_tunnel')
    node = DetectTunnel()
    node.main()
