#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将订阅/turtle1/pose话题，消息类型turtlesim::Pose

import rospy
# from turtlesim.msg import Pose
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt
from nav_msgs.msg import Path

x_list=[]
y_list=[]
z_list=[]
v_x_list=[]
v_y_list=[]

def poseCallback(msg):
    # rospy.loginfo("Turtle pose: x:%0.6f, y:%0.6f", msg.x, msg.y)
    # rospy.loginfo(msg.points[0].x)
    print("check")
    for i in msg.poses:
        # print(i.pose.position.x)
        v_x_list.append(i.pose.position.x)
        v_y_list.append(i.pose.position.y)
    # x_list.append(msg.points[0].x)
    # y_list.append(msg.points[0].y)
    fig = plt.figure()
    x_plot=fig.add_subplot(2,1,1)
    y_plot=fig.add_subplot(2,1,2)
    x_plot.plot(v_x_list)
    y_plot.plot(v_y_list)
    plt.show()
    x_list.clear()
    y_list.clear()

def showCallback(msg):
    fig = plt.figure()
    x_plot=fig.add_subplot(2,1,1)
    y_plot=fig.add_subplot(2,1,2)
    x_plot.plot(x_list)
    y_plot.plot(y_list)
    plt.show()
    x_list.clear()
    y_list.clear()


def pose_subscriber():
	# ROS节点初始化
    print("check2")
    rospy.init_node('pose_subscriber', anonymous=True)

	# 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
    rospy.Subscriber("/trajectory_generator_node/vel", Path, poseCallback)
    # rospy.Subscriber("/waypoint_generator/waypoints",Path,showCallback)

	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    pose_subscriber()


