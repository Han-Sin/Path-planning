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

def poseCallback(msg):
    # rospy.loginfo("Turtle pose: x:%0.6f, y:%0.6f", msg.x, msg.y)
    # rospy.loginfo(msg.points[0].x)
    x_list.append(msg.points[0].x)
    y_list.append(msg.points[0].y)
    # plt.plot(x_list)
    # plt.pause(0.01)

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
    rospy.init_node('pose_subscriber', anonymous=True)

	# 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
    rospy.Subscriber("/drone_node/drone2_pos", Marker, poseCallback)
    rospy.Subscriber("/waypoint_generator/waypoints",Path,showCallback)

	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    pose_subscriber()


