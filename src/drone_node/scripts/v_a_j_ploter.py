#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将订阅/turtle1/pose话题，消息类型turtlesim::Pose

import rospy
# from turtlesim.msg import Pose
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
import time
import numpy as np
import math

x_list=[]
y_list=[]
z_list=[]
v_x_list=[]
v_y_list=[]

a_x_list=[]
a_y_list=[]

j_x_list=[]
j_y_list=[]


v_mod_list=[]
a_mod_list=[]
j_mod_list=[]

raw_vx_list=[]
raw_ax_list=[]
raw_jx_list=[]
raw_vy_list=[]
raw_ay_list=[]
raw_jy_list=[]
raw_vz_list=[]
raw_az_list=[]
raw_jz_list=[]



def velCallback(msg):
    global raw_vx_list,raw_vy_list,raw_vz_list
    for i in msg.poses:
        raw_vx_list.append(i.pose.position.x)
        raw_vy_list.append(i.pose.position.y)
        raw_vz_list.append(i.pose.position.z)
    plot()

def accCallback(msg):
    global raw_ax_list,raw_ay_list,raw_az_list
    for i in msg.poses:
        raw_ax_list.append(i.pose.position.x)
        raw_ay_list.append(i.pose.position.y)
        raw_az_list.append(i.pose.position.z)
    plot()

def jerkCallback(msg):
    global raw_jx_list,raw_jy_list,raw_jz_list
    for i in msg.poses:
        raw_jx_list.append(i.pose.position.x)
        raw_jy_list.append(i.pose.position.y)
        raw_jz_list.append(i.pose.position.z)
    plot()


def plot():
    global raw_vx_list,raw_vy_list,raw_vz_list,raw_ax_list,raw_ay_list,raw_az_list,raw_jx_list,raw_jy_list,raw_jz_list
    # print("length = ",len(raw_vx_list), len(raw_j_list),len(raw_a_list))
    if  len(raw_vx_list)>0 and len(raw_jx_list)>0 and len(raw_ax_list)>0 :
        
        t=np.arange(0,len(raw_vx_list),1)
        t=t/100.0

        plt.title("vel,acc and jerk in besier")

        plt.subplot(3,3,1)
        plt.plot(t,raw_vx_list)
        plt.xlabel("time")
        plt.ylabel("vel x / m/s")

        plt.subplot(3,3,4)
        plt.plot(t,raw_ax_list)
        plt.xlabel("time")
        plt.ylabel("acc x / m/s2")

        plt.subplot(3,3,7)
        plt.plot(t,raw_jx_list)
        plt.xlabel("time")
        plt.ylabel("jerk x / m/s3")#plot x

        plt.subplot(3,3,2)
        plt.plot(t,raw_vy_list)
        plt.xlabel("time")
        plt.ylabel("vel y / m/s")

        plt.subplot(3,3,5)
        plt.plot(t,raw_ay_list)
        plt.xlabel("time")
        plt.ylabel("acc y / m/s2")

        plt.subplot(3,3,8)
        plt.plot(t,raw_jy_list)
        plt.xlabel("time")
        plt.ylabel("jerk y / m/s3")#plot y
        
        plt.subplot(3,3,3)
        plt.plot(t,raw_vz_list)
        plt.xlabel("time")
        plt.ylabel("vel z / m/s")

        plt.subplot(3,3,6)
        plt.plot(t,raw_az_list)
        plt.xlabel("time")
        plt.ylabel("acc z / m/s2")

        plt.subplot(3,3,9)
        plt.plot(t,raw_jz_list)
        plt.xlabel("time")
        plt.ylabel("jerk z / m/s3")#plot z

        plt.show()



def get_mod(x1,x2,x3):
    return math.sqrt(x1**2+x2**2+x3**2)

def get_mod(msg):
    return math.sqrt(msg.x**2+msg.y**2+msg.z**2)

target_x=0
target_y=0

cur_x=0
cur_y=0

plot_flag=0

start_time=time.time()
cur_time=time.time()
time_init_flag=1


def poseCallback(msg):
    global cur_x,cur_y
    cur_x=msg.points[0].x
    cur_y=msg.points[0].y
    # print("pose update")
    


def targetCallback(msg):
    global target_x,target_y
    target_x=msg.poses[0].pose.position.x
    target_y=msg.poses[0].pose.position.y
    print("target update  ",target_x,"  ",target_y)



def vel_acc_Callback(msg):
    global x_list,y_list,a_x_list,a_y_list,j_x_list,j_y_list,plot_flag
    global v_mod_list,a_mod_list,j_mod_list
    # rospy.loginfo("Turtle pose: x:%0.6f, y:%0.6f", msg.x, msg.y)
    # rospy.loginfo(msg.points[0].x)
    # print("check")
    plt.close()
    # for i in msg.poses:
    #     # print(i.pose.position.x)
    #     v_x_list.append(i.pose.position.x)
    #     v_y_list.append(i.pose.position.y)
    #     print(i.pose.position.x)

    x_list.append(msg.poses[0].pose.position.x)
    y_list.append(msg.poses[0].pose.position.y)
    a_x_list.append(msg.poses[1].pose.position.x)
    a_y_list.append(msg.poses[1].pose.position.y)
    j_x_list.append(msg.poses[2].pose.position.x)
    j_y_list.append(msg.poses[2].pose.position.y)

    v_mod_list.append(get_mod(msg.poses[0].pose.position))
    a_mod_list.append(get_mod(msg.poses[1].pose.position))
    j_mod_list.append(get_mod(msg.poses[2].pose.position))


    threshold=0.1

    if abs(cur_y-target_y)>threshold:
        plot_flag=1
    # print(msg.poses[0].pose.position.x)



    print("cur_x = %f y = %f target_x=%f y=%f"%(cur_x,target_x,cur_y,target_y))
    if abs(cur_x-target_x)<threshold and abs(cur_y-target_y)<threshold and not target_x==0 and plot_flag:
        # fig = plt.figure()
        # x_plot=fig.add_subplot(2,1,1)
        # y_plot=fig.add_subplot(2,1,2)
        # x_plot.plot(x_list)
        # # x_plot.xlabel("time")
        # plt.xlabel("time")
        # y_plot.plot(y_list)
        # plt.xlabel("time2")
        # # y_plot.xlabel("time")
        # plt.show()
        # plot_flag=0
        # x_list=[]
        # y_list=[]

        t=np.arange(0,len(x_list),1)
        t=t/100.0

        plt.subplot(3,2,1)
        plt.plot(t,x_list)
        plt.xlabel("time")
        plt.ylabel("v_x / m/s")


        plt.subplot(3,2,2)
        plt.plot(t,y_list)
        plt.xlabel("time")
        plt.ylabel("v_y / m/s")

        plt.subplot(3,2,3)
        plt.plot(t,a_x_list)
        plt.xlabel("time")
        plt.ylabel("a_x / m/s2")


        plt.subplot(3,2,4)
        plt.plot(t,a_y_list)
        plt.xlabel("time")
        plt.ylabel("a_y / m/s2")

        plt.subplot(3,2,5)
        plt.plot(t,j_x_list)
        plt.xlabel("time")
        plt.ylabel("j_x / m/s2")

        plt.subplot(3,2,6)
        plt.plot(t,j_y_list)
        plt.xlabel("time")
        plt.ylabel("j_y / m/s2")

        # plt.subplot(3,1,1)
        # plt.plot(t,v_mod_list)
        # plt.xlabel("time")
        # plt.ylabel("vel / m/s")

        # plt.subplot(3,1,2)
        # plt.plot(t,a_mod_list)
        # plt.xlabel("time")
        # plt.ylabel("acc / m/s2")

        # plt.subplot(3,1,3)
        # plt.plot(t,j_mod_list)
        # plt.xlabel("time")
        # plt.ylabel("jerk / m/s3")


        plt.show()
        plot_flag=0
        x_list=[]
        y_list=[]
        a_x_list=[]
        a_y_list=[]
        j_x_list=[]
        j_y_list=[]
        v_mod_list=[]
        a_mod_list=[]
        j_mod_list=[]


    # plt.pause(0.1)
    
    # x_list.clear()
    # y_list.clear()
    # x=[1,2,3]
    # y=[4,7,5]

    # plt.plot(x,y)
    # plt.show()

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
    rospy.Subscriber("/trajectory_generator_node/vel", Path, velCallback)
    rospy.Subscriber("/trajectory_generator_node/acc", Path, accCallback)
    rospy.Subscriber("/trajectory_generator_node/jerk", Path, jerkCallback)
    # rospy.Subscriber("/drone_node/front_drone_v_a", Path, vel_acc_Callback)
    # rospy.Subscriber("/waypoint_generator/waypoints", Path, targetCallback)
    # rospy.Subscriber("/drone_node/drone_pos", Marker, poseCallback)
    
    # rospy.Subscriber("/waypoint_generator/waypoints",Path,showCallback)

	# 循环等待回调函数
    rospy.spin()


if __name__ == '__main__':
    pose_subscriber()


