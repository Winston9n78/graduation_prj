#!/usr/bin/env python
#-*- coding: UTF-8 -*- 
from sys import path
import rospy
from geometry_msgs.msg import *
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32
from nav_msgs.msg import Path

# import numpy as np

def talker():

    rospy.init_node('parameters_node', anonymous=True)
    # pub_goal = rospy.Publisher('goal_point', PoseStamped, queue_size=1)
    pub_start = rospy.Publisher('start_point', PoseStamped, queue_size=1)

    pub_v = rospy.Publisher('velocity_pid_parameter', Float64MultiArray, queue_size=1)
    pub_t = rospy.Publisher('turning_pid_parameter', Float64MultiArray, queue_size=1)

    pub_c = rospy.Publisher('connect_pid_parameter', Float64MultiArray, queue_size=1)
    pub_s = rospy.Publisher('stick_pid_parameter', Float64MultiArray, queue_size=1)

    pub_set_v = rospy.Publisher('velocity_set', Float32, queue_size=1)

    pub_path = rospy.Publisher('path_set', Float64MultiArray, queue_size=100)

    point = PoseStamped()

    path_path = Path()

####################################################
# adjust the parameter here 
###################################################   

    path = [0,5.8,
            0,23.0,
            8.4,19,
            5.8,9.2,
            0,5.8] #手动输入路径坐标点

    velocity_set = 2 #设定速度

    velocity_pid_parameter = [-150,0,0] #-120
    turning_pid_parameter = [-3,0,0] # -3 符号正确性

    connected_parameter = [400,0,0]
    stick_parameter = [200,0,0]


#####参数显示
    print("路径坐标：")
    print(path)
    print("\n")
    
#####################################################
    velocity_pid_parameter = Float64MultiArray(data = velocity_pid_parameter)
    turning_pid_parameter = Float64MultiArray(data = turning_pid_parameter)
    connected_parameter = Float64MultiArray(data = connected_parameter)
    stick_parameter = Float64MultiArray(data = stick_parameter)
    path_parameter = Float64MultiArray(data = path)



    # for i in range(0, 5):
    #     point.pose.position.x = path[i][0]
    #     point.pose.position.y = path[i][1]
    #     path_path.poses.append(point)
        # point.pose.position.x = path[1][0]
        # point.pose.position.y = path[1][1]
        # path_path.poses.append(point)
        # path_path.poses.append(point)
        # path_path.poses.insert(i, point)


    # path_path.poses.pop()
    # print(path_path.poses.__format__())

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        pub_v.publish(velocity_pid_parameter)
        pub_t.publish(turning_pid_parameter)

        pub_c.publish(connected_parameter)
        pub_s.publish(stick_parameter)

        pub_set_v.publish(velocity_set)
        # pub_goal.publish(point0)
        # pub_start.publish(point1)   
        pub_path.publish(path_parameter)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

# #目标点坐标
#     point0.pose.position.x = 6
#     point0.pose.position.y = 7
# #起始点坐标
#     point1.pose.position.x = 6
#     point1.pose.position.y = 20