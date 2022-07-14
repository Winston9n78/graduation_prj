#!/usr/bin/env python3

from wldvl import WlDVL

import rospy
from std_msgs.msg import Float64MultiArray

import time

if __name__=="__main__":

    dvl = WlDVL("/dev/ttyUSB0")

    rospy.init_node('dvl_a50_node', anonymous=True)
    pub_dvl = rospy.Publisher('dvl_a50', Float64MultiArray, queue_size=1)


    now = time.time() 
    latest_time = time.time() 
    xx=0
    yy=0
    zz=0
    vel_x_old=0
    vel_y_old=0
    vel_z_old=0
    
    rate = rospy.Rate(100) # 10hz

    while not rospy.is_shutdown():

        report = dvl.read()
        
        now = time.time() 
        vel_x=report['vx']
        vel_y=report['vy']
        vel_z=report['vz']
        dt=now - latest_time
        xx = xx + dt * (vel_x+vel_x_old)/2
        yy = yy + dt * (vel_y+vel_y_old)/2
        zz = zz + dt * (vel_z+vel_z_old)/2

        #print(report)
        latest_time = now
        vel_x_old=vel_x
        vel_y_old=vel_y
        vel_z_old=vel_z

        dvl_data = [xx, yy, vel_x, vel_y]

        dvl_data = Float64MultiArray(data = dvl_data)

        pub_dvl.publish(dvl_data)

    # Show data
        print("dis ", xx,  yy,  zz)
        print("Velocity ", report['vx'], report['vx'], report['vz'])
        print("Valid measurement ", "Yes" if report['valid'] else "No")

        rate.sleep()
