#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import serial
import time
import serial       
import binascii
import rospy
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu
import codecs

ACCData = [0.0] * 8
GYROData = [0.0] * 8
AngleData = [0.0] * 8

FrameState = 0 
Bytenum = 0  
CheckSum = 0x0 

a = [0.0] * 3
w = [0.0] * 3
Angle = [0.0] * 3


def DueData(inputdata):  
    global FrameState 
    global Bytenum
    global CheckSum
    global a
    global w
    global Angle
    for data in inputdata:  
        #data = ord(data)
        #print('%#x'%ord(data))
        if FrameState == 0: 
            if ord(data) == 0x55 and Bytenum == 0: 
                CheckSum = ord(data)
                Bytenum = 1
                continue
            elif ord(data) == 0x51 and Bytenum == 1: 
                CheckSum += ord(data)
                FrameState = 1
                Bytenum = 2
            elif ord(data) == 0x52 and Bytenum == 1: 
                CheckSum += ord(data)
                FrameState = 2
                Bytenum = 2
            elif ord(data) == 0x53 and Bytenum == 1:
                CheckSum += ord(data)
                FrameState = 3
                Bytenum = 2
                #print("hello")
        elif FrameState == 1:  # acc    
            if Bytenum < 10:  
                ACCData[Bytenum - 2] = ord(data)  
                CheckSum += ord(data)
                Bytenum += 1
            else:
                if ord(data) == (CheckSum & 0xff):  # 
                    a = get_acc(ACCData)
                CheckSum = 0  #
                Bytenum = 0
                FrameState = 0
        elif FrameState == 2:  # gyro
            if Bytenum < 10:
                GYROData[Bytenum - 2] = ord(data)
                CheckSum += ord(data)
                Bytenum += 1
            else:
                if ord(data) == (CheckSum & 0xff):
                    w = get_gyro(GYROData)
                CheckSum = 0
                Bytenum = 0
                FrameState = 0
        elif FrameState == 3:  # angle
            if Bytenum < 10:
                AngleData[Bytenum - 2] = ord(data)
                CheckSum += ord(data)
                Bytenum += 1
            else:
                if ord(data) == (CheckSum & 0xff):
                    Angle = get_angle(AngleData)
                    #d = a + w + Angle
                    #print("Angle(deg):%10.3f %10.3f %10.3f" % Angle)
                    #print("hello")
                CheckSum = 0x0
                Bytenum = 0
                FrameState = 0
        
        


def get_acc(datahex):
    axl = datahex[0]
    axh = datahex[1]
    ayl = datahex[2]
    ayh = datahex[3]
    azl = datahex[4]
    azh = datahex[5]

    k_acc = 16.0
    
    acc_x = (axh * 256 + axl) / 32768.0 * k_acc
    acc_y = (ayh * 256 + ayl) / 32768.0 * k_acc
    acc_z = (azh * 256 + azl) / 32768.0 * k_acc
    if acc_x >= k_acc:
        acc_x -= 2 * k_acc
    if acc_y >= k_acc:
        acc_y -= 2 * k_acc
    if acc_z >= k_acc:
        acc_z -= 2 * k_acc

    return acc_x, acc_y, acc_z


def get_gyro(datahex):
    wxl = datahex[0]
    wxh = datahex[1]
    wyl = datahex[2]
    wyh = datahex[3]
    wzl = datahex[4]
    wzh = datahex[5]
    k_gyro = 2000.0

    gyro_x = (wxh *256 + wxl) / 32768.0 * k_gyro
    gyro_y = (wyh *256 + wyl) / 32768.0 * k_gyro
    gyro_z = (wzh *256 + wzl) / 32768.0 * k_gyro
    if gyro_x >= k_gyro:
        gyro_x -= 2 * k_gyro
    if gyro_y >= k_gyro:
        gyro_y -= 2 * k_gyro
    if gyro_z >= k_gyro:
        gyro_z -= 2 * k_gyro
    return gyro_x, gyro_y, gyro_z


def get_angle(datahex):
    rxl = datahex[0]
    rxh = datahex[1]
    ryl = datahex[2]
    ryh = datahex[3]
    rzl = datahex[4]
    rzh = datahex[5]
    k_angle = 180.0

    angle_x = (rxh *256+ rxl) / 32768.0 * k_angle
    angle_y = (ryh *256+ ryl) / 32768.0 * k_angle
    angle_z = (rzh *256+ rzl) / 32768.0 * k_angle
    if angle_x >= k_angle:
        angle_x -= 2 * k_angle
    if angle_y >= k_angle:
        angle_y -= 2 * k_angle
    if angle_z >= k_angle:
        angle_z -= 2 * k_angle

    return angle_x, angle_y, angle_z

def talker():
    rospy.init_node("imu")
    imuPub = rospy.Publisher("imu", Imu, queue_size=1)
    rate = rospy.Rate(100)

    port_name = rospy.get_param('~port','/dev/ttyUSB0')
    ser = serial.Serial(port_name, 115200, timeout=0.5)

    imuMsg = Imu()
    

    while not rospy.is_shutdown():
        datahex = ser.read(33)
        DueData(datahex)

        stamp = rospy.get_rostime()
        imuPub = rospy.Publisher("imu_data", Imu, queue_size=1)
        imuMsg.header.stamp, imuMsg.header.frame_id = stamp, "imu_link"
        #print(Angle[0])
        imuMsg.orientation.x = Angle[0]
        imuMsg.orientation.y = Angle[1]
        imuMsg.orientation.z = Angle[2] 
        imuMsg.angular_velocity.x, imuMsg.angular_velocity.y, imuMsg.angular_velocity.z = w
        imuMsg.linear_acceleration.x, imuMsg.linear_acceleration.y, imuMsg.linear_acceleration.z=a
        imuPub.publish(imuMsg)

        rate.sleep()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass