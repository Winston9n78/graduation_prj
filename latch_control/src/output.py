#!/usr/bin/env python3

from re import sub
from xmlrpc.client import boolean
# from turtle import delay
import serial
import time
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
import time

a = 0
b = 0
c = 0
d = 0

is_at9_open = 0

at9_control_1 = 0
at9_control_2 = 0
at9_control_3 = 0

reverse_flag = 0

def at9_callback_1(msg):
    global at9_control_1
    at9_control_1 = msg.data


def at9_callback_2(msg):
    global at9_control_2
    at9_control_2 = msg.data

def at9_callback_3(msg):
    global at9_control_3
    at9_control_3 = msg.data

def latch_callback(msg):
    global a,b,c,d
    a = msg.data[0]
    b = msg.data[1]
    c = msg.data[2]
    d = msg.data[3]

def at9_status_callback(msg):
    global is_at9_open
    is_at9_open = msg.data

def reverse_callback(msg):
    global reverse_flag
    reverse_flag = msg.data

def talker():
    global reverse_flag,is_at9_open,a,b,c,d,at9_control_1,at9_control_2,at9_control_3

    rospy.init_node('latch_output_node', anonymous=True)

    sub_latch_cmd = rospy.Subscriber('/latch_command', Int32MultiArray, latch_callback)

    sub_at9_status = rospy.Subscriber('/is_at9_open', Bool, at9_status_callback)

    sub_at9_status = rospy.Subscriber('/at9_control_1', Bool, at9_callback_1)
    sub_at9_status = rospy.Subscriber('/at9_control_2', Bool, at9_callback_2)
    sub_at9_status = rospy.Subscriber('/at9_control_3', Int32, at9_callback_3)

    sub_reverse_status = rospy.Subscriber('/reverse_flag', Bool, reverse_callback)

    pub_reverse_flag = rospy.Publisher('reverse_flag', Bool, queue_size=1)

    reverse_flag = False

    serial_sensor = serial.Serial("/dev/ttyUSB1", timeout=5)

    send_signal_1_out = '01 0F 00 00 00 04 01 05 FE 95'
    send_signal_1_out = bytes.fromhex(send_signal_1_out)

    send_signal_1_back = '01 0F 00 00 00 04 01 0A BE 91'
    send_signal_1_back = bytes.fromhex(send_signal_1_back)

    # send_signal_2_out = '01 02 00 00 00 04 79 C9'
    # send_signal_2_out = bytes.fromhex(send_signal_2_out)

    # send_signal_2_back = '01 02 00 00 00 04 79 C9'
    # send_signal_2_back = bytes.fromhex(send_signal_2_back)

    # send_signal_4 = '01 02 00 00 00 04 79 C9'
    # send_signal_4 = bytes.fromhex(send_signal_4)

    # send_signal_5 = '01 02 00 00 00 04 79 C9'
    # send_signal_5 = bytes.fromhex(send_signal_5)

    # send_signal_6 = '01 02 00 00 00 04 79 C9'
    # send_signal_6 = bytes.fromhex(send_signal_6)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():

        # print(is_at9_open)
        if is_at9_open == True:
            # print(at9_control_1,at9_control_2,at9_control_3)
            if at9_control_1:
                serial_sensor.write(send_signal_1_out)
            else:
                serial_sensor.write(send_signal_1_back)
            
            # 被动船加
            # if at9_control_2:
            #     serial_sensor.write(send_signal_2_out)
            # else:
            #     serial_sensor.write(send_signal_2_back)

            # 主动船和被动船都加，都只是遥控
            # if at9_control_3 == 1:
            #     serial_sensor.write(send_signal_2) #支腿上升
            # elif at9_control_3 == 2:
            #     serial_sensor.write(send_signal_2) #支腿下降
            # else:
            #     serial_sensor.write(send_signal_2) #支腿不动作
            
        elif  (not is_at9_open) and reverse_flag: #翻转的时候才需要控制，这里是主动船,只有一个钩子
            if a: #开关钩子1
                serial_sensor.write(send_signal_1_out)
            else:
                serial_sensor.write(send_signal_1_back)

            # 被动船加，主动船只有一个
            # if b: #开关钩子2
            #     serial_sensor.write(send_signal_2_out)
            # else:
            #     serial_sensor.write(send_signal_4)

        # 读取光电传感器数据并且publish，被动船无需此功能
        send_signal = '01 02 00 00 00 04 79 C9'
        send_signal = bytes.fromhex(send_signal)
        serial_sensor.write(send_signal)        
        time.sleep(0.1)
        count= serial_sensor.inWaiting()
        if count > 0:
            recv_data = serial_sensor.read(count)
            # print(recv_data)

        if recv_data == b'\x01\x02\x01\x01`H' :
            reverse_flag = True
        elif recv_data == b'\x01\x02\x01\x02 I':
            reverse_flag = False
        else:
            print("camera in wrong position!!")

        reverse_flag = Bool(data = reverse_flag)
        pub_reverse_flag.publish(reverse_flag)

        rate.sleep()


    serial_sensor.close()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
