#!/usr/bin/env python3

import serial
import time
import rospy
from std_msgs.msg import Bool

def talker():

    rospy.init_node('photoelectric_node', anonymous=True)

    serial_sensor = serial.Serial("/dev/ttyUSB0", timeout=5)

    send_signal = '01 02 00 00 00 04 79 C9'
    send_signal = bytes.fromhex(send_signal)

    pub_reverse_flag = rospy.Publisher('reverse_flag', Bool, queue_size=1)

    reverse_flag = False

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        serial_sensor.write(send_signal)
        time.sleep(0.1)
        count= serial_sensor.inWaiting()
        if count > 0:
            recv_data = serial_sensor.read(count)
            print(recv_data)
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
