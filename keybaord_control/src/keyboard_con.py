#!/usr/bin/env python3
import sys
import tty
import termios
import signal
import time
import rospy
from std_msgs.msg import *

global keyboard_start
 
def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch
 
def readkey(getchar_fn=None):
    getchar = getchar_fn or readchar
    c1 = getchar()
    if ord(c1) != 0x1b:
        return c1
    c2 = getchar()
    if ord(c2) != 0x5b:
        return c1
    c3 = getchar()
    return chr(0x10 + ord(c3) - 65)
 

def set_timeout(num, callback):
    def wrap(func):
        def handle(signum, frame):  # 收到信号 SIGALRM 后的回调函数，第一个参数是信号的数字，第二个参数是the interrupted stack frame.
            raise RuntimeError
 
        def to_do(*args, **kwargs):
            try:
                signal.signal(signal.SIGALRM, handle)  # 设置信号和回调函数
                signal.alarm(num)  # 设置 num 秒的闹钟
                # print('press k to stop this program...')
                r = func(*args, **kwargs)
                # print('Detected Key pressed.\n')
                signal.alarm(0)  # 关闭闹钟
                return r
            except RuntimeError as e:
                callback()
 
        return to_do
 
    return wrap
 
 
def after_timeout():  # 超时后的处理函数   

    global keyboard_start
    stop_usv = [0,0,keyboard_start]
    stop_usv = Int32MultiArray(data = stop_usv)
    pub.publish(stop_usv)
    print(keyboard_start)
    print("stop! press 'k' to exit\n")

@set_timeout(1, after_timeout)  # 限时 1 秒超时
def press_pause():
    key = readkey()
    return key

# 要做到转向的时候只转向
if __name__ == '__main__':

    for_back_ward = 0
    for_back_ward_init = 100

    turnnig = 0
    turnnig_init = 80

    push_F = 0

    keyboard_start = 0

    rospy.init_node('keyboard_control_node', anonymous=True)

    global pub

    pub = rospy.Publisher('keyboard_control', Int32MultiArray, queue_size=1)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        # 检测按键，停止训练，保存当前模型和最佳模型
        # key = press_pause()
            key = press_pause()
            # print(key)
            if key != None:
                if key == 'w': #前进
                    for_back_ward = for_back_ward_init + push_F
                    print(for_back_ward)

                elif key == 's': #后退
                    for_back_ward = -for_back_ward_init - push_F
                    print(for_back_ward)

                elif key == 'a': #左转
                    for_back_ward = 0
                    turnnig = turnnig_init + push_F
                    print(turnnig)

                elif key == 'd': #右转
                    for_back_ward = 0
                    turnnig = -turnnig_init - push_F
                    print(turnnig)
            ######进退档
                elif key == 'q': #进档
                    for_back_ward = 0
                    turnnig = 0
                    push_F = push_F + 50
                    print(push_F)

                elif key == 'e': #退档
                    for_back_ward = 0
                    turnnig = 0
                    push_F = push_F - 50
                    print(push_F)

                elif key == 'r': #遥控模式与自动模式选择
                    keyboard_start = ~keyboard_start
                    # keyboard_start = -keyboard_start
                    print(keyboard_start)
            
                else:
                    for_back_ward = 0
                    turnnig = 0
                    push_F = 0


                control_array = [for_back_ward, turnnig, keyboard_start]
                control_array = Int32MultiArray(data = control_array)

                pub.publish(control_array)

                turnnig = 0
                # for_back_ward = 0

            # if key != None:
                # print(key+'\n')
                if key == 'k':
                    print('exit')
                    break
            rate.sleep()
