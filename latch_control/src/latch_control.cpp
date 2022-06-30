#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "ros/ros.h"

bool is_ok, done;

void status_callback(const std_msgs::Bool msg){
    is_ok = msg.data;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "latch_control");

    ros::NodeHandle nh;

    ros::Publisher latch_signal, is_lock_ok;

    ros::Subscriber status_sub = nh.subscribe("is_ok", 1, &status_callback);
    latch_signal = nh.advertise<std_msgs::Int32MultiArray>("latch_command", 1);
    is_lock_ok = nh.advertise<std_msgs::Bool>("is_lock_ok", 1);
    std_msgs::Int32MultiArray control_cmd;
    std_msgs::Bool done_;
    
    // a锁与b锁
    int a_up = 0, a_down = 1, b_up = 2, b_down = 3;
    //设置默认状态打开钩子
    control_cmd.data.push_back(0);
    control_cmd.data.push_back(0);
    control_cmd.data.push_back(0);
    control_cmd.data.push_back(0);

    latch_signal.publish(control_cmd);


    double frequency = 100.0;
    double deltaTime = 1.0 / frequency;
    ros::Rate rate(frequency);
    while (nh.ok()) {
        if(is_ok && !done){ // 勾住
            control_cmd.data[a_up] = 1;
            latch_signal.publish(control_cmd);
            //延时操作,延时前的状态也需要发布
            ros::Duration(0.5).sleep();
            control_cmd.data[a_down] = 1;
            latch_signal.publish(control_cmd);
            ros::Duration(0.5).sleep();
            done = true;
        }
        else if(!is_ok && done){ // 勾住失败而打开
            control_cmd.data[a_up] = 0;
            control_cmd.data[a_down] = 0;
            done = false;
            latch_signal.publish(control_cmd);
        }
        done_.data = done;
        is_lock_ok.publish(done_);
        latch_signal.publish(control_cmd);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}