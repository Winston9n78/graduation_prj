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

    ros::Publisher latch_signal;

    ros::Subscriber status_sub = nh.subscribe("is_ok", 10, &status_callback);
    latch_signal = nh.advertise<std_msgs::Int32MultiArray>("latch_command", 1);
    std_msgs::Int32MultiArray control_cmd;
    
    // a锁与b锁
    int up_a = 0, down_a = 1, up_b = 2, down_b = 3;
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

            control_cmd.data[up_a] = 1;
            latch_signal.publish(control_cmd);
            //延时操作,延时前的状态也需要发布
            ros::Duration(0.5).sleep();
            control_cmd.data[down_a] = 1;
            latch_signal.publish(control_cmd);
            done = true;
        }
        else if(!is_ok && done){ // 勾住失败而打开
            control_cmd.data[up_a] = 0;
            control_cmd.data[down_a] = 0;
            latch_signal.publish(control_cmd);
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}