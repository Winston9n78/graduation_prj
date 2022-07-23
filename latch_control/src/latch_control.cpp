#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "ros/ros.h"

bool is_ok_a, is_ok_b, done;

bool at9_control_1, at9_control_2, at9_control_3,reverse_flag;

void status_callback_a(const std_msgs::Bool msg){
    is_ok_a = msg.data;
}

void status_callback_b(const std_msgs::Bool msg){
    is_ok_b = msg.data;
}

void reverse_callback_b(const std_msgs::Bool msg){
    reverse_flag = msg.data;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "latch_control");

    ros::NodeHandle nh;

    ros::Publisher latch_signal, is_lock_ok;

    ros::Subscriber status_a_sub = nh.subscribe("is_ok_from_a", 1, &status_callback_a);
    ros::Subscriber status_b_sub = nh.subscribe("is_ok_from_b", 1, &status_callback_b);

    ros::Subscriber reverse_flag_sub = nh.subscribe("reverse_flag", 1, &reverse_callback_b);

    latch_signal = nh.advertise<std_msgs::Int32MultiArray>("latch_command", 1);
    is_lock_ok = nh.advertise<std_msgs::Bool>("is_lock_ok", 1); //将锁是否闭合完成发布出去
    std_msgs::Int32MultiArray control_cmd;
    std_msgs::Bool done_;
    
    // a锁与b锁
    int a = 0, b = 1, c = 2, d = 3;
    //设置默认状态打开钩子
    control_cmd.data.push_back(1);
    control_cmd.data.push_back(1);
    control_cmd.data.push_back(1);
    control_cmd.data.push_back(1);

    latch_signal.publish(control_cmd);
    // ros::Duration(5).sleep(); // 0.5ms脉冲

    double frequency = 10.0;
    double deltaTime = 1.0 / frequency;
    ros::Rate rate(frequency);

    static int count;
    while (nh.ok()) {
      if(reverse_flag){ //反方向才需要勾手动作，所以不翻转则直接不发，在被动船中，正反方向发的数据不一样，都要发

        if(is_ok_b && !done){ // 勾住 (is_ok_a||is_ok_b) && !done
            count++;

            control_cmd.data[a] = 1;/*勾住流程电平还不确定的，这里暂时示范*/

            if(count > 80){
              std::cout << "钩闭合" << std::endl;
              done = true; /*勾住动作完成*/
              count = 0;
            }

        }
        else if(!is_ok_b && done){ // 勾住失败而打开
            
            count++;
            control_cmd.data[a] = 0;/*勾住流程电平还不确定的，这里暂时示范*/

            if(count > 80){
              std::cout << "钩打开" << std::endl;
              done = false; /*勾住动作完成*/
              count = 0;
            }
        }

        done_.data = done;
        is_lock_ok.publish(done_);
        latch_signal.publish(control_cmd);
      }
        

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}