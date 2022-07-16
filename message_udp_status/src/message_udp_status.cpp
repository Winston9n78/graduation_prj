#include "message_udp_status/message_udp_status.h"



float usv_x = 0.1, usv_y = 0.12345, usv_orien = 0.123456;

int is_ok = 0;

int is_ok_from_a = 0; // 将接收到的另一艘船的解码的变量

void usv_status_callback(const otter_control::usv_status msg){
    usv_x = msg.position_z;
    usv_y = msg.position_x;
    usv_orien = msg.orientation_pitch;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "messsage_udp_latch_node");

  ros::NodeHandle nh;

  ros::Publisher is_ok_pub;

  ros::Subscriber status_sub = nh.subscribe("usv_status", 1, &usv_status_callback);

  /* sock_fd --- socket文件描述符 创建udp套接字*/
  // int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);

  sock_fd_init();
  addr_init_all();

  if(sock_fd_check())
  {
    perror("socket");
    exit(1);
  }

  /* 绑定socket */
  if(bind_check())
  {
    perror("bind error:");
    exit(1);
  }

  thread_on();

  double frequency = 10.0;
  ros::Rate rate(frequency);

  while(nh.ok())
  {

    std::stringstream ss;
    usv_status = "x:";

    /****把姿态以一定格式发送出去****/
    ss <<  usv_x;
    std::string asString = ss.str();// x:0.1,y:0.12345,o:0.123456789
    usv_status += asString;

    ss.str("");
    usv_status += ",y:";
    ss << usv_y;
    asString = ss.str();
    usv_status += asString;

    ss.str("");
    usv_status += ",orien:";
    ss << usv_orien;
    asString = ss.str();
    usv_status += asString;
    /***把姿态以一定格式发送出去****/

    ros::spinOnce();
    rate.sleep();
  }

  fd_close_all();

  return 0;
}


