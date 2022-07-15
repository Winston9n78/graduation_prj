#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

#include <sstream>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <otter_control/usv_status.h>

std::stringstream ss;

#define SERV_PORT  8000

float usv_x = 0.1, usv_y = 0.12345, usv_orien = 0.123456;

int is_ok = 0;

int is_ok_from_a = 0; // 将接收到的另一艘船的解码的变量

void usv_status_callback(const otter_control::usv_status msg){
    usv_x = msg.position_z;
    usv_y = msg.position_x;
    usv_orien = msg.orientation_pitch;
}

void to_lock_callback(const std_msgs::Bool msg){
    is_ok = msg.data;
}

int main(int argc, char* argv[])
{

  ros::init(argc, argv, "messsage_udp_status_node");

  ros::NodeHandle nh;

  ros::Publisher is_ok_pub;

  ros::Subscriber status_sub = nh.subscribe("usv_status", 1, &usv_status_callback);
  // ros::Subscriber to_lock_sub = nh.subscribe("is_ok_from_b", 1, &to_lock_callback);
  // is_ok_pub = nh.advertise<std_msgs::Bool>("is_ok_from_a", 1);
  /* sock_fd --- socket文件描述符 创建udp套接字*/
  int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if(sock_fd < 0)
  {
    perror("socket");
    exit(1);
  }

  /* 将套接字和IP、端口绑定 */
  struct sockaddr_in addr_serv;
  int len;
  memset(&addr_serv, 0, sizeof(struct sockaddr_in));  //每个字节都用0填充
  addr_serv.sin_family = AF_INET;
  addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);
  addr_serv.sin_port = htons(SERV_PORT);
  
  //addr_serv.sin_family = AF_INET;  　　　　　　　　　　　 //使用IPV4地址
  //addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);  //自动获取IP地址      
  //addr_serv.sin_port = htons(SERV_PORT);  　　　　　　　 //端口
  /* INADDR_ANY表示不管是哪个网卡接收到数据，只要目的端口是SERV_PORT，就会被该应用程序接收到 */

  len = sizeof(addr_serv);

  /* 绑定socket */
  if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
  {
    perror("bind error:");
    exit(1);
  }

  int recv_num;
  int send_num;
  char send_buf[50] = "";
  char recv_buf[20];
  struct sockaddr_in addr_client;
  struct sockaddr_in addr_client_;
  
  addr_client.sin_family = AF_INET;
  addr_client.sin_addr.s_addr = htonl(INADDR_ANY);
  addr_client.sin_port = htons(atoi(argv[1]));

  int flag = 0;

  double frequency = 10.0;
  ros::Rate rate(frequency);

  while(nh.ok())
  {
    std::stringstream ss;
    std::string usv_status = "x:";

    /*发送动锁信号*/
    // ss <<  is_ok;
    // std::string asString = ss.str();// x:0.1,y:0.12345,o:0.123456789
    // lock_status += asString;

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

    // std::cout << lock_status << std::endl;

    // printf("server wait:\n");
    
    // if(!flag){
    recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), MSG_PEEK | MSG_DONTWAIT, (struct sockaddr *)&addr_client, (socklen_t *)&len);
    // addr_client_ = addr_client;
      // flag = 1;
    // }

    if(recv_num <= 0)
    {
      perror("recvfrom nothing:");
      break;
      //exit(1);
    }

    recv_buf[recv_num] = '\0';
    printf("server receive %d bytes: %s\n", recv_num, recv_buf);

    /*提取数字给is_ok_from_a*/
    // const char *d = ":";
    // char *p;
    // p = std::strtok(recv_buf, d);
    // p = std::strtok(NULL, d);
    // is_ok_from_a = atoi(p);

    send_num = sendto(sock_fd, usv_status.c_str(), usv_status.size(), 0, (struct sockaddr *)&addr_client, len);

    if(send_num < 0)
    {
      perror("sendto error:");
      exit(1);
    }

    // std_msgs::Bool is_ok_from_a_;
    // is_ok_from_a_.data = static_cast<bool>(is_ok_from_a);
    // is_ok_pub.publish(is_ok_from_a_);

    ros::spinOnce();
    rate.sleep();
  }

  close(sock_fd);

  return 0;
}
