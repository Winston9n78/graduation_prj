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
#include <otter_control/usv_status.h>

std::stringstream ss;

#define SERV_PORT  8000

float usv_x = 0.1, usv_y = 0.12345, usv_orien = 0.123456;

void usv_status_callback(const otter_control::usv_status msg){
    usv_x = msg.position_z;
    usv_y = msg.position_x;
    usv_orien = msg.orientation_pitch;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "latch_control");

  ros::NodeHandle nh;

  ros::Publisher latch_signal, is_lock_ok;

  ros::Subscriber status_sub = nh.subscribe("usv_status", 1, &usv_status_callback);
  latch_signal = nh.advertise<std_msgs::Float32>("latch_command", 1);

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
  int flag = 0;

  double frequency = 1.0;
  ros::Rate rate(frequency);

  while(nh.ok())
  {
    std::stringstream ss;
    std::string usv_status = "x:";

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
    /****把姿态以一定格式发送出去****/

    std::cout << usv_status << std::endl;

    // printf("server wait:\n");
    
    if(!flag){
      recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);
      addr_client_ = addr_client;
      flag = 1;
    }
    
    if(recv_num < 0)
    {
      perror("recvfrom error:");
      exit(1);
    }

    recv_buf[recv_num] = '\0';
    printf("server receive %d bytes: %s\n", recv_num, recv_buf);

    send_num = sendto(sock_fd, usv_status.c_str(), usv_status.size(), 0, (struct sockaddr *)&addr_client_, len);

    if(send_num < 0)
    {
      perror("sendto error:");
      exit(1);
    }

    ros::spinOnce();
    rate.sleep();
  }

  close(sock_fd);

  return 0;
}
