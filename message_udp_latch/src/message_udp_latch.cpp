#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

#include <arpa/inet.h>

#include <sstream>
#include <string>
#include <thread>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <otter_control/usv_status.h>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

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

int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
struct sockaddr_in addr_serv;
int len;
int recv_num;
int send_num;
char send_buf[50] = "is_ok_from_b:0";
char recv_buf[50] = "is_ok_from_a:0";
struct sockaddr_in addr_client;
struct sockaddr_in addr_client_;
int flag = 0;
std::string lock_status = "is_ok_from_b:";

void recieve_thread_function(){

  int recv_num;
  while(1){
    recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);
    if(recv_num < 0)
    {
      perror("recvfrom error:");
      exit(1);
    }
    recv_buf[recv_num] = '\0';
    std::cout<<"server receive " << recv_num << " bytes: " << recv_buf;

    /*提取数字给is_ok_from_a*/
    if(recv_num > 0){

      std::string str(recv_buf);
      std::vector<std::string> vStr;
      boost::split(vStr, str, boost::is_any_of( ",:" ), boost::token_compress_on);
      is_ok_from_a = std::stod(vStr[1]);
      // std::cout << is_ok_from_a << std::endl;

    }
    addr_client_ = addr_client;

    flag = 1;
  }
}

void send_thread_function(){

  while(1){
    if(flag){
      send_num = sendto(sock_fd, lock_status.c_str(), lock_status.size(), 0, (struct sockaddr *)&addr_client_, len);
      if(send_num < 0)
      {
        perror("sendto error:");
        exit(1);
      }
    }
    sleep(1);
    
  }
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "messsage_udp_latch_node");

  ros::NodeHandle nh;

  ros::Publisher is_ok_pub;

  ros::Subscriber status_sub = nh.subscribe("usv_status", 1, &usv_status_callback);
  ros::Subscriber to_lock_sub = nh.subscribe("is_ok_from_b", 1, &to_lock_callback);
  is_ok_pub = nh.advertise<std_msgs::Bool>("is_ok_from_a", 1);
  /* sock_fd --- socket文件描述符 创建udp套接字*/
  // int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if(sock_fd < 0)
  {
    perror("socket");
    exit(1);
  }

  /* 将套接字和IP、端口绑定 */
  // struct sockaddr_in addr_serv;
  // int len;
  memset(&addr_serv, 0, sizeof(struct sockaddr_in));  //每个字节都用0填充
  addr_serv.sin_family = AF_INET;
  addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);
  addr_serv.sin_port = htons(SERV_PORT);
  
  // addr_client.sin_family = AF_INET;
  // addr_client.sin_port = htons(8000);    // 
  // inet_pton(AF_INET, "192.168.1.102", &addr_client.sin_addr.s_addr);
  
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

  std::thread recv_msg(recieve_thread_function);
  std::thread send_msg(send_thread_function);

  double frequency = 10.0;
  ros::Rate rate(frequency);

  while(nh.ok())
  {
    std::stringstream ss;
    // std::string 
    lock_status = "is_ok_from_b:";

    /*发送动锁信号*/
    ss <<  is_ok;
    std::string asString = ss.str();// x:0.1,y:0.12345,o:0.123456789
    lock_status += asString;


    std_msgs::Bool is_ok_from_a_;
    is_ok_from_a_.data = static_cast<bool>(is_ok_from_a);
    is_ok_pub.publish(is_ok_from_a_);

    ros::spinOnce();
    rate.sleep();
  }

  close(sock_fd);

  return 0;
}
