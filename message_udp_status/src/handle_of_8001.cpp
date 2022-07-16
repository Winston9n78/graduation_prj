#include "message_udp_status/message_udp_status.h"

int len;
int recv_num;
int send_num;
char send_buf[50] = "x:0,y:0,angle:0";
char recv_buf[50] = "x:0,y:0,angle:0";

int flag = 0;

struct sockaddr_in addr_serv_8001;
struct sockaddr_in addr_client_8001, addr_client_8001_;

std::string usv_status;

int sock_fd_8001;

void recieve_thread_function_8001(){

  int recv_num;
  while(1){
    recv_num = recvfrom(sock_fd_8001, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client_8001, (socklen_t *)&len);
    if(recv_num < 0)
    {
      perror("recvfrom error:");
      exit(1);
    }
    recv_buf[recv_num] = '\0';
    std::cout<<"server receive " << recv_num << " bytes: " << recv_buf << std::endl;
    
    /*解析命令并且存储命令*/
    if(recv_num > 0)
    {
      std::string str(recv_buf);
      std::vector<std::string> vStr;
      boost::split(vStr, str, boost::is_any_of( ",:" ), boost::token_compress_on);
      for(int i = 0; i < vStr.size(); i++)
      {
        std::cout << vStr[1] << std::endl;
        double recv_data = std::stod(vStr[1]);
        std::cout << recv_data << std::endl;
        switch (i)
        {
          case 1:/* code */break;
          case 2:/* code */break;
          case 3:/* code */break;
          case 4:/* code */break;
          case 5:/* code */break;
          case 6:/* code */break;
          case 7:/* code */break;
          case 8:/* code */break;
          case 9:/* code */break;
          
          default:
            break;
        }
      }
    }

    addr_client_8001_ = addr_client_8001;

    flag = 1;
  }
}

void send_thread_function_8001(){

  while(1){
    if(flag){
      send_num = sendto(sock_fd_8001, usv_status.c_str(), usv_status.size(), 0, (struct sockaddr *)&addr_client_8001_, len);
      if(send_num < 0)
      {
        perror("sendto error:");
        exit(1);
      }
    }
    sleep(1);
    
  }
}