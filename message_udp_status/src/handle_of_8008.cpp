#include "message_udp_status/message_udp_status.h"

struct sockaddr_in addr_serv_8008;
struct sockaddr_in addr_client_8008, addr_client_8008_;

int sock_fd_8008;

int recv_num_8008;
int send_num_8008;
char send_buf_8008[50] = "x:0,y:0,angle:0";
char recv_buf_8008[50] = "x:0,y:0,angle:0";

void recieve_thread_function_8008(){

  int recv_num;
  map_path.data.clear(); //清除栈
  while(1){
    recv_num = recvfrom(sock_fd_8008, recv_buf_8008, sizeof(recv_buf_8008), 0, (struct sockaddr *)&addr_client_8008, (socklen_t *)&len);
    if(recv_num < 0)
    {
      perror("recvfrom error:");
      exit(1);
    }
    recv_buf_8008[recv_num] = '\0';
    std::cout<<"server receive " << recv_num_8008 << " bytes: " << recv_buf_8008 << std::endl;
    
    /*解析命令并且存储命令*/
    if(recv_num > 0)
    {
        std::string str(recv_buf_8008);
        std::vector<std::string> vStr;
        boost::split(vStr, str, boost::is_any_of( " " ), boost::token_compress_on);
        for(int i = 0; i < vStr.size(); i++)
        {
            map_path.data.push_back(std::stod(vStr[i]));
        }
    }
  }
}

void send_thread_function_8008(){
    while(1){

        send_num_8008 = sendto(sock_fd_8008, usv_status.c_str(), usv_status.size(), 0, (struct sockaddr *)&addr_client_8008, len);
        if(send_num_8008 < 0)
        {
            perror("sendto error:");
            exit(1);
        }

        sleep(1);
    
  }
}