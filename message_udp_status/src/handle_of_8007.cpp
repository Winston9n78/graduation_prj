#include "message_udp_status/message_udp_status.h"

struct sockaddr_in addr_serv_8007;
struct sockaddr_in addr_client_8007, addr_client_8007_;

int sock_fd_8007;

int recv_num_8007;
int send_num_8007;
char send_buf_8007[50] = "x:0,y:0,angle:0";
char recv_buf_8007[50] = "x:0,y:0,angle:0";

int flag_8007 = 1;

void recieve_thread_function_8007(){
  int recv_num;
  while(1){
    recv_num = recvfrom(sock_fd_8007, recv_buf_8007, sizeof(recv_buf_8007), 0, (struct sockaddr *)&addr_client_8007, (socklen_t *)&len);
    if(recv_num < 0)
    {
      perror("recvfrom error:");
      exit(1);
    }
    recv_buf_8007[recv_num] = '\0';
    std::cout<<"server receive " << recv_num_8007 << " bytes: " << recv_buf_8007 << std::endl;
    
    /*解析命令并且存储命令*/
    if(recv_num > 0)
    {

    }

    addr_client_8007_ = addr_client_8007;

    flag_8007 = 1;
  }
}

void send_thread_function_8007(){
    while(1){

        if(flag_8007){
            send_num_8007 = sendto(sock_fd_8007, usv_status.c_str(), usv_status.size(), 0, (struct sockaddr *)&addr_client_8007_, len);
            if(send_num_8007 < 0)
            {
                perror("sendto error:");
                exit(1);
            }
        }

    sleep(1);
    
  }
}