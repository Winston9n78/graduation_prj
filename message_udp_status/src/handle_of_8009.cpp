#include "message_udp_status/message_udp_status.h"

struct sockaddr_in addr_serv_8009;
struct sockaddr_in addr_client_8009, addr_client_8009_;

int sock_fd_8009;

int recv_num_8009;
int send_num_8009;
char send_buf_8009[50] = "x:0,y:0,angle:0";
char recv_buf_8009[50] = "x:0,y:0,angle:0";

void recieve_thread_function_8009(){
  int recv_num;
  while(1){
    recv_num = recvfrom(sock_fd_8009, recv_buf_8009, sizeof(recv_buf_8009), 0, (struct sockaddr *)&addr_client_8009, (socklen_t *)&len);
    if(recv_num < 0)
    {
      perror("recvfrom error:");
      exit(1);
    }
    recv_buf_8009[recv_num] = '\0';
    std::cout<<"server receive " << recv_num_8009 << " bytes: " << recv_buf_8009 << std::endl;
    
    /*解析命令并且存储命令*/
    if(recv_num > 0)
    {

    }
  }
}

void send_thread_function_8009(){
    while(1){

        send_num_8009 = sendto(sock_fd_8009, usv_status.c_str(), usv_status.size(), 0, (struct sockaddr *)&addr_client_8009, len);
        if(send_num_8009 < 0)
        {
            perror("sendto error:");
            exit(1);
        }

        sleep(1);
    
  }
}