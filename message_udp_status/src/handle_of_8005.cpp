#include "message_udp_status/message_udp_status.h"

struct sockaddr_in addr_serv_8005;
struct sockaddr_in addr_client_8005, addr_client_8005_;

int sock_fd_8005;

int recv_num_8005;
int send_num_8005;
char send_buf_8005[50] = "x:0,y:0,angle:0";
char recv_buf_8005[50] = "x:0,y:0,angle:0";

int flag_8005 = 1;

void recieve_thread_function_8005(){
  int recv_num;
  while(1){
    recv_num = recvfrom(sock_fd_8005, recv_buf_8005, sizeof(recv_buf_8005), 0, (struct sockaddr *)&addr_client_8005, (socklen_t *)&len);
    if(recv_num < 0)
    {
      perror("recvfrom error:");
      exit(1);
    }
    recv_buf_8005[recv_num] = '\0';
    std::cout<<"server receive " << recv_num_8005 << " bytes: " << recv_buf_8005 << std::endl;
    
    /*解析命令并且存储命令*/
    if(recv_num > 0)
    {

    }

    addr_client_8005_ = addr_client_8005;

    flag_8005 = 1;
  }
}

void send_thread_function_8005(){
    while(1){

        if(flag_8005){
            send_num_8005 = sendto(sock_fd_8005, usv_status.c_str(), usv_status.size(), 0, (struct sockaddr *)&addr_client_8005_, len);
            if(send_num_8005 < 0)
            {
                perror("sendto error:");
                exit(1);
            }
        }

    sleep(1);
    
  }
}