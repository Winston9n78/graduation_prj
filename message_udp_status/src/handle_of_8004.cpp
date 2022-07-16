#include "message_udp_status/message_udp_status.h"

struct sockaddr_in addr_serv_8004;
struct sockaddr_in addr_client_8004, addr_client_8004_;

int sock_fd_8004;

int recv_num_8004;
int send_num_8004;
char send_buf_8004[50] = "x:0,y:0,angle:0";
char recv_buf_8004[50] = "x:0,y:0,angle:0";

int flag_8004 = 1;

void recieve_thread_function_8004(){
  int recv_num;
  while(1){
    recv_num = recvfrom(sock_fd_8004, recv_buf_8004, sizeof(recv_buf_8004), 0, (struct sockaddr *)&addr_client_8004, (socklen_t *)&len);
    if(recv_num < 0)
    {
      perror("recvfrom error:");
      exit(1);
    }
    recv_buf_8004[recv_num] = '\0';
    std::cout<<"server receive " << recv_num_8004 << " bytes: " << recv_buf_8004 << std::endl;
    
    /*解析命令并且存储命令*/
    if(recv_num > 0)
    {

    }

    addr_client_8004_ = addr_client_8004;

    flag_8004 = 1;
  }
}

void send_thread_function_8004(){
    while(1){

        if(flag_8004){
            send_num_8004 = sendto(sock_fd_8004, usv_status.c_str(), usv_status.size(), 0, (struct sockaddr *)&addr_client_8004_, len);
            if(send_num_8004 < 0)
            {
                perror("sendto error:");
                exit(1);
            }
        }

    sleep(1);
    
  }
}