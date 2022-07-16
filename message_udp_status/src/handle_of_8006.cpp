#include "message_udp_status/message_udp_status.h"

struct sockaddr_in addr_serv_8006;
struct sockaddr_in addr_client_8006, addr_client_8006_;

int sock_fd_8006;

int recv_num_8006;
int send_num_8006;
char send_buf_8006[50] = "x:0,y:0,angle:0";
char recv_buf_8006[50] = "x:0,y:0,angle:0";

int flag_8006 = 1;

void recieve_thread_function_8006(){
  int recv_num;
  while(1){
    recv_num = recvfrom(sock_fd_8006, recv_buf_8006, sizeof(recv_buf_8006), 0, (struct sockaddr *)&addr_client_8006, (socklen_t *)&len);
    if(recv_num < 0)
    {
      perror("recvfrom error:");
      exit(1);
    }
    recv_buf_8006[recv_num] = '\0';
    std::cout<<"server receive " << recv_num_8006 << " bytes: " << recv_buf_8006 << std::endl;
    
    /*解析命令并且存储命令*/
    if(recv_num > 0)
    {

    }

    addr_client_8006_ = addr_client_8006;

    flag_8006 = 1;
  }
}

void send_thread_function_8006(){
    while(1){

        if(flag_8006){
            send_num_8006 = sendto(sock_fd_8006, usv_status.c_str(), usv_status.size(), 0, (struct sockaddr *)&addr_client_8006_, len);
            if(send_num_8006 < 0)
            {
                perror("sendto error:");
                exit(1);
            }
        }

    sleep(1);
    
  }
}