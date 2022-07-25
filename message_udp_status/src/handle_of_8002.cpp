#include "message_udp_status/message_udp_status.h"

struct sockaddr_in addr_serv_8002;
struct sockaddr_in addr_client_8002, addr_client_8002_;

int sock_fd_8002;

int recv_num_8002;
int send_num_8002;
// char send_buf_8002[512] = "x:0,y:0,angle:0";
char recv_buf_8002[50] = "x:0,y:0,angle:0";

void recieve_thread_function_8002(){
  int recv_num;
  while(1){
    recv_num = recvfrom(sock_fd_8002, recv_buf_8002, sizeof(recv_buf_8002), 0, (struct sockaddr *)&addr_client_8002, (socklen_t *)&len);
    if(recv_num < 0)
    {
      perror("recvfrom error:");
      exit(1);
    }
    recv_buf_8002[recv_num] = '\0';
    std::cout<<"server receive " << recv_num_8002 << " bytes: " << recv_buf_8002 << std::endl;
    
    /*解析命令并且存储命令*/
    if(recv_num > 0)
    {

    }
  }
}

//发送代码状态信息，启动过程等，报错等

void send_thread_function_8002(){
    while(1){

        if(commander_order_reset.data == true){
          char send_buf_8002[50] = "B reset system";
          send_num_8002 = sendto(sock_fd_8002, send_buf_8002, sizeof(send_buf_8002), 0, (struct sockaddr *)&addr_client_8002, len);
        }

        if(commander_order_latch.data == true){
          char send_buf_8002[50] = "B latch_turn_on";
          send_num_8002 = sendto(sock_fd_8002, send_buf_8002, sizeof(send_buf_8002), 0, (struct sockaddr *)&addr_client_8002, len);         
        }
        else{
          char send_buf_8002[50] = "B latch_turn_off";
          send_num_8002 = sendto(sock_fd_8002, send_buf_8002, sizeof(send_buf_8002), 0, (struct sockaddr *)&addr_client_8002, len);
        }
        if(commander_order_guidance.data == true){
          char send_buf_8002[50] = "B guidance_turn_on";
          send_num_8002 = sendto(sock_fd_8002, send_buf_8002, sizeof(send_buf_8002), 0, (struct sockaddr *)&addr_client_8002, len);          
        }else{
          char send_buf_8002[50] = "B guidance_turn_on_off";
          send_num_8002 = sendto(sock_fd_8002, send_buf_8002, sizeof(send_buf_8002), 0, (struct sockaddr *)&addr_client_8002, len);
        }

        
        if(send_num_8002 < 0)
        {
            perror("sendto error:");
            exit(1);
        }

    sleep(1);
  }
}