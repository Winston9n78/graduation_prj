#include "message_udp_status/message_udp_status.h"

struct sockaddr_in addr_serv_8003;
struct sockaddr_in addr_client_8003, addr_client_8003_;

int sock_fd_8003;

int recv_num_8003;
int send_num_8003;
char send_buf_8003[50] = "x:0,y:0,angle:0";
char recv_buf_8003[50] = "x:0,y:0,angle:0";

int key;

int open_camera,close_camera;

void recieve_thread_function_8003(){
  int recv_num;
  while(1){
    recv_num = recvfrom(sock_fd_8003, recv_buf_8003, sizeof(recv_buf_8003), 0, (struct sockaddr *)&addr_client_8003, (socklen_t *)&len);
    if(recv_num < 0)
    {
      perror("recvfrom error:");
      exit(1);
    }
    recv_buf_8003[recv_num] = '\0';
    std::cout<<"server receive " << recv_num_8003 << " bytes: " << recv_buf_8003 << std::endl;
    
    /*解析命令并且存储命令*/
    if(recv_num > 0)
    {
        std::string str(recv_buf_8003);
        std::vector<std::string> vStr;
        boost::split(vStr, str, boost::is_any_of( ",:" ), boost::token_compress_on);
        for(int i = 0; i < vStr.size(); i++)
        {
            std::cout << vStr[i] << std::endl;
            if(vStr[i] == "open") key = 1;
            if(vStr[i] == "close") key = 2;
            i+=1;
            double recv_data = std::stod(vStr[i]);
            std::cout << recv_data << std::endl;
            switch (key)
            {
                case 1:open_camera = recv_data;/* code */break;
                case 2:close_camera = recv_data;/* code */break;
                default:
                break;
            }
        
        }

    }
  }
}

void send_thread_function_8003(){
    while(1){

      //   send_num_8003 = sendto(sock_fd_8003, usv_status.c_str(), usv_status.size(), 0, (struct sockaddr *)&addr_client_8003, len);
      //   if(send_num_8003 < 0)
      //   {
      //       perror("sendto error:");
      //       exit(1);
      //   }
        
      // sleep(1);
    
  }
}