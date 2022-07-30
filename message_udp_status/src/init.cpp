#include "message_udp_status/message_udp_status.h"

void serv_addr_init(struct sockaddr_in &sockaddr, int port){
  memset(&sockaddr, 0, sizeof(struct sockaddr_in));  //每个字节都用0填充
  sockaddr.sin_family = AF_INET;
  sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  sockaddr.sin_port = htons(port);
}

void client_addr_init(struct sockaddr_in &sockaddr, int port){
  memset(&sockaddr, 0, sizeof(struct sockaddr_in));  //每个字节都用0填充
  sockaddr.sin_family = AF_INET;
  sockaddr.sin_port = htons(port);
  inet_pton(AF_INET, "192.168.1.103", &sockaddr.sin_addr.s_addr);
}

void sock_fd_init(){
    sock_fd_8001 = socket(AF_INET, SOCK_DGRAM, 0);
    sock_fd_8002 = socket(AF_INET, SOCK_DGRAM, 0);
    sock_fd_8003 = socket(AF_INET, SOCK_DGRAM, 0);
    sock_fd_8004 = socket(AF_INET, SOCK_DGRAM, 0);
    sock_fd_8005 = socket(AF_INET, SOCK_DGRAM, 0);
    sock_fd_8006 = socket(AF_INET, SOCK_DGRAM, 0);
    sock_fd_8007 = socket(AF_INET, SOCK_DGRAM, 0);
    sock_fd_8008 = socket(AF_INET, SOCK_DGRAM, 0);
    sock_fd_8009 = socket(AF_INET, SOCK_DGRAM, 0);
}

void serv_addr_init_all(){
    serv_addr_init(addr_serv_8001, SERV_PORT_8001);
    serv_addr_init(addr_serv_8002, SERV_PORT_8002);
    serv_addr_init(addr_serv_8003, SERV_PORT_8003);
    serv_addr_init(addr_serv_8004, SERV_PORT_8004);
    serv_addr_init(addr_serv_8005, SERV_PORT_8005);
    serv_addr_init(addr_serv_8006, SERV_PORT_8006);
    serv_addr_init(addr_serv_8007, SERV_PORT_8007);
    serv_addr_init(addr_serv_8008, SERV_PORT_8008);
    serv_addr_init(addr_serv_8009, SERV_PORT_8009);
}

void client_addr_init_all(){
    client_addr_init(addr_client_8001, SERV_PORT_9001);
    client_addr_init(addr_client_8002, SERV_PORT_9002);
    client_addr_init(addr_client_8003, SERV_PORT_9003);
    client_addr_init(addr_client_8004, SERV_PORT_9004);
    client_addr_init(addr_client_8005, SERV_PORT_9005);
    client_addr_init(addr_client_8006, SERV_PORT_9006);
    client_addr_init(addr_client_8007, SERV_PORT_9007);
    client_addr_init(addr_client_8008, SERV_PORT_9008);
    client_addr_init(addr_client_8009, SERV_PORT_9009);
}

bool bind_check(){

    if(bind(sock_fd_8001, (struct sockaddr *)&addr_serv_8001, sizeof(addr_serv_8001)) < 0
        ||bind(sock_fd_8002, (struct sockaddr *)&addr_serv_8002, sizeof(addr_serv_8002)) < 0
        ||bind(sock_fd_8003, (struct sockaddr *)&addr_serv_8003, sizeof(addr_serv_8003)) < 0
        ||bind(sock_fd_8004, (struct sockaddr *)&addr_serv_8004, sizeof(addr_serv_8004)) < 0
        ||bind(sock_fd_8005, (struct sockaddr *)&addr_serv_8005, sizeof(addr_serv_8005)) < 0
        ||bind(sock_fd_8006, (struct sockaddr *)&addr_serv_8006, sizeof(addr_serv_8006)) < 0
        ||bind(sock_fd_8007, (struct sockaddr *)&addr_serv_8007, sizeof(addr_serv_8007)) < 0
        ||bind(sock_fd_8008, (struct sockaddr *)&addr_serv_8008, sizeof(addr_serv_8008)) < 0
        ||bind(sock_fd_8009, (struct sockaddr *)&addr_serv_8009, sizeof(addr_serv_8009)) < 0
    ) return true;
    else return false;
}

bool sock_fd_check(){
    if(sock_fd_8001 < 0 || sock_fd_8002 < 0 ||sock_fd_8003 < 0
        ||sock_fd_8004 < 0 || sock_fd_8005 < 0 ||sock_fd_8006 < 0
        ||sock_fd_8007 < 0 || sock_fd_8008 < 0 || sock_fd_8009 < 0
    ) return true;
    else return false;
}


void thread_on(){

    // std::thread recv_msg_8001(recieve_thread_function_8001);
    // std::thread send_msg_8001(send_thread_function_8001);

//   std::thread recv_msg_8002(recieve_thread_function_8002);
//   std::thread send_msg_8002(send_thread_function_8002);

//   std::thread recv_msg_8003(recieve_thread_function_8003);
//   std::thread send_msg_8003(send_thread_function_8003);

//   std::thread recv_msg_8004(recieve_thread_function_8004);
//   std::thread send_msg_8004(send_thread_function_8004);

//   std::thread recv_msg_8005(recieve_thread_function_8005);
//   std::thread send_msg_8005(send_thread_function_8005);

//   std::thread recv_msg_8006(recieve_thread_function_8006);
//   std::thread send_msg_8006(send_thread_function_8006);

//   std::thread recv_msg_8007(recieve_thread_function_8007);
//   std::thread send_msg_8007(send_thread_function_8007);

//   std::thread recv_msg_8008(recieve_thread_function_8008);
//   std::thread send_msg_8008(send_thread_function_8008);

//   std::thread recv_msg_8009(recieve_thread_function_8009);
//   std::thread send_msg_8009(send_thread_function_8009);

}

void fd_close_all(){
    close(sock_fd_8001);
    close(sock_fd_8002);
    close(sock_fd_8003);
    close(sock_fd_8004);
    close(sock_fd_8005);
    close(sock_fd_8006);
    close(sock_fd_8007);
    close(sock_fd_8008);
    close(sock_fd_8009);
}
