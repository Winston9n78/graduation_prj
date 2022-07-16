#ifndef _MESSAGE_UDP_STATUS_H_
#define _MESSAGE_UDP_STATUS_H_

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

#include <arpa/inet.h>

#include <sstream>
#include <string>
#include <thread>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include <otter_control/usv_status.h>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#define SERV_PORT_8001  8001
#define SERV_PORT_8002  8002
#define SERV_PORT_8003  8003
#define SERV_PORT_8004  8004
#define SERV_PORT_8005  8005
#define SERV_PORT_8006  8006
#define SERV_PORT_8007  8007
#define SERV_PORT_8008  8008
#define SERV_PORT_8009  8009

extern int sock_fd_8001;
extern int sock_fd_8002;
extern int sock_fd_8003;
extern int sock_fd_8004;
extern int sock_fd_8005;
extern int sock_fd_8006;
extern int sock_fd_8007;
extern int sock_fd_8008;
extern int sock_fd_8009;

extern int len;

extern std::string usv_status;

extern struct sockaddr_in addr_serv_8001;
extern struct sockaddr_in addr_client_8001, addr_client_8001_;

extern struct sockaddr_in addr_serv_8002;
extern struct sockaddr_in addr_client_8002, addr_client_8002_;

extern struct sockaddr_in addr_serv_8003;
extern struct sockaddr_in addr_client_8003, addr_client_8003_;

extern struct sockaddr_in addr_serv_8004;
extern struct sockaddr_in addr_client_8004, addr_client_8004_;

extern struct sockaddr_in addr_serv_8005;
extern struct sockaddr_in addr_client_8005, addr_client_8005_;

extern struct sockaddr_in addr_serv_8006;
extern struct sockaddr_in addr_client_8006, addr_client_8006_;

extern struct sockaddr_in addr_serv_8007;
extern struct sockaddr_in addr_client_8007, addr_client_8007_;

extern struct sockaddr_in addr_serv_8008;
extern struct sockaddr_in addr_client_8008, addr_client_8008_;

extern struct sockaddr_in addr_serv_8009;
extern struct sockaddr_in addr_client_8009, addr_client_8009_;

extern float usv_x, usv_y, usv_orien;

extern std_msgs::Float32MultiArray map_path;

void send_thread_function_8001();
void send_thread_function_8002();
void send_thread_function_8003();
void send_thread_function_8004();
void send_thread_function_8005();
void send_thread_function_8006();
void send_thread_function_8007();
void send_thread_function_8008();
void send_thread_function_8009();

void recieve_thread_function_8001();
void recieve_thread_function_8002();
void recieve_thread_function_8003();
void recieve_thread_function_8004();
void recieve_thread_function_8005();
void recieve_thread_function_8006();
void recieve_thread_function_8007();
void recieve_thread_function_8008();
void recieve_thread_function_8009();

void addr_init();
void serv_addr_init_all();
void sock_fd_init();
bool bind_check();
bool sock_fd_check();
void thread_on();
void fd_close_all();
void client_addr_init_all();

#endif