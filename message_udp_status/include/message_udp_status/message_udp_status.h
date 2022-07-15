#ifndef MESSAGE_UDP_STATUS_H_
#define MESSAGE_UDP_STATUS_H_

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

int sock_fd_8001 = socket(AF_INET, SOCK_DGRAM, 0);
int sock_fd_8002 = socket(AF_INET, SOCK_DGRAM, 0);
int sock_fd_8003 = socket(AF_INET, SOCK_DGRAM, 0);
int sock_fd_8004 = socket(AF_INET, SOCK_DGRAM, 0);
int sock_fd_8005 = socket(AF_INET, SOCK_DGRAM, 0);
int sock_fd_8006 = socket(AF_INET, SOCK_DGRAM, 0);
int sock_fd_8007 = socket(AF_INET, SOCK_DGRAM, 0);
int sock_fd_8008 = socket(AF_INET, SOCK_DGRAM, 0);
int sock_fd_8009 = socket(AF_INET, SOCK_DGRAM, 0);

struct sockaddr_in addr_serv_8001,addr_serv_8002,addr_serv_8003,addr_serv_8004,addr_serv_8005;
struct sockaddr_in addr_serv_8006,addr_serv_8007,addr_serv_8008,addr_serv_8009;
struct sockaddr_in addr_client_8001, addr_client_8001_;
struct sockaddr_in addr_client_8002, addr_client_8002_;
struct sockaddr_in addr_client_8003, addr_client_8003_;
struct sockaddr_in addr_client_8004, addr_client_8004_;
struct sockaddr_in addr_client_8005, addr_client_8005_;
struct sockaddr_in addr_client_8006, addr_client_8006_;
struct sockaddr_in addr_client_8007, addr_client_8007_;
struct sockaddr_in addr_client_8008, addr_client_8008_;
struct sockaddr_in addr_client_8009, addr_client_8009_;

void send_thread_function_8001(){}
void send_thread_function_8002(){}
void send_thread_function_8003(){}
void send_thread_function_8004(){}
void send_thread_function_8005(){}
void send_thread_function_8006(){}
void send_thread_function_8007(){}
void send_thread_function_8008(){}
void send_thread_function_8009(){}

void recieve_thread_function_8001(){}
void recieve_thread_function_8002(){}
void recieve_thread_function_8003(){}
void recieve_thread_function_8004(){}
void recieve_thread_function_8005(){}
void recieve_thread_function_8006(){}
void recieve_thread_function_8007(){}
void recieve_thread_function_8008(){}
void recieve_thread_function_8009(){}

#endif