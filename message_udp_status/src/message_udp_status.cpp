#include "message_udp_status/message_udp_status.h"

std::string usv_status;

float usv_x = 0.1, usv_y = 0.12345, usv_orien = 0.123456;

int is_ok = 0;

int is_ok_from_a = 0; // 将接收到的另一艘船的解码的变量

int len;
int recv_num;
int send_num;
char send_buf[50] = "x:0,y:0,angle:0";
char recv_buf[50] = "x:0,y:0,angle:0";

int flag = 0;

void usv_status_callback(const otter_control::usv_status msg){
    usv_x = msg.position_z;
    usv_y = msg.position_x;
    usv_orien = msg.orientation_pitch;
}

void recieve_thread_function_8001(){

  int recv_num;
  while(1){
    recv_num = recvfrom(sock_fd_8001, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client_8001, (socklen_t *)&len);
    if(recv_num < 0)
    {
      perror("recvfrom error:");
      exit(1);
    }
    recv_buf[recv_num] = '\0';
    std::cout<<"server receive " << recv_num << " bytes: " << recv_buf << std::endl;
    
    /*解析命令并且存储命令*/
    if(recv_num > 0)
    {
      std::string str(recv_buf);
      std::vector<std::string> vStr;
      boost::split(vStr, str, boost::is_any_of( ",:" ), boost::token_compress_on);
      for(int i = 0; i < vStr.size(); i++)
      {
        std::cout << vStr[1] << std::endl;
        double recv_data = std::stod(vStr[1]);
        std::cout << recv_data << std::endl;
        switch (i)
        {
          case 1:/* code */break;
          case 2:/* code */break;
          case 3:/* code */break;
          case 4:/* code */break;
          case 5:/* code */break;
          case 6:/* code */break;
          case 7:/* code */break;
          case 8:/* code */break;
          case 9:/* code */break;
          
          default:
            break;
        }
      }
    }

    addr_client_8001_ = addr_client_8001;

    flag = 1;
  }
}

void send_thread_function_8001(){

  while(1){
    if(flag){
      send_num = sendto(sock_fd_8001, usv_status.c_str(), usv_status.size(), 0, (struct sockaddr *)&addr_client_8001_, len);
      if(send_num < 0)
      {
        perror("sendto error:");
        exit(1);
      }
    }
    sleep(1);
    
  }
}


void addr_init(sockaddr_in &sockaddr, int port){
  memset(&sockaddr, 0, sizeof(struct sockaddr_in));  //每个字节都用0填充
  sockaddr.sin_family = AF_INET;
  sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  sockaddr.sin_port = htons(port);
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "messsage_udp_latch_node");

  ros::NodeHandle nh;

  ros::Publisher is_ok_pub;

  ros::Subscriber status_sub = nh.subscribe("usv_status", 1, &usv_status_callback);

  /* sock_fd --- socket文件描述符 创建udp套接字*/
  // int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if(sock_fd_8001 < 0 || sock_fd_8002 < 0 ||sock_fd_8003 < 0
    ||sock_fd_8004 < 0 || sock_fd_8005 < 0 ||sock_fd_8006 < 0
    ||sock_fd_8007 < 0 || sock_fd_8008 < 0 || sock_fd_8009 < 0
  )
  {
    perror("socket");
    exit(1);
  }

  /* 将套接字和IP、端口绑定 */
  // struct sockaddr_in addr_serv;
  // int len;
  // memset(&addr_serv_8001, 0, sizeof(struct sockaddr_in));  //每个字节都用0填充
  // addr_serv_8001.sin_family = AF_INET;
  // addr_serv_8001.sin_addr.s_addr = htonl(INADDR_ANY);
  // addr_serv_8001.sin_port = htons(SERV_PORT_8001);
  addr_init(addr_serv_8001, SERV_PORT_8001);
  addr_init(addr_serv_8002, SERV_PORT_8002);
  addr_init(addr_serv_8003, SERV_PORT_8003);
  addr_init(addr_serv_8004, SERV_PORT_8004);
  addr_init(addr_serv_8005, SERV_PORT_8005);
  addr_init(addr_serv_8006, SERV_PORT_8006);
  addr_init(addr_serv_8007, SERV_PORT_8007);
  addr_init(addr_serv_8008, SERV_PORT_8008);
  addr_init(addr_serv_8009, SERV_PORT_8009);

  len = sizeof(addr_serv_8001);

  /* 绑定socket */
  if(bind(sock_fd_8001, (struct sockaddr *)&addr_serv_8001, sizeof(addr_serv_8001)) < 0
    ||bind(sock_fd_8002, (struct sockaddr *)&addr_serv_8002, sizeof(addr_serv_8002)) < 0
    ||bind(sock_fd_8003, (struct sockaddr *)&addr_serv_8003, sizeof(addr_serv_8003)) < 0
    ||bind(sock_fd_8004, (struct sockaddr *)&addr_serv_8004, sizeof(addr_serv_8004)) < 0
    ||bind(sock_fd_8005, (struct sockaddr *)&addr_serv_8005, sizeof(addr_serv_8005)) < 0
    ||bind(sock_fd_8006, (struct sockaddr *)&addr_serv_8006, sizeof(addr_serv_8006)) < 0
    ||bind(sock_fd_8007, (struct sockaddr *)&addr_serv_8007, sizeof(addr_serv_8007)) < 0
    ||bind(sock_fd_8008, (struct sockaddr *)&addr_serv_8008, sizeof(addr_serv_8008)) < 0
    ||bind(sock_fd_8009, (struct sockaddr *)&addr_serv_8009, sizeof(addr_serv_8009)) < 0
  )
  {
    perror("bind error:");
    exit(1);
  }

  std::thread recv_msg_8001(recieve_thread_function_8001);
  std::thread send_msg_8001(send_thread_function_8001);

  std::thread recv_msg_8002(recieve_thread_function_8002);
  std::thread send_msg_8002(send_thread_function_8002);

  std::thread recv_msg_8003(recieve_thread_function_8003);
  std::thread send_msg_8003(send_thread_function_8003);

  std::thread recv_msg_8004(recieve_thread_function_8004);
  std::thread send_msg_8004(send_thread_function_8004);

  std::thread recv_msg_8005(recieve_thread_function_8005);
  std::thread send_msg_8005(send_thread_function_8005);

  std::thread recv_msg_8006(recieve_thread_function_8006);
  std::thread send_msg_8006(send_thread_function_8006);

  std::thread recv_msg_8007(recieve_thread_function_8007);
  std::thread send_msg_8007(send_thread_function_8007);

  std::thread recv_msg_8008(recieve_thread_function_8008);
  std::thread send_msg_8008(send_thread_function_8008);

  std::thread recv_msg_8009(recieve_thread_function_8009);
  std::thread send_msg_8009(send_thread_function_8009);

  double frequency = 10.0;
  ros::Rate rate(frequency);

  while(nh.ok())
  {

    std::stringstream ss;
    usv_status = "x:";

    /****把姿态以一定格式发送出去****/
    ss <<  usv_x;
    std::string asString = ss.str();// x:0.1,y:0.12345,o:0.123456789
    usv_status += asString;

    ss.str("");
    usv_status += ",y:";
    ss << usv_y;
    asString = ss.str();
    usv_status += asString;

    ss.str("");
    usv_status += ",orien:";
    ss << usv_orien;
    asString = ss.str();
    usv_status += asString;
    /***把姿态以一定格式发送出去****/

    ros::spinOnce();
    rate.sleep();
  }

  close(sock_fd_8001);
  close(sock_fd_8002);
  close(sock_fd_8003);
  close(sock_fd_8004);
  close(sock_fd_8005);
  close(sock_fd_8006);
  close(sock_fd_8007);
  close(sock_fd_8008);
  close(sock_fd_8009);

  return 0;
}


