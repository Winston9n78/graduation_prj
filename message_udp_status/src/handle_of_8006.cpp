#include "message_udp_status/message_udp_status.h"

struct sockaddr_in addr_serv_8006;
struct sockaddr_in addr_client_8006, addr_client_8006_;

int sock_fd_8006;

int recv_num_8006;
int send_num_8006;
char send_buf_8006[50] = "x:0,y:0,angle:0";
char recv_buf_8006[50] = "x:0,y:0,angle:0";

std::vector<uchar> buff(65546);

cv::Mat frame;

void recieve_thread_function_8006(){
  int recv_num;
  unsigned char buf[426672];
  while(1){

    // recv_num = recvfrom(sock_fd_8006, recv_buf_8006, sizeof(recv_buf_8006), 0, (struct sockaddr *)&addr_client_8006, (socklen_t *)&len);
    recv_num = recvfrom(sock_fd_8006, buf, sizeof(buf), 0, (struct sockaddr *)&addr_serv_8006, (socklen_t *)&len);
    if(recv_num < 0)
    {
      perror("recvfrom error:");
      exit(1);
    }
    printf("recv.....%d\n", recv_num);
    int pos = 0;

    std::vector<uchar> decode;

    while (pos < recv_num)
		{
			decode.push_back(buf[pos++]);//存入vector
		}
		buf[recv_num] = 0;

		// image = imdecode(decode, CV_LOAD_IMAGE_COLOR);//图像解码

    frame = cv::imdecode(decode, CV_LOAD_IMAGE_COLOR);

    cv::imshow("view", frame);
    cv::waitKey(10);

    // recv_buf_8006[recv_num] = '\0';
    // std::cout<<"server receive " << recv_num_8006 << " bytes: " << recv_buf_8006 << std::endl;
    
    /*解析命令并且存储命令*/
    // if(recv_num > 0)
    // {

    // }
  }
}

void send_thread_function_8006(){
    while(1){
        // sleep(10);
        // (const uchar*)buf.data()
        if(recv_flag){
          send_num_8006 = sendto(sock_fd_8006, encodeImg, buf.size(), 0, (struct sockaddr *)&addr_client_8006, len);
          if(send_num_8006 < 0)
          {
              perror("sendto error:");
              exit(1);
          }
          printf("send out...\n");
          delete encodeImg;
          recv_flag = 0;
        }
  }
}