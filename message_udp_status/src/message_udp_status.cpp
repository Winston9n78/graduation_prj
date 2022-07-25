#include "message_udp_status/message_udp_status.h"

float usv_x = 0.1, usv_y = 0.12345, usv_orien = 0.123456;
int is_ok = 0;
int is_ok_from_a = 0; // 将接收到的另一艘船的解码的变量

int len;

bool recv_flag;

ros::Publisher path_pub,commander_order_pub_start,commander_order_pub_reset,commander_order_pub_guidance,commander_order_pub_latch;
std_msgs::Bool commander_order_start,commander_order_reset,commander_order_guidance,commander_order_latch;

std::string usv_status;

std::vector<uchar> buf;
std::vector<uchar> buf_origin;

cv::Mat decode_img, encode_img;

void usv_status_callback(const otter_control::usv_status msg){
    usv_x = msg.position_z;
    usv_y = msg.position_x;
    usv_orien = msg.orientation_pitch;
}

unsigned char *encodeImg;

void image_callback(const sensor_msgs::CompressedImageConstPtr& msg){

  std::vector<int> params;
  params.push_back(cv::IMWRITE_JPEG_QUALITY);
  params.push_back(50); // compression quality, ranges between 0-100

  try
  {
    if(recv_flag == false){ //与发送进程互斥
      // cv::Mat image = cv::imdecode(cv::Mat(msg->data), 1);

      buf = msg->data;
      // if (!cv::imencode(".jpg", image, buf, params)) {
      //   std::cout << "JPEG compression failed.\n";
      // }

      int nSize = buf.size();
      encodeImg = new unsigned char[nSize]; // 是否需要delete
      // encode_data = new unsigned char[buf.size()];

      for (int i = 0; i < buf.size(); i++)
      {
        encodeImg[i] = buf[i];
        // std::cout << encodeImg[i];
        /* code */
      }
      printf("img_recv..\n");
      recv_flag = 1;
      // sendto(sock_fd_8006, encodeImg, nSize, 0, (struct sockaddr *)&addr_client_8006, len);

      // decode_img = cv::imdecode(buf, CV_LOAD_IMAGE_COLOR);
      // printf("%d\n",decode_img.size());
    }
  }
  catch(const cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert to image!");
  }
  
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "messsage_udp_latch_node");

  ros::NodeHandle nh;

  ros::Publisher is_ok_pub;

  ros::Subscriber status_sub = nh.subscribe("usv_status", 1, &usv_status_callback);
  printf("111\n");
  ros::Subscriber image_sub = nh.subscribe("/d400/color/image_raw/compressed", 1, &image_callback);
  printf("222\n");
  path_pub =
    nh.advertise<std_msgs::Float32MultiArray>("/map_path", 10);

  commander_order_pub_start =
    nh.advertise<std_msgs::Bool>("/commander_order_start", 10);

  commander_order_pub_reset =
    nh.advertise<std_msgs::Bool>("/commander_order_reset", 10);

  commander_order_pub_guidance =
    nh.advertise<std_msgs::Bool>("/commander_order_guidance", 10);

  commander_order_pub_latch =
    nh.advertise<std_msgs::Bool>("/commander_order_latch", 10);

  sock_fd_init();
  serv_addr_init_all();
  client_addr_init_all();

  if(sock_fd_check())
  {
    perror("socket");
    exit(1);
  }

  /* 绑定socket */
  if(bind_check())
  {
    perror("bind error:");
    exit(1);
  }

  len = sizeof(addr_serv_8001);

  // thread_on();
  std::thread recv_msg_8001(recieve_thread_function_8001); //测试通信
  // std::thread send_msg_8001(send_thread_function_8001);

  std::thread recv_msg_8002(recieve_thread_function_8002); // 发送本机程序状态
  std::thread send_msg_8002(send_thread_function_8002);

  std::thread recv_msg_8003(recieve_thread_function_8003); // 接收open和close
  // std::thread send_msg_8003(send_thread_function_8003);

  std::thread recv_msg_8004(recieve_thread_function_8004); // 
  std::thread send_msg_8004(send_thread_function_8004); // 发送本船状态

  // std::thread recv_msg_8005(recieve_thread_function_8005); // 同上，被动船用
  // std::thread send_msg_8005(send_thread_function_8005);

  // std::thread recv_msg_8006(recieve_thread_function_8006); // 主动船摄像头图像发送用
  std::thread send_msg_8006(send_thread_function_8006);

  // std::thread recv_msg_8007(recieve_thread_function_8007); // 被动船发送摄像头
  // std::thread send_msg_8007(send_thread_function_8007);

  std::thread recv_msg_8008(recieve_thread_function_8008);  // 主动船接收航点
  // std::thread send_msg_8008(send_thread_function_8008);

  // std::thread recv_msg_8009(recieve_thread_function_8009); //多出来
  // std::thread send_msg_8009(send_thread_function_8009);
  // cv::startWindowThread();
  cv::namedWindow("view");

  double frequency = 10.0;
  ros::Rate rate(frequency);

  while(nh.ok())
  {
    path_pub.publish(map_path);
    commander_order_pub_start.publish(commander_order_start);
    commander_order_pub_reset.publish(commander_order_reset);
    commander_order_pub_guidance.publish(commander_order_guidance);
    commander_order_pub_latch.publish(commander_order_latch);
    ros::spinOnce();
    rate.sleep();
  }

  fd_close_all();

  return 0;
}


