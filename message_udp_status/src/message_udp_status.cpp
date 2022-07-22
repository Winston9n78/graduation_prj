#include "message_udp_status/message_udp_status.h"

float usv_x = 0.1, usv_y = 0.12345, usv_orien = 0.123456;
int is_ok = 0;
int is_ok_from_a = 0; // 将接收到的另一艘船的解码的变量

int len;

ros::Publisher path_pub;

std::string usv_status;

std_msgs::Float32MultiArray map_path;

void usv_status_callback(const otter_control::usv_status msg){
    usv_x = msg.position_z;
    usv_y = msg.position_x;
    usv_orien = msg.orientation_pitch;
}


void image_callback(const sensor_msgs::CompressedImageConstPtr& msg){

  try
  {
    cv::Mat image = cv::imdecode(cv::Mat(msg->data), 1);
    int a = sizeof(msg->data);
    int b = sizeof(msg->data[0]);
    printf("length:%d\n", a/b);
    // cv::imshow("view", image);
    // cv::waitKey(0);


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
    nh.advertise<std_msgs::Float32MultiArray>("map_path", 10);

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
  std::thread recv_msg_8001(recieve_thread_function_8001);
  std::thread send_msg_8001(send_thread_function_8001);

  // std::thread recv_msg_8002(recieve_thread_function_8002);
  // std::thread send_msg_8002(send_thread_function_8002);

  std::thread recv_msg_8003(recieve_thread_function_8003);
  std::thread send_msg_8003(send_thread_function_8003);

  std::thread recv_msg_8004(recieve_thread_function_8004);
  std::thread send_msg_8004(send_thread_function_8004);

  // std::thread recv_msg_8005(recieve_thread_function_8005);
  // std::thread send_msg_8005(send_thread_function_8005);

  // std::thread recv_msg_8006(recieve_thread_function_8006);
  // std::thread send_msg_8006(send_thread_function_8006);

  // std::thread recv_msg_8007(recieve_thread_function_8007);
  // std::thread send_msg_8007(send_thread_function_8007);

  // std::thread recv_msg_8008(recieve_thread_function_8008);
  // std::thread send_msg_8008(send_thread_function_8008);

  // std::thread recv_msg_8009(recieve_thread_function_8009);
  // std::thread send_msg_8009(send_thread_function_8009);
  // cv::namedWindow("view");
  // cv::startWindowThread();

  double frequency = 100.0;
  ros::Rate rate(frequency);

  while(nh.ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  fd_close_all();

  return 0;
}


