#include <otter_control/controller.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <eigen3/Eigen/QR>

#include <cmath>

// #include <nlink_parser/LinktrackAnchorframe0.h>
#include <nlink_parser/LinktrackTagframe0.h>

#include <apriltags2_ros/AprilTagDetectionArray.h>

#include <sensor_msgs/Imu.h>

#include <otter_control/usv_status.h>

#include <queue>

OtterController::OtterController() : T(3, 2)
{
  ros::NodeHandle nh;

  m_leftPub = nh.advertise<std_msgs::Float32>("left_thrust_cmd", 10);
  m_rightPub = nh.advertise<std_msgs::Float32>("right_thrust_cmd", 10);

  m_headPub = nh.advertise<std_msgs::Float32>("head_thrust_cmd", 10);
  m_tailPub = nh.advertise<std_msgs::Float32>("tail_thrust_cmd", 10);

  ok_to_latch = nh.advertise<std_msgs::Bool>("is_ok_from_b", 1);

  usv_status_pub = nh.advertise<otter_control::usv_status>("usv_status",1);

  uwb_gps_pub = nh.advertise<nav_msgs::Odometry>("gps", 100);

  heading_angle_pub = nh.advertise<std_msgs::Float32>("heading_angle",1);
  ros::Subscriber sub = nh.subscribe("speed_heading", 1000, &OtterController::inputCallback, this); //获得速度和期望航向角
  ros::Subscriber sub_imu = nh.subscribe("imu_data", 1000, &OtterController::imu_Callback, this); //获得imu数据作为控制
  // ros::Subscriber sub_voltage = nh.subscribe("voltage", 1000, &OtterController::voltage_Callback, this);
  // 3400 14v
  ros::Subscriber sub_ariltag = nh.subscribe("/tag_detections", 1, &OtterController::apriltag_Callback, this);

  ros::Subscriber goal_Sub = nh.subscribe("goal_point", 1000, &OtterController::setPoint, this); //从这个话题中得到一个坐标，然后回调，存到标准类型的向量里，然后使用。
  
  ros::Subscriber subImu = nh.subscribe("is_lock_ok", 1, &OtterController::lock_statusCB, this);

  ros::Subscriber sub_nlink =
      nh.subscribe("/nlink_linktrack_tagframe0", 1000, &OtterController::tagframe0Callback, this);//this的作用是什么

  ros::Subscriber sub_odom_t265 = 
      nh.subscribe("/camera/odom/sample", 10, &OtterController::t265_odom_Callback, this);

  ros::Subscriber sub_dvl_a50 = 
      nh.subscribe("dvl_a50", 10, &OtterController::dvl_a50_Callback, this);

  ros::Subscriber sub_reverse_flag = 
      nh.subscribe("reverse_flag", 1, &OtterController::reverse_flag_Callback, this);
  // // Initialize thruster configuration matrix  初始化推进器控制的矩阵
  // T << 50, 50, 0, 0, -0.39 * 50, 0.39 * 50;
  tf::TransformListener listener;
  

  //启动动态参数服务器节点
  dynamic_reconfigure::Server<parameter_server::drConfig> server;
  // dynamic_reconfigure::Server<parameter_set::drConfig>::CallbackType cbType;
  // cbType = boost::bind(&OtterController::cb,_1,_2);
  // server.setCallback(cbType);
  ros::Timer timer = nh.createTimer(ros::Duration(0.1), &OtterController::callback, this);

  double frequency = 100.0;
  double deltaTime = 1.0 / frequency;
  ros::Rate rate(frequency);
  while (nh.ok()) {

    static int latch_statues,is_step1_done;
    double tauSurge = 0, tauYaw = 0;
    
    get_control_param();
    
    try{
      // 得到坐标odom和坐标base_link之间的关系
      // listener.waitForTransform("my_bundle", "camera", ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("my_bundle", "camera",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      // ROS_ERROR("%s",ex.what());
      ros::Duration(0).sleep();
    }

    /*
      对接程序框架(当被动船确定没有识别二维码时框架可直接用在被动船)：
      holding position
      ->看到apriltag->
      latching
      ->锁住则成功，其他则为失败,自动循环对接，直到成功->
      if failed:holding position until arrive
      else:next operation
    */

    // if(1)
    //   tauYaw = calculateYawMoment(deltaTime, heading_angle, 0);
    // else
    //   tauYaw = 0;

    //  tauYaw = - (Kp_psi * (angle_z - 70) + Kd_psi * angular_velocity_z);

    //  std::cout << "角度输出：" << tauYaw << std::endl;

    // stick_to_point();

#if 0 /*对接测试*/

    if(is_step1_done == false){
      if(!Arrive_master) tauSurge = calculateSurgeForce(deltaTime, velocity);
      else tauSurge = 0; //不仅u_d = 0(期望速度)，这里也为0
      tauYaw = calculateYawMoment(deltaTime, heading_angle, 0); //根据控制周期，航向角控制
      is_step1_done = stick_to_point(); /* 对点锁定 */
    }

    if(is_step1_done == true && (latch_statues!=1)){
      latch_statues = latching_algorithm();// 对接逻辑，被动船不用
      tauYaw = 0;
      tauSurge = 0;
    }

    if(latch_statues == 1){
      /* 对接成功的操作 */ 
    }

#else /* 对准测试 */
    latching_algorithm();
#endif

    double left_output = output_dead + tauSurge - tauYaw  - connect_pwm_orientation + connect_pwm_x - stick_to_point_pwm_x - stick_to_point_pwm_o;
    double right_output = output_dead - tauSurge - tauYaw - connect_pwm_orientation - connect_pwm_x + stick_to_point_pwm_x - stick_to_point_pwm_o;

    double head_output = output_dead - connect_pwm_y + stick_to_point_pwm_y;
    double tail_output = output_dead + connect_pwm_y - stick_to_point_pwm_y;

    if(reverse_flag){
      left_output = output_dead - tauSurge - tauYaw  - connect_pwm_orientation - connect_pwm_x - stick_to_point_pwm_x - stick_to_point_pwm_o;
      right_output = output_dead + tauSurge - tauYaw - connect_pwm_orientation + connect_pwm_x + stick_to_point_pwm_x - stick_to_point_pwm_o;
      //其他需要进行修正的位置是tag对调一下，也就是朝向
      head_output = output_dead + connect_pwm_y + stick_to_point_pwm_y;
      tail_output = output_dead - connect_pwm_y - stick_to_point_pwm_y;
    }

    thrust_ouput_limit(left_output);
    thrust_ouput_limit(right_output);
    thrust_ouput_limit(head_output);
    thrust_ouput_limit(tail_output);

    otter_control::usv_status status;
    // status.force_head = head_output;
    // status.force_left = left_output;
    // status.force_right = right_output;
    // status.force_tail = tail_output;
    status.orientation_pitch = orientation_error;//pitch
    status.orientation_roll = roll;
    status.orientation_yaw = yaw;
    status.position_x = camera_y;
    status.position_y = camera_z;
    status.position_z = camera_x;

    std_msgs::Float32 left;
    left.data = static_cast<float>(left_output);
    std_msgs::Float32 right;
    right.data = static_cast<float>(right_output);
    std_msgs::Float32 head;
    head.data = static_cast<float>(head_output);
    std_msgs::Float32 tail;
    tail.data = static_cast<float>(tail_output);
    std_msgs::Bool is_ok_;
    is_ok_.data = static_cast<bool>(is_ok);

    m_leftPub.publish(left);
    m_rightPub.publish(right);
    m_headPub.publish(head);
    m_tailPub.publish(tail);
    usv_status_pub.publish(status);
    ok_to_latch.publish(is_ok_);


    // 方便调试直接在这里输出信息了
    // ROS_INFO_STREAM("batterty_voltage: " << voltage);
    std::cout << "batterty_voltage: " << voltage << std::endl;
    // ROS_INFO_STREAM("heading_angle_current: " << heading_angle);
    std::cout << "heading_angle_current: " << heading_angle << std::endl;
    // ROS_INFO_STREAM("heading_angle_expected: " << psi_d);
    std::cout << "heading_angle_expected: " << psi_d << std::endl;

    // ROS_INFO_STREAM("velocity_current: " << velocity); // 目前直接给,在.h文件中，应该直接给出来
    std::cout << "velocity_current: " << velocity << std::endl;
    // ROS_INFO_STREAM("velocity_expected: " << u_d); // 发布的u， velocity
    std::cout << "velocity_expected: " << u_d << std::endl;

    if(!flag_missed_target) //ROS_INFO_STREAM("conectting...........");
    std::cout << "conectting..........." << std::endl;
    else //ROS_INFO_STREAM("no Apriltag detected...");
    std::cout << "no Apriltag detected..." << std::endl;

    // ROS_INFO_STREAM("left_output: " << left_output); 
    std::cout <<"left_output: " << left_output << std::endl;
    // ROS_INFO_STREAM("right_output: " << right_output);
    std::cout <<"right_output: " << 3000 - right_output << std::endl;

    // ROS_INFO_STREAM("head_output: " << head_output); 
    std::cout << "head_output: " << head_output << std::endl;
    // ROS_INFO_STREAM("tail_output: " << tail_output);std::fixed << std::setprecision(2) <<
    std::cout << "tail_output: " << 3000 - tail_output << std::endl;
    // std::cout << "x: " << x_error_connect << std::endl;
    printf("x:%.2f\n",x_error_connect);
    printf("y:%.2f\n",y_error_connect);
    // std::cout << "y: " << y_error_connect << std::endl;
    // std::cout << "y: " << stick_to_point_pwm_x << std::endl;
    //std::cout << "do: " << orientation_error << std::endl;
    //ROS_INFO_STREAM("--------------------------INFO-------------------------------");
    std::cout << "--------------------------INFO-------------------------------" << std::endl;
    ros::spinOnce();
    rate.sleep();
  }
}

void OtterController::setPoint(const geometry_msgs::PoseStamped& point) { 
  // m_path = path; 
  Point_set = point;
  // std::cout << "订阅的目标点坐标(" << Point_goal.pose.position.x << ", " << Point_goal.pose.position.y << ")" << std::endl;
} 

/*
return 0：对接中
return 1：对接成功
*/
int OtterController::latching_algorithm(){

  static bool prepared_flag = 0, done_flag = 0, back_flag = 0;
  const double x_offset = 1.36, x_offset_back = 1.8;
  const double y_offset = 0.015;
  const double o_offset = 2.9;

  connect_pwm_y = minimize(y_error_connect - y_offset, kp_con_y, kd_con_y, d_y);
  connect_pwm_orientation = minimize(orientation_error - o_offset, kp_con_orient, kd_con_orient, angular_velocity_z);
  if(!flag_missed_target){ //如果扫描到了tag就开始，否则就按照LOS继续跑就行
/**********************************MIT的控制思路*************************************************/
    // if(prepared_flag){
    //   // std::cout << "准备就绪" << std::endl;
    //   // 对接测试的时候这个条件应该是对接点在往前一点的距离，肯定不是刚刚好那个点作为条件
    //   if((x_error_connect - 1.0) > 0){ //二维码和摄像头对接距离为1m，如果是只用二维码测试的话可以小点

    //     if(y_error_connect < 0.05 && y_error_connect > -0.05 && orientation_error < 5 && orientation_error > -5){ //或者需要航向偏差小于一定值
    //       connect_pwm_x = minimize(x_error_connect - 1.0, kp_con_x, kd_con_x, d_x);//测试
    //     }
    //     else{ 
    //       //横向偏差太大不能前进
    //       connect_pwm_x = 0;
    //     }
    //   }
    //   else{ //向前过头
    //     prepared_flag = 0;
    //   }
    //   //if(x,y,fi)都小于，则is_ok = 1
    // }

    // else{
    //   connect_pwm_x = minimize(x_error_connect - 1.0, kp_con_x, kd_con_x, d_x); // 在原来基础上退后1m重新对接
    //   if((x_error_connect - 2.0) > 0){
    //     prepared_flag = 1;
    //   }
    // }
    // if((y_error_connect - 0.13) < 0.1 && (y_error_connect - 0.13) > -0.1 && orientation_error < 5 && orientation_error > -5)
    //   connect_pwm_x = minimize(x_error_connect - 1.3, kp_con_x, kd_con_x, d_x);//测试
    // else //扰动时退回来一点点
    //   connect_pwm_x = 0;//minimize(x_error_connect - 1.32 - 0.15, kp_con_x, kd_con_x, d_x);
    // // b船运行：ottercontrol message_udp  a船运行：roscore rosserial_python latch_control message_udp
    // // b船先运行起来，再运行a船
    // if((x_error_connect - 1.3) < 0.05 && (x_error_connect - 0.05) > -0.1 
    //   && orientation_error < 5 && orientation_error > -5
    //   &&(y_error_connect - 0.13) < 0.05 && (y_error_connect - 0.13) > -0.05) is_ok = 1;//锁开始闭合
/*****************************本船对接******************************************************/
    
    if(!done_flag){
      if(!back_flag){
        if((y_error_connect - y_offset) < 0.1 && (y_error_connect - y_offset) > -0.1 
            && (orientation_error - o_offset) < 5 && (orientation_error - o_offset) > -5)
          connect_pwm_x = minimize(x_error_connect - x_offset, kp_con_x, kd_con_x, d_x);//测试
        else //扰动时退回来一点点
          connect_pwm_x = 0;//minimize(x_error_connect - 1.32 - 0.15, kp_con_x, kd_con_x, d_x);
        // b船运行：ottercontrol message_udp  a船运行：roscore rosserial_python latch_control message_udp
        // b船先运行起来，再运行a船
        if((x_error_connect - x_offset) < 0.05 && (x_error_connect - x_offset) > -0.05 
          && (orientation_error - o_offset) < 5 && (orientation_error - o_offset) > -5
          &&(y_error_connect - y_offset) < 0.05 && (y_error_connect - y_offset) > -0.05) is_ok = 1;//锁开始闭合
      }

      if(is_ok && !is_lock_ok){
        start = 1;
        if(count > 80){
          start = 0;
          count = 0;
          is_lock_ok = 1;
        }
      }

      if(is_lock_ok){
        connect_pwm_x = minimize(x_error_connect - x_offset_back, kp_con_x, kd_con_x, d_x);
        back_flag = 1; /*正在后退*/
        start = 1; /*开始计时*/
      }

      if(is_lock_ok && back_flag && count > 30){ //大于3秒才开始判断，判断的同时一直在后退
        if((x_error_connect - x_offset) < 0.08 && (x_error_connect - x_offset) > -0.08){ //3秒距离拉开不超过这个点
          done_flag = 1;/*退出对接程序*/
        }
        /*对接失败*/
        else if((x_error_connect - x_offset_back) < 0.08 && (x_error_connect - x_offset_back) > -0.08){ 
          back_flag = 0; /*后退标志位置0*/
          is_ok = 0; /*锁打开*/
          start = 0; /*关闭计时*/
          count = 0; /*清空计数*/
          is_lock_ok = 0;
        }
        
      }
    }
    std::cout << "back_flag: " << back_flag << std::endl;
    std::cout << "count: " << count << std::endl;
    std::cout << "done_flag: " << done_flag << std::endl;
    std::cout << "x_error_connect - x_offset_back: " << x_error_connect - x_offset_back << std::endl;
     
    if(done_flag){
      connect_pwm_y = 0;
      connect_pwm_orientation = 0;
      connect_pwm_x = 0;
    }

/*******************************************************************************/

  }
  else{
    connect_pwm_x = 0;
    connect_pwm_orientation = 0;
    connect_pwm_y = 0;
  }

  if(done_flag) return 1;
  else return 0;
  // 由锁的状态返回对接成功状态
  /*
  if(conneected()) return 1;
  else return 0;
  */
}

// 计算对接偏差输出
void OtterController::apriltag_Callback(const apriltags2_ros::AprilTagDetectionArray& msg){
  
  static double x_error_last, y_error_last, o_error_last, camera_fi_last;

  const int filter_size = 10;
  double sum_x = 0, sum_y = 0, sum_o = 0;
  static double move_avg_filter_x[10],move_avg_filter_y[10],move_avg_filter_o[10];
  
  static int count = 0;

  if(msg.detections.size() != 0){

    camera_x = transform.getOrigin().x();
    camera_y = transform.getOrigin().y();
    camera_z = transform.getOrigin().z();

    tf::Quaternion quat;
    (tf::Quaternion&)quat  = tf::Quaternion(transform.getRotation()[0],transform.getRotation()[1],
                                              transform.getRotation()[2],transform.getRotation()[3]);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    roll = roll/3.14159*180;
    pitch = pitch/3.14159*180;
    yaw = yaw/3.14159*180;

    camera_pitch = -pitch;

    y_error_connect = camera_y; // 0 左右偏差 // error变量中的x和y是和对接示意图对应的
    x_error_connect = camera_z; // 0.5 距离
    camera_fi = atan2(y_error_connect, x_error_connect) / 3.14159 * 180;
    orientation_error = camera_fi - camera_pitch; // 旋转角偏差

    for(int i = filter_size - 1; i >= 1; i--){
      move_avg_filter_x[i] = move_avg_filter_x[i-1];
      move_avg_filter_y[i] = move_avg_filter_y[i-1];
      move_avg_filter_o[i] = move_avg_filter_o[i-1];
    }

    move_avg_filter_x[0] = x_error_connect;
    move_avg_filter_y[0] = y_error_connect;
    move_avg_filter_o[0] = orientation_error;

    for(int i = 0; i < filter_size;  i++){
      sum_x += move_avg_filter_x[i];
      sum_y += move_avg_filter_y[i];
      sum_o += move_avg_filter_o[i];
    }

    x_error_connect = sum_x/filter_size;
    y_error_connect = sum_y/filter_size;
    orientation_error = sum_o/filter_size;

    d_y = y_error_connect - y_error_last;
    d_x = x_error_connect - x_error_last;
    d_o = orientation_error - o_error_last;
    // d_fi = camera_fi - camera_fi_last;

    x_error_last = x_error_connect;
    y_error_last = y_error_connect;
    o_error_last = orientation_error;
    // camera_fi_last = camera_fi;

    flag_missed_target = false;
  }
  else{
    /*丢失目标，所有控制量清零*/
    y_error_connect = 0;
    x_error_connect = 0;
    orientation_error = 0;
    x_error_last = 0;
    y_error_last = 0;
    o_error_last = 0;
    flag_missed_target = true;
    d_y = 0;
    d_x = 0;
    d_o = 0;
    count = 0;
  }

  // std::cout << "connect_pwm_x = " << connect_pwm_x << std::endl;
  // std::cout << "connect_pwm_z = " << connect_pwm_z << std::endl;

}

double OtterController::minimize(double error, double kp, double ki, double integral){

    // integral > 10 ? 10 : integral; 

    if (flag_missed_target)
    {
      return 0;
    }
    //不到目标是偏差等于0，不输出
    return -kp * error - ki * integral;
}

void OtterController::keyboard_Callback(const std_msgs::Int32MultiArray& msg){

  keyboard_move = output_dead - msg.data[3];//左右平移 需要加偏置
  keyboard_val_speed = output_dead - msg.data[2] + 10;//前后运动
  keyboard_val_turning = output_dead - msg.data[0];//航向转动
  keyboard_start = msg.data[9];//遥控模式开关

  if(keyboard_start == 1065) keyboard_start = true;
  else keyboard_start = false;
  
  // std::cout << "keyboard_recieve: "<< keyboard_val_speed << std::endl;
  // std::cout << "keyboard_recieve: "<< keyboard_val_turning << std::endl;

}

void OtterController::t265_odom_Callback(const nav_msgs::Odometry &msg){
  point_now_x_t265 = msg.pose.pose.position.x;
  point_now_y_t265 = msg.pose.pose.position.y;
}

void OtterController::dvl_a50_Callback(const std_msgs::Float64MultiArray& msg){

  point_now_x_dvl_a50 = msg.data[0];
  point_now_y_dvl_a50 = msg.data[1];

  point_now_x_dvl_a50 = int(1000*point_now_x_dvl_a50);
  point_now_y_dvl_a50 = int(1000*point_now_y_dvl_a50);
  point_now_x_dvl_a50 = point_now_x_dvl_a50 / 1000;
  point_now_y_dvl_a50 = point_now_y_dvl_a50 / 1000;

  v_dvl_a50_x = msg.data[2];
  v_dvl_a50_y = msg.data[3];

}

/*
return 0: 未开始holding或者正在holding
return 1: holding过程中检测到了二维码则立马进入对接状态
*/
int OtterController::stick_to_point(){

  static double x_error_last, y_error_last, d_hold_x, d_hold_y;
  static bool latch_flag;
  static double angle_hold, position_hold_x, position_hold_y;
  static int count, done;
  double radius = 0.5; // holding半径
  //设定点应该自动计算
  double x_error_dist = point_now_x - Point_set.pose.position.x;
  double y_error_dist = point_now_y - Point_set.pose.position.y;
  x_error_stick = point_now_x_dvl_a50 - position_hold_x;
  y_error_stick = point_now_y_dvl_a50 - position_hold_y;
  angle_error_stick = angle_z - angle_hold;

  /*角度偏差映射，防止临界值小误差大转动*/
  if(angle_error_stick > 180 && angle_error_stick < 360) angle_error_stick -= 360;
  if(angle_error_stick < -180 && angle_error_stick > -360) angle_error_stick += 360;

  d_hold_x = v_dvl_a50_x;//x_error_stick - x_error_last;
  d_hold_y = v_dvl_a50_y;//y_error_stick - y_error_last;

  x_error_last = x_error_stick;
  y_error_last = y_error_stick;
  
  stick_to_point_pwm_x = (kp_stick_x * x_error_stick + kd_stick_x * d_hold_x);
  stick_to_point_pwm_y = (kp_stick_y * y_error_stick + kd_stick_y * d_hold_y);
  stick_to_point_pwm_o =  - (kp_stick_o * angle_error_stick + kd_stick_o * angular_velocity_z);

  double dist = std::sqrt(std::pow(x_error_dist, 2) + std::pow(y_error_dist, 2));

  std::cout << angle_error_stick << std::endl;
  // std::cout << angle_hold << std::endl;
  // std::cout << x_error_stick << std::endl;
  // std::cout << y_error_stick << std::endl;
  // std::cout << kp_stick_x << std::endl;
  // std::cout << kp_stick_x * x_error_stick + kd_stick_x * d_hold_x << std::endl;
  // std::cout << stick_to_point_pwm_x << std::endl;

  // 退回终点的时候用,主动船用
  // if(Arrive_master && flag_missed_target){
  //   stick_to_point_pwm_x = (kp_stick_x * x_error_stick + kd_stick_x * d_hold_x);
  //   stick_to_point_pwm_y = (kp_stick_y * y_error_stick + kd_stick_y * d_hold_y);
  //   latch_flag = false;
  // }
  if(!done) count++;
  if(!Arrive_master && count > 100){ //if(dist < radius)
    Arrive_master = true;
    angle_hold = angle_z;
    position_hold_x = point_now_x_dvl_a50;
    position_hold_y = point_now_y_dvl_a50;
    done = 1;
    count = 0;
  }
  if(!Arrive_master){
    stick_to_point_pwm_x = 0;
    stick_to_point_pwm_y = 0; 
    stick_to_point_pwm_o = 0;
  }

  if(Arrive_master && !flag_missed_target || latch_flag) { //就不再进行holding了
    stick_to_point_pwm_x = 0;
    stick_to_point_pwm_y = 0;
    stick_to_point_pwm_o = 0;     
    latch_flag = true;
    return 1;
  }
  else return 0;

}

void OtterController::thrust_ouput_limit(double& output_value){

  int dead_zone = 5; //电机死区

  if(output_value >= output_dead) output_value += dead_zone; //死区处理
  if(output_value < output_dead) output_value -= dead_zone;

  output_value = output_value > output_max ? output_max : output_value;
  output_value = output_value < output_min ? output_min : output_value;

}

void OtterController::tagframe0Callback(const nlink_parser::LinktrackTagframe0 &msg){

  static uint8_t i,j;
  double tmp[10] = {0};
  double tmp_v[10] = {0};
  static double sum, sum_v;
  static double tag0_x,tag0_y,tag0_z,tag1_x,tag1_y,tag1_z;

  static int count, done;
  const int filter_size = 10;
  double sum_x0 = 0, sum_y0 = 0, sum_x1 = 0,sum_y1 = 0;
  static double move_avg_filter_x0[10],move_avg_filter_y0[10],move_avg_filter_x1[10],move_avg_filter_y1[10];
  static double tag0_x_last,tag0_y_last,tag1_x_last,tag1_y_last;

  if(msg.id == 0){
    tag0_y = msg.pos_3d[1];
    tag0_x = msg.pos_3d[0];
    tag0_z = msg.pos_3d[2];
  }
  else{
    tag1_y = msg.pos_3d[1];
    tag1_x = msg.pos_3d[0];
    tag1_z = msg.pos_3d[2];
  }
  if(count > 50){
    // tag0_x = abnomal_detect(tag0_x, tag0_x_last);
    // tag0_y = abnomal_detect(tag0_y, tag0_y_last);
    // tag1_x = abnomal_detect(tag1_x, tag1_x_last);
    // tag1_y = abnomal_detect(tag1_y, tag1_y_last);
    done = 1;
  }
  if(!done) count++;

  for(int i = filter_size - 1; i >= 1; i--){
    move_avg_filter_x0[i] = move_avg_filter_x0[i-1];
    move_avg_filter_y0[i] = move_avg_filter_y0[i-1];
    move_avg_filter_x1[i] = move_avg_filter_x1[i-1];
    move_avg_filter_y1[i] = move_avg_filter_y1[i-1];
  }

  move_avg_filter_x0[0] = tag0_x;
  move_avg_filter_y0[0] = tag0_y;
  move_avg_filter_x1[0] = tag1_x;
  move_avg_filter_y1[0] = tag1_y;

  for(int i = 0; i < filter_size;  i++){
    sum_x0 += move_avg_filter_x0[i];
    sum_y0 += move_avg_filter_y0[i];
    sum_x1 += move_avg_filter_x1[i];
    sum_y1 += move_avg_filter_y1[i];
  }

  tag0_x = sum_x0/filter_size;
  tag0_y = sum_y0/filter_size;
  tag1_x = sum_x1/filter_size;  
  tag1_y = sum_y1/filter_size;
  // 0在前面
  double delta_y = tag0_y - tag1_y;
  double delta_x = tag0_x - tag1_x;

  if(reverse_flag){
    delta_y = - delta_y;
    delta_x = - delta_x;
  }

  point_now_x = (tag0_x + tag1_x) / 2;
  point_now_y = (tag0_y + tag1_y) / 2;

  point_now_z = tag0_z;

  tag0_x_last = tag0_x;
  tag0_y_last = tag0_y;
  tag1_x_last = tag1_x;
  tag1_y_last = tag1_y;

  heading_angle =  (90 - (atan2(delta_y, delta_x) / 3.14) * 180);
  
  std_msgs::Float32 heading_;
  heading_.data = static_cast<float>(heading_angle);
  heading_angle_pub.publish((std_msgs::Float32)heading_);
  
  float straight_line = pow((tag0_y - record_pos_y_node1),2) + pow((tag0_x-record_pos_x_node1),2);
  //sqrt(straight_line)/0.1;

  record_pos_x_node1 = tag0_x;
  record_pos_y_node1 = tag0_y;

  nav_msgs::Odometry uwb_data;

  uwb_data.header.stamp = ros::Time::now();
  uwb_data.header.frame_id = "odom";
  uwb_data.child_frame_id = "base_footprint";

  uwb_data.pose.pose.position.x = point_now_x;
  uwb_data.pose.pose.position.y = point_now_y;
  uwb_data.pose.pose.position.z = 0;

  uwb_data.pose.pose.orientation.x = 1;
  uwb_data.pose.pose.orientation.y = 0;
  uwb_data.pose.pose.orientation.z = 0;
  uwb_data.pose.pose.orientation.w = 0;

  uwb_data.pose.covariance = {0.0225*25, 0, 0, 0, 0, 0,
                              0, 0.0225*25, 0, 0, 0, 0,
                              0, 0, 0.1, 0, 0, 0,
                              0, 0, 0, 0.1, 0, 0,
                              0, 0, 0, 0, 0.1, 0,
                              0, 0, 0, 0, 0, 0.1};

  uwb_gps_pub.publish(uwb_data);

}

void OtterController::imu_Callback(const sensor_msgs::Imu& msg){

  static double v_x, position_x, v_y, position_y;

  angular_velocity_x = msg.angular_velocity.x;
  angular_velocity_y = msg.angular_velocity.y;
  angular_velocity_z = msg.angular_velocity.z;
  angle_z = msg.orientation.z;
  linear_acc_x = msg.linear_acceleration.x;
  linear_acc_y = msg.linear_acceleration.y;

  // v_x += linear_acc_x;
  // v_y += linear_acc_y;

  // position_x += v_x;
  // position_y += v_y;
  // std::cout << "速度_x:" << v_x <<std::endl;
  // std::cout << "速度_y:" << v_y <<std::endl;
  // std::cout << "位置_x:" << position_x <<std::endl;
  // std::cout << "位置_y:"<< position_y <<std::endl;
}

double OtterController::abnomal_detect(double now, double last){
  return (now - last) > 0.20 ? last : now;
}

void OtterController::voltage_Callback(const std_msgs::Float32& msg){
  voltage = msg.data;
  // std::cout <<angular_velocity_z <<std::endl;
}

double OtterController::calculateSurgeForce(double deltaTime, double u)
{
  static double integralTerm = 0.0;
  int is_shutdown;

  if(speed_shutdown_flag) is_shutdown = 0; //当前速度修改，让船停或者走
  else is_shutdown = 1;

  double u_d_dot = 0.0;
  double u_tilde = u_d - u; //期望速度是1或者0

  integralTerm += u_tilde * deltaTime;
  integralTerm > 10? 10 : integralTerm;
  //  先纯P控制
  // return mass_u * (u_d_dot - Kp_u * u_tilde - Ki_u * integralTerm) + damp_u * u; //pid控制速度
  return -is_shutdown*(u_d_dot - Kp_u * u_tilde - Ki_u * integralTerm);
  // return 3000;//开环
}

double OtterController::calculateYawMoment(double deltaTime, double psi_slam, double r)
{
  // TODO: reference model

  // ROS_INFO_STREAM("Psi: " << psi_slam);

  static double integralTerm = 0.0;
  double D;

  double output;
  static double error_last;

  double r_d_dot = 0.0;
  double r_tilde = 0.0; // r - r_d;
  double psi_tilde = psi_slam - psi_d;//psi_d;
  if (psi_tilde > 180) {
    psi_tilde -= 2 * 180;
  } else if (psi_tilde < -180) {
    psi_tilde += 2 * 180;
  }

  
  //if偏差大于一定值速度设置为0
  if(psi_tilde > 60) speed_shutdown_flag = true;
  else speed_shutdown_flag = false;
  
  D = psi_tilde - error_last;
  error_last = psi_tilde;
  // TODO: anti windup
  integralTerm += psi_tilde * deltaTime;
  integralTerm > 10 ? 10:integralTerm;
  
  output =  Kp_psi * psi_tilde + Ki_psi * integralTerm + Kd_psi * D;
  // output > 200 ? 200 : output;
  // output < -200 ? -200 : output;
  // return mass_psi * (r_d_dot - Kd_psi * r_tilde - Kp_psi * psi_tilde - Ki_psi * integralTerm) - damp_psi * r;
  return output;
}

Eigen::Vector2d OtterController::thrustAllocation(Eigen::Vector3d tau_d)
{
  // Initialize thruster configuration matrix pseudoinverse
  static bool initialized = false;
  static Eigen::MatrixXd pinv(3, 2);
  if (!initialized) {
    initialized = true;
    pinv = T.completeOrthogonalDecomposition().pseudoInverse();
  }

  // Calculate thruster output 计算推进器输出
  Eigen::Vector2d u = pinv * tau_d;

  // Ensure in interval [-1, 1] 确保输出在这个范围内
  u[0] = std::min(std::max(u[0], -1.0), 1.0);
  u[1] = std::min(std::max(u[1], -1.0), 1.0);

  return u;
}

void OtterController::inputCallback(const std_msgs::Float64MultiArray& msg)
{
  u_d = msg.data[0]; //到目的地是就会变为0
  psi_d = msg.data[1];
  // ROS_INFO_STREAM("Psi_d: " << psi_d);
}

void OtterController::speedCallback(const geometry_msgs::Vector3Stamped& msg)
{
  // GPS in ENU => rotate speed vector pi/2
  double dir = std::atan2(msg.vector.y, msg.vector.x) + M_PI_2;
  // ROS_INFO_STREAM("Speed dir: " << dir);
  if (std::abs(dir - psi) > M_PI_2 && std::abs(dir - psi) < 3 * M_PI_2) {
    u = -std::sqrt(std::pow(msg.vector.x, 2) + std::pow(msg.vector.y, 2));
  } else {
    u = std::sqrt(std::pow(msg.vector.x, 2) + std::pow(msg.vector.y, 2));
  }
}

void OtterController::imuCallback(const sensor_msgs::Imu& msg)
{
  psi = tf2::getYaw(msg.orientation);
  r = msg.angular_velocity.z;
}

double OtterController::getYaw()
{
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped tfStamped;
  try {
    tfStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0.0), ros::Duration(1.0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  return tf2::getYaw(tfStamped.transform.rotation);
}

void OtterController::lock_statusCB(const std_msgs::Bool& msg){
  is_lock_ok = msg.data;
}

void OtterController::callback(const ros::TimerEvent& event)
{
  if(start) count++;
}

void OtterController::reverse_flag_Callback(const std_msgs::Bool& msg)
{
  reverse_flag = msg.data;
}

void OtterController::get_control_param(){

  ros::param::get("/OtterController/Connect_P_y",kp_con_y);
  ros::param::get("/OtterController/Connect_D_y",kd_con_y);
  ros::param::get("/OtterController/Connect_P_x",kp_con_x);
  ros::param::get("/OtterController/Connect_D_x",kd_con_x);
  ros::param::get("/OtterController/Connect_P_orien",kp_con_orient);
  ros::param::get("/OtterController/Connect_D_orien",kd_con_orient);
  ros::param::get("/OtterController/T_P",Kp_psi);
  ros::param::get("/OtterController/T_I",Ki_psi);
  ros::param::get("/OtterController/T_D",Kd_psi);
  ros::param::get("/OtterController/V_P",Kp_u);
  ros::param::get("/OtterController/V_I",Ki_u);
  ros::param::get("/OtterController/Stick_P_x",kp_stick_x);
  ros::param::get("/OtterController/Stick_D_x",kd_stick_x);
  ros::param::get("/OtterController/Stick_P_y",kp_stick_y);
  ros::param::get("/OtterController/Stick_D_y",kd_stick_y);
  ros::param::get("/OtterController/Stick_P_o",kp_stick_o);
  ros::param::get("/OtterController/Stick_D_o",kd_stick_o);

}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "OtterController");
  OtterController otterController;
  return 0;
}
