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

  ok_to_latch = nh.advertise<std_msgs::Bool>("is_ok", 10);

  usv_status_pub = nh.advertise<otter_control::usv_status>("usv_status",1);
  heading_angle_pub = nh.advertise<std_msgs::Float32>("heading_angle",1);
  ros::Subscriber sub = nh.subscribe("speed_heading", 1000, &OtterController::inputCallback, this); //获得速度和期望航向角
  ros::Subscriber sub_imu = nh.subscribe("imu", 1000, &OtterController::imu_Callback, this); //获得imu数据作为控制
  // ros::Subscriber sub_voltage = nh.subscribe("voltage", 1000, &OtterController::voltage_Callback, this);
  //3400 14v
  ros::Subscriber sub_ariltag = nh.subscribe("/tag_detections", 1, &OtterController::apriltag_Callback, this);

  ros::Subscriber goal_Sub = nh.subscribe("goal_point", 1000, &OtterController::setPoint, this); //从这个话题中得到一个坐标，然后回调，存到标准类型的向量里，然后使用。
  
  ros::Subscriber subImu = nh.subscribe("is_lock_ok", 1, &OtterController::lock_statusCB, this);

  ros::Subscriber sub_nlink =
      nh.subscribe("/nlink_linktrack_tagframe0", 1000, &OtterController::tagframe0Callback, this);//this的作用是什么
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
        //得到坐标odom和坐标base_link之间的关系
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
#if 0 /*对接测试*/

    if(is_step1_done == false){
      if(!Arrive_master) tauSurge = calculateSurgeForce(deltaTime, velocity);
      else tauSurge = 0;
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

    double left_output = output_dead + tauSurge + tauYaw  - connect_pwm_orientation + connect_pwm_x + stick_flag * stick_to_point_pwm_y;
    double right_output = output_dead - tauSurge + tauYaw - connect_pwm_orientation - connect_pwm_x + stick_flag * stick_to_point_pwm_y;

    double head_output = output_dead - connect_pwm_y + stick_to_point_pwm_x;
    double tail_output = output_dead + connect_pwm_y + stick_to_point_pwm_x;

    thrust_ouput_limit(left_output);
    thrust_ouput_limit(right_output);
    thrust_ouput_limit(head_output);
    thrust_ouput_limit(tail_output);

    otter_control::usv_status status;
    // status.force_head = head_output;
    // status.force_left = left_output;
    // status.force_right = right_output;
    // status.force_tail = tail_output;
    status.orientation_pitch = pitch;
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
    // ROS_INFO_STREAM("tail_output: " << tail_output);
    std::cout << "tail_output: " << 3000 - tail_output << std::endl;
    std::cout << "dx: " << kd_con_x*d_x << std::endl;
    std::cout << "dy: " << kd_con_y*d_y << std::endl;
    std::cout << "do: " << kd_con_orient*d_o << std::endl;
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

  connect_pwm_y = minimize(y_error_connect, kp_con_y, kd_con_y, d_y);
  connect_pwm_orientation = minimize(orientation_error, kp_con_orient, kd_con_orient, d_o);
  if(!flag_missed_target){ //如果扫描到了tag就开始，否则就按照LOS继续跑就行
/***********************************************************************************/
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
    connect_pwm_x = minimize(x_error_connect - 1.0, kp_con_x, kd_con_x, d_x);//测试

/*******************************对准测试用****************************************************/
    // if(!done_flag){
    //   if(!back_flag){
    //     connect_pwm_x = minimize(x_error_connect - 1.0, kp_con_x, kd_con_x, d_x);
    //     if((x_error_connect - 1.0) < 0.05 && (x_error_connect - 1.0) > -0.05) is_ok = 1;//锁开始闭合
    //   }
    //   if(is_lock_ok){
    //     connect_pwm_x = minimize(x_error_connect - 1.5, kp_con_x, kd_con_x, d_x);
    //     back_flag = 1; /*正在后退*/
    //     start = 1; /*开始计时*/
    //   }

    //   if(is_lock_ok && back_flag && count > 30){
    //     if(x_error_connect - 1.2 < 0){
    //       done_flag = 1;/*退出对接程序*/
    //     }
    //     /*对接失败*/
    //     else if((x_error_connect - 1.5) < 0.05 && (x_error_connect - 1.5) > -0.05){ 
    //       back_flag = 0; /*后退标志位置0*/
    //       is_ok = 0; /*锁打开*/
    //       start = 0; /*关闭计时*/
    //       count = 0; /*清空计数*/
    //     }
    //   }
    // }

    // else{
    //   connect_pwm_y = 0;
    //   connect_pwm_orientation = 0;
    //   connect_pwm_x = 0;
    // }

/*******************************对接*************************************************/

  }
  else{
    connect_pwm_x = 0;
    connect_pwm_orientation = 0;
    connect_pwm_y = 0;
  }

  if(0) return 1;
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

  std::queue<double> moving_avg_x, tmp_x, moving_avg_y, tmp_y, moving_avg_o, tmp_o;
  
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

    /************暴力滑动滤波器***********************************/
    if(count < 10){
      moving_avg_x.push(x_error_connect);
      moving_avg_y.push(y_error_connect);
      moving_avg_o.push(orientation_error);
      count++;
    }
    else{
      moving_avg_x.pop();
      moving_avg_x.push(x_error_connect);
      moving_avg_y.pop();
      moving_avg_y.push(y_error_connect);
      moving_avg_o.pop();
      moving_avg_o.push(orientation_error);
    }

    for(int i = 0; i < moving_avg_x.size(); i++){
      x_error_connect += moving_avg_x.front();
      tmp_x.push(moving_avg_x.front());
      moving_avg_x.pop();

      y_error_connect += moving_avg_y.front();
      tmp_y.push(moving_avg_y.front());
      moving_avg_y.pop();

      tmp_y.push(moving_avg_o.front());
      orientation_error += moving_avg_o.front();
      moving_avg_o.pop();
    }

    moving_avg_x = tmp_x;
    moving_avg_y = tmp_y;
    moving_avg_o = tmp_o;

    x_error_connect = x_error_connect/moving_avg_x.size();
    y_error_connect = y_error_connect/moving_avg_y.size();
    orientation_error = orientation_error/moving_avg_o.size();

    /************暴力滑动滤波器***********************************/

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

/*
return 0: 未开始holding或者正在holding
return 1: holding过程中检测到了二维码则立马进入对接状态
*/
int OtterController::stick_to_point(){

  static double x_integral,y_integral;
  double radius = 0.8; // holding半径
  //设定点应该自动计算
  x_error_stick = point_now_x - Point_set.pose.position.x;
  y_error_stick = point_now_y - Point_set.pose.position.y;

  // 可以用minimize来算
  x_integral += x_error_stick;
  y_integral += y_error_stick;
  y_integral > 20 ? 20 : y_integral;
  x_integral > 20 ? 20 : x_integral;

  stick_to_point_pwm_x = - kp_stick * x_error_stick - ki_stick * x_integral;
  stick_to_point_pwm_y = - kp_stick * y_error_stick - ki_stick * y_integral;

  double dist = std::sqrt(std::pow(x_error_stick, 2) + std::pow(y_error_stick, 2));

  if(dist < radius) Arrive_master = true;

  if(Arrive_master && dist < radius){
    stick_to_point_pwm_x = 0;
    stick_to_point_pwm_y = 0;
  }
  else if(Arrive_master && dist > radius){ //离开目标点一定范围，开始通过x与y坐标偏差控制。航向角是一直开着的。
   
  }

  if(Arrive_master && !flag_missed_target) {stick_flag = 0;return 1;}
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
  // 0在前面
  double delta_y = tag0_y - tag1_y;
  double delta_x = tag0_x - tag1_x;

  point_now_x = tag0_x;
  point_now_y = tag0_y;
  point_now_z = tag0_z;

  heading_angle =  (90 - (atan2(delta_y, delta_x) / 3.14) * 180);
  
  std_msgs::Float32 heading_;
  heading_.data = static_cast<float>(heading_angle);
  heading_angle_pub.publish((std_msgs::Float32)heading_);
  
  float straight_line = pow((tag0_y-record_pos_y_node1),2) + pow((tag0_x-record_pos_x_node1),2);
  //sqrt(straight_line)/0.1;

  record_pos_x_node1 = tag0_x;
  record_pos_y_node1 = tag0_y;

}

void OtterController::imu_Callback(const sensor_msgs::Imu& msg){

  angular_velocity_x = msg.angular_velocity.x;
  angular_velocity_y = msg.angular_velocity.y;
  angular_velocity_z = msg.angular_velocity.z;
  // std::cout <<angular_velocity_z <<std::endl;

}

void OtterController::voltage_Callback(const std_msgs::Float32& msg){

  voltage = msg.data;
  // std::cout <<angular_velocity_z <<std::endl;

}

double OtterController::calculateSurgeForce(double deltaTime, double u)
{
  static double integralTerm = 0.0;

  if(speed_shutdown_flag) u_d = 2;
  else u_d = 1;

  double u_d_dot = 0.0;
  double u_tilde = u - u_d;

  integralTerm += u_tilde * deltaTime;
  integralTerm > 10? 10 : integralTerm;
  //  先纯P控制
  // return mass_u * (u_d_dot - Kp_u * u_tilde - Ki_u * integralTerm) + damp_u * u; //pid控制速度
  return -(u_d_dot - Kp_u * u_tilde - Ki_u * integralTerm);
  // return 3000;//开环
}

double OtterController::calculateYawMoment(double deltaTime, double psi_slam, double r)
{
  // TODO: reference model

  // ROS_INFO_STREAM("Psi: " << psi_slam);

  static double integralTerm = 0.0;
  double D;

  double output;

  double r_d_dot = 0.0;
  double r_tilde = 0.0; // r - r_d;
  double psi_tilde = psi_slam - psi_d;
  if (psi_tilde > 180) {
    psi_tilde -= 2 * 180;
  } else if (psi_tilde < -180) {
    psi_tilde += 2 * 180;
  }

  //if偏差大于一定值速度设置为0
  if(psi_tilde > 60) speed_shutdown_flag = true;
  else speed_shutdown_flag = false;
  
  D = angular_velocity_z;
  
  // TODO: anti windup
  integralTerm += psi_tilde * deltaTime;
  integralTerm > 10 ? 10:integralTerm;
  
  output =  Kp_psi * psi_tilde + Ki_psi * integralTerm + Kd_psi * D;
  output > 200 ? 200 : output;
  output < -200 ? -200 : output;
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

void OtterController::inputCallback(const usv_msgs::SpeedCourse& msg)
{
  u_d = msg.speed;
  psi_d = msg.course;
  
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
  ros::param::get("/OtterController/Stick_P",kp_stick);
  ros::param::get("/OtterController/Stick_D",kd_stick);

}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "OtterController");
  OtterController otterController;
  return 0;
}
