#include <otter_control/controller.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <eigen3/Eigen/QR>

#include <cmath>

#include <nlink_parser/LinktrackAnchorframe0.h>

#include <apriltags2_ros/AprilTagDetectionArray.h>

#include <sensor_msgs/Imu.h>



OtterController::OtterController() : T(3, 2)
{
  ros::NodeHandle nh;

  m_leftPub = nh.advertise<std_msgs::Float32>("left_thrust_cmd", 10);
  m_rightPub = nh.advertise<std_msgs::Float32>("right_thrust_cmd", 10);

  m_headPub = nh.advertise<std_msgs::Float32>("head_thrust_cmd", 10);
  m_tailPub = nh.advertise<std_msgs::Float32>("tail_thrust_cmd", 10);

  ros::Subscriber sub = nh.subscribe("speed_heading", 1000, &OtterController::inputCallback, this); //获得速度和期望航向角
  ros::Subscriber sub_imu = nh.subscribe("imu", 1000, &OtterController::imu_Callback, this); //获得imu数据作为控制
  // ros::Subscriber sub_voltage = nh.subscribe("voltage", 1000, &OtterController::voltage_Callback, this);
//3400 14v
  ros::Subscriber sub_ariltag = nh.subscribe("/tag_detections", 1, &OtterController::apriltag_Callback, this);

  ros::Subscriber goal_Sub = nh.subscribe("goal_point", 1000, &OtterController::setPoint, this); //从这个话题中得到一个坐标，然后回调，存到标准类型的向量里，然后使用。
  // ros::Subscriber subSpeed = nh.subscribe("gps/vel", 1000, &OtterController::speedCallback, this); //
  // ros::Subscriber subImu = nh.subscribe("imu/data", 1000, &OtterController::imuCallback, this);

  ros::Subscriber sub_nlink =
      nh.subscribe("/nlink_linktrack_anchorframe0", 1000, &OtterController::tagframe0Callback, this);//this的作用是什么
  // // Initialize thruster configuration matrix  初始化推进器控制的矩阵
  // T << 50, 50, 0, 0, -0.39 * 50, 0.39 * 50;

  //启动动态参数服务器节点
  dynamic_reconfigure::Server<parameter_server::drConfig> server;
  // dynamic_reconfigure::Server<parameter_set::drConfig>::CallbackType cbType;
  // cbType = boost::bind(&OtterController::cb,_1,_2);
  // server.setCallback(cbType);

  double frequency = 100.0;
  double deltaTime = 1.0 / frequency;
  ros::Rate rate(frequency);
  while (nh.ok()) {

    get_control_param();
    
    auto tauSurge = calculateSurgeForce(deltaTime, velocity);

    // Heading controller 航向角控制
    // auto tauYaw = calculateYawMoment(deltaTime, psi_slam, r); //根据控制周期，航向角控制
    auto tauYaw = 0 - calculateYawMoment(deltaTime, heading_angle, 0); //根据控制周期，航向角控制

    if(tauYaw > 150) tauYaw = 150;
    if(tauYaw < -150) tauYaw = -150;

    /*
     对接程序框架：
     holding position
     ->看到apriltag->
     latching
     ->锁住则成功，其他则为失败->
     if failed:holding position until arrive
     else:next operation
    */
    static int holding_statues,latch_statues,step1_done;
    if(!step1_done)
      holding_statues = stick_to_point();// 对点锁定

    if(holding_statues == true && (latch_statues!=1)){
      step1_done = true;
      latch_statues = latching_algorithm();// 对接逻辑，被动船不用
    }

    if(latch_statues == 1){
      // 对接成功的操作
    }

    double left_output = output_dead + speed_swtich_flag * tauSurge + tauYaw - connect_pwm_y + connect_pwm_x + stick_flag * stick_to_point_pwm_y;
    double right_output = output_dead + speed_swtich_flag * tauSurge - tauYaw - connect_pwm_y + connect_pwm_x + stick_flag * stick_to_point_pwm_y;

    double head_output = output_dead - connect_pwm_orientation + stick_flag * stick_to_point_pwm_x;
    double tail_output = output_dead + connect_pwm_orientation + stick_flag * stick_to_point_pwm_x;

    thrust_ouput_limit(left_output);
    thrust_ouput_limit(right_output);
    thrust_ouput_limit(head_output);
    thrust_ouput_limit(tail_output);
  
    std_msgs::Float32 left;
    left.data = static_cast<float>(left_output);
    std_msgs::Float32 right;
    right.data = static_cast<float>(right_output);
    std_msgs::Float32 head;
    head.data = static_cast<float>(head_output);
    std_msgs::Float32 tail;
    tail.data = static_cast<float>(tail_output);

    m_leftPub.publish(left);
    m_rightPub.publish(right);
    m_headPub.publish(head);
    m_tailPub.publish(tail);

    // 方便调试直接在这里输出信息了
    ROS_INFO_STREAM("batterty_voltage: " << voltage);
    ROS_INFO_STREAM("heading_angle_current: " << heading_angle);
    ROS_INFO_STREAM("heading_angle_expected: " << psi_d);

    ROS_INFO_STREAM("velocity_current: " << velocity); // 目前直接给,在.h文件中，应该直接给出来
    ROS_INFO_STREAM("velocity_expected: " << u_d); // 发布的u， velocity

    if(!flag_missed_target) ROS_INFO_STREAM("conectting...........");
    else ROS_INFO_STREAM("no Apriltag detected...");

    ROS_INFO_STREAM("left_output: " << left_output); 
    ROS_INFO_STREAM("right_output: " << right_output);

    ROS_INFO_STREAM("head_output: " << head_output); 
    ROS_INFO_STREAM("tail_output: " << tail_output);

    ROS_INFO_STREAM("--------------------------INFO-------------------------------");

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
return 0：未成功
return 1：对接成功
*/
int OtterController::latching_algorithm(){

  static bool prepared_flag = 0;

  connect_pwm_y = minimize(y_error_connect, kp_con, ki_con, y_error_integral);
  connect_pwm_orientation = minimize(orientation_error, kp_con_orient, ki_con_orient, orientation_error_integral);
  if(!flag_missed_target){ //如果扫描到了tag就开始，否则就按照LOS继续跑就行

    speed_swtich_flag = 0; //速度为0了，航向角的控制变不变0呢？

    if(prepared_flag){
      // std::cout << "准备就绪" << std::endl;
      if(x_error_connect > 0){

        if(y_error_connect < 0.4 &&  y_error_connect > -0.4){ //或者需要航向偏差小于一定值
          connect_pwm_x = minimize(x_error_connect, kp_con, ki_con, x_error_integral);
        }
        else{ 
          //横向偏差太大不能前进
          connect_pwm_x = 0;
        }
      }
      else{ //向前过头
        prepared_flag = 0;
      }
    }

    else{
      connect_pwm_x = minimize(x_error_connect - 1, kp_con, ki_con, x_error_integral); // 改变目标距离
      if(x_error_connect > 1){
        prepared_flag = 1;
      }
    }
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

  if(msg.detections.size() != 0){

    apriltag_x = msg.detections[0].pose.pose.pose.position.x;
    apriltag_y = msg.detections[0].pose.pose.pose.position.y;
    apriltag_z = msg.detections[0].pose.pose.pose.position.z;

    apriltag_orientation_x = msg.detections[0].pose.pose.pose.orientation.z;
    // apriltag_orientation_y = msg.detections[0].pose.pose.pose.orientation.y;
    // apriltag_orientation_z = msg.detections[0].pose.pose.pose.orientation.z;
    // apriltag_orientation_w = msg.detections[0].pose.pose.pose.orientation.w;

    y_error_connect = apriltag_x; // 0 左右偏差 // error变量中的x和y是和对接示意图对应的
    x_error_connect = apriltag_z; // 0.5 距离
    orientation_error = apriltag_orientation_x; // 旋转角偏差

    y_error_integral += y_error_connect;
    x_error_integral += x_error_connect;
    orientation_error_integral += orientation_error;

    flag_missed_target = false;
  }
  else{
    y_error_integral = 0;
    x_error_integral = 0;
    orientation_error_integral = 0;
    flag_missed_target = true;
  }

  // std::cout << "connect_pwm_x = " << connect_pwm_x << std::endl;
  // std::cout << "connect_pwm_z = " << connect_pwm_z << std::endl;

}

double OtterController::minimize(double error, double kp, double ki, double integral){

    integral > 10 ? 10 : integral; 

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
return 1: holding过程中
*/
int OtterController::stick_to_point(){

  static double x_integral,y_integral;
  static bool arrive;
  double radius = 0.8; // holding半径
  //设定点应该自动计算
  x_error_stick = point_now_x - Point_set.pose.position.x;
  y_error_stick = point_now_y - Point_set.pose.position.y;

  x_integral += x_error_stick;
  y_integral += y_error_stick;

  stick_to_point_pwm_x = - kp_stick * x_error_stick - ki_stick * x_integral;
  stick_to_point_pwm_y = - kp_stick * y_error_stick - ki_stick * y_integral;

  double dist = std::sqrt(std::pow(x_error_stick, 2) + std::pow(y_error_stick, 2));

  if(dist < radius) arrive = true;

  if(arrive && dist < radius){
    speed_swtich_flag = 0;
    stick_flag = 0;
  }
  else if(arrive && dist > radius){ //离开目标点一定范围，开始通过x与y坐标偏差控制。航向角是一直开着的。
    stick_flag = 1;
    speed_swtich_flag = 0;    
  }

  if(arrive && !flag_missed_target) {stick_flag = 0;return 1;}
  else return 0;

}

void OtterController::thrust_ouput_limit(double& output_value){

  int dead_zone = 5; //电机死区

  if(output_value >= output_dead) output_value += dead_zone; //死区处理
  if(output_value < output_dead) output_value -= dead_zone;

  output_value = output_value > output_max ? output_max : output_value;
  output_value = output_value < output_min ? output_min : output_value;

}

void OtterController::tagframe0Callback(const nlink_parser::LinktrackAnchorframe0 &msg){

  static uint8_t i,j;
  double tmp[10] = {0};
  double tmp_v[10] = {0};
  static double sum, sum_v;

  double delta_y = msg.nodes[0].pos_3d[1] - msg.nodes[1].pos_3d[1];
  double delta_x = msg.nodes[0].pos_3d[0] - msg.nodes[1].pos_3d[0];

  point_now_x = msg.nodes[0].pos_3d[0];
  point_now_y = msg.nodes[0].pos_3d[1];
  point_now_z = msg.nodes[0].pos_3d[2];

  heading_angle =  (90 - (atan2(delta_y, delta_x) / 3.14) * 180);
  
  float straight_line = pow((msg.nodes[0].pos_3d[1]-record_pos_y_node1),2) + pow((msg.nodes[0].pos_3d[0]-record_pos_x_node1),2);
  //sqrt(straight_line)/0.1;

  record_pos_x_node1 = msg.nodes[0].pos_3d[0];
  record_pos_y_node1 = msg.nodes[0].pos_3d[1];

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
  float D;

  float output_turning;

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
  integralTerm > 10? 10:integralTerm;
    
  // return mass_psi * (r_d_dot - Kd_psi * r_tilde - Kp_psi * psi_tilde - Ki_psi * integralTerm) - damp_psi * r;
  return ( - Kp_psi * psi_tilde - Ki_psi * integralTerm - Kd_psi * D);
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

void OtterController::get_control_param(){

  ros::param::get("/OtterController/Connect_P",kp_con);
  ros::param::get("/OtterController/Connect_I",ki_con);
  ros::param::get("/OtterControllerr/Connect_D",kd_con);
  ros::param::get("/OtterController/Connect_P_orien",kp_con_orient);
  ros::param::get("/OtterController/Connect_I_orien",ki_con_orient);
  ros::param::get("/OtterController/T_P",Kp_psi);
  ros::param::get("/OtterController/T_I",Ki_psi);
  ros::param::get("/OtterController/T_D",Kd_psi);
  ros::param::get("/OtterController/V_P",Kp_u);
  ros::param::get("/OtterController/V_I",Ki_u);
  ros::param::get("/OtterController/Stick_P",kp_stick);
  ros::param::get("/OtterController/Stick_I",ki_stick);
  ros::param::get("/OtterController/Stick_D",kd_stick);

}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "OtterController");
  OtterController otterController;
  return 0;
}