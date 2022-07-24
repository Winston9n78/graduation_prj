#ifndef OTTER_CONTROL_H
#define OTTER_CONTROL_H

#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
// #include <usv_msgs/SpeedCourse.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <eigen3/Eigen/Core>

// #include <nlink_parser/LinktrackAnchorframe0.h>
#include <nlink_parser/LinktrackTagframe0.h>

#include <apriltags2_ros/AprilTagDetectionArray.h>

#include <geometry_msgs/PoseStamped.h>

#include "dynamic_reconfigure/server.h"
#include "parameter_set/drConfig.h"

#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>

#include <queue>

class OtterController
{
public:
  OtterController();

private:
  
  void set_param();
  double calculateSurgeForce(double deltaTime, double u);
  double calculateYawMoment(double deltaTime, double psi, double r);
  Eigen::Vector2d thrustAllocation(Eigen::Vector3d tau_d);
  void inputCallback(const std_msgs::Float64MultiArray& msg);
  void speedCallback(const geometry_msgs::Vector3Stamped& msg);
  void imuCallback(const sensor_msgs::Imu& msg);
  double getYaw();

  void imu_Callback(const sensor_msgs::Imu& msg);
  void voltage_Callback(const std_msgs::Float32& msg);
  void tagframe0Callback(const nlink_parser::LinktrackTagframe0 &msg);
  void thrust_ouput_limit(double& output_value);

  void t265_odom_Callback(const nav_msgs::Odometry &msg);
  void dvl_a50_Callback(const std_msgs::Float64MultiArray& msg);

  void keyboard_Callback(const std_msgs::Int32MultiArray& msg);
  void apriltag_Callback(const apriltags2_ros::AprilTagDetectionArray& msg);

  void reverse_flag_Callback(const std_msgs::Bool& msg);
  void reset_flag_Callback(const std_msgs::Bool& msg);
  void reset();

  void setPoint(const geometry_msgs::PoseStamped& point);

  void lock_statusCB(const std_msgs::Bool& msg);
  void callback(const ros::TimerEvent& event);
  
  int stick_to_point();
  int latching_algorithm();

  void get_control_param();

  double abnomal_detect(double now, double last);

  void cb(parameter_server::drConfig& config, uint32_t level);
  double minimize(double error, double kp, double ki, double integral);


  

  // Heading controller
  double Kp_psi = 0;
  double Ki_psi = 0;
  double Kd_psi = 0;
  double mass_psi = 10.0 - 1.0; // Iz - nDotR
  double damp_psi = 20.0; // nR

  // Speed controller
  double Kp_u = 0;
  double Ki_u = 0;
  double mass_u = 29 - 5.0; // m - xDotU: 29 - 5
  double damp_u = 20.0; // xU

  //connected controller
  double kp_con_x = 0;
  double kd_con_x = 0;

  double kp_con_y = 0;
  double kd_con_y = 0;

  double kp_con_orient = 0;
  double kd_con_orient = 0;
  //点保持参数
  double kp_stick_x = 0, kd_stick_x = 0, kp_stick_y = 0, kd_stick_y = 0, kp_stick_o = 0, kd_stick_o = 0;
  double kd_stick = 0;

  // Sensor data
  double u = 0.0;
  double psi = 0.0;
  double r = 0.0;

  // Desired values
  double u_d = 0.0;
  double psi_d = 0.0;

  double heading_angle = 0;

  double velocity = 0; //假设当前速度，如果期望速度大于0，则会动，反之不动

  float record_pos_x_node1 = 0;
  float record_pos_y_node1 = 0;

  float angular_velocity_x = 0;
  float angular_velocity_y = 0;
  float angular_velocity_z = 0;
  float angle_x = 0;
  float angle_y = 0;
  float angle_z = 0;
  float linear_acc_x = 0;
  float linear_acc_y = 0;
  float linear_acc_z = 0;

  float output_max = 1700;
  float output_min = 1300;
  float output_dead = 1500;

  double camera_x = 0, camera_y = 0, camera_z = 0, camera_fi = 0;
  double camera_pitch = 0; //作为yaw
  double connect_pwm_y = 0, connect_pwm_x = 0, connect_pwm_orientation = 0,connect_pwm_fi = 0;

  double point_now_x = 0, point_now_y = 0, point_now_z = 0;
  double point_now_x_t265 = 0, point_now_y_t265 = 0, point_now_z_t265 = 0;
  double point_now_x_dvl_a50 = 0, point_now_y_dvl_a50 = 0, v_dvl_a50_x = 0, v_dvl_a50_y = 0;

  double point_set_x = 1.5, point_set_y = 1.5, point_set_z;
  double stick_to_point_pwm_x = 0, stick_to_point_pwm_y = 0, stick_to_point_pwm_o = 0;

  int16_t keyboard_val_speed = 0, keyboard_val_turning = 0, keyboard_start = 1, keyboard_move = 0;
  int16_t keyboard_con_mode = 0, aoto_con_mode = -1;
  bool speed_shutdown_flag = true;
  bool flag_missed_target = true;
  bool stick_flag = 0, connect_flag_x = 0, connect_flag_y = 0;

  bool speed_swtich_flag = 1, turnning_switch_flag = 1;

  bool Arrive_master = false, Arrive_slave;

  double y_error_connect = 0, x_error_connect = 0, orientation_error = 0;
  double d_y = 0,d_x = 0,d_o = 0,d_fi = 0;
  double x_error_stick = 0, y_error_stick = 0, angle_error_stick = 0;

  double target_z = 0.5;

  double roll, pitch, yaw,fi;

  double voltage = 0;

  bool is_ok = false, is_lock_ok = false;
  int start = 0, count = 0;

  bool reverse_flag = false, reset_flag = false;
  bool turn_off_guidance = false;

  //latch_control中的变量
  bool prepared_flag = 0, done_flag = 0, back_flag = 0;

  tf::StampedTransform transform; // my_bundle里的坐标变换关系

  geometry_msgs::PoseStamped Point_set;
  // Thruster configuration matrix
  Eigen::MatrixXd T;

  // Publishers for thruster commands
  ros::Publisher m_leftPub;
  ros::Publisher m_rightPub;
  ros::Publisher m_headPub;
  ros::Publisher m_tailPub;
  ros::Publisher usv_status_pub;
  ros::Publisher heading_angle_pub;
  ros::Publisher ok_to_latch;
  ros::Publisher uwb_gps_pub;
  
};

#endif
