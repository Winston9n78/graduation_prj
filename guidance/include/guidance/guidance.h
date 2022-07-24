#ifndef OTTER_GUIDANCE_H_
#define OTTER_GUIDANCE_H_

#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include "guidance/usv_pose.h"
#include <nlink_parser/LinktrackAnchorframe0.h>
#include "stof/stof.h"
#include <string.h>

namespace otter_coverage
{

  class Guidance
  {
  public:
    Guidance();
    ~Guidance();

  private:
    void newWaypoint(const geometry_msgs::PoseStamped& waypoint);
    void newPoint(const geometry_msgs::PoseStamped& point);
    void start_Point(const geometry_msgs::PoseStamped& point);
    void followPath(double x, double y, double psi, double x_start, double y_start, double x_goal, double y_goal);
    double dist(double x0, double y0, double x1, double y1) const
    {
      return std::sqrt(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2));
    }
    void set_v_callback(const std_msgs::Float32& msg);
    void path_callback(const std_msgs::Float32MultiArray& msg);

    void tagframe0Callback(const nlink_parser::LinktrackAnchorframe0 &msg);

    void path_set();

    void reset_flag_Callback(const std_msgs::Bool& msg);
    void reset();

    std::vector<geometry_msgs::PoseStamped>::iterator iterator_path;

    double m_path[point_number] = {0}; //来自rqt
    double map_path[1024] = {0}, map_path_size = 0; //来自上位机
    double u = 1; //期望速度是1，当前当前速度设置为0，调P让他动起来

    ros::Publisher m_controllerPub;

    ros::Publisher goal_point_pub;

    ros::Publisher usv_pose_pub;

    geometry_msgs::PoseStamped Point_goal, Point_start;

    // lookahead distance
    double DELTA = 0.5;

    // time-varying lookahead distance
    double delta_max = 4.0;
    double delta_min = 1.0;
    double delta_k = 1.0;

    // circle of acceptance
    double R = 1.0;

    double m_maxSpeed;
    double m_maxSpeedTurn;
    double m_minSpeed;

    double x_0;
    double y_0;
    double x_1;
    double y_1;
    double heading_angle;

    double m_path_[point_number];

    bool reset_flag;
    int path_i = 0, path_j = 0;

    guidance::usv_pose usv_pose;
  };

} // namespace otter_coverage

#endif
