#ifndef OTTER_GUIDANCE_H_
#define OTTER_GUIDANCE_H_

#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

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
    void path_callback(const std_msgs::Float64MultiArray& msg);

    std::vector<geometry_msgs::PoseStamped>::iterator iterator_path;

    double m_path[5*2];

    ros::Publisher m_controllerPub;

    ros::Publisher goal_point_pub;

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
  };

} // namespace otter_coverage

#endif
