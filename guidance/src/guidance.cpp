#include <guidance/guidance.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <std_msgs/Float32.h>

#include <iostream>
#include <cmath>

#include <usv_msgs/SpeedCourse.h>

namespace otter_coverage
{

Guidance::Guidance()
{
  ros::NodeHandle nh;
  ros::NodeHandle nhP("~");

  m_maxSpeed = nhP.param("max_speed", 1.5);
  m_maxSpeedTurn = nhP.param("max_speed_turn", 0.6);
  m_minSpeed = nhP.param("min_speed", 0.4);

  //订阅新路径（目标点和起始点）
  ros::Subscriber start_Sub =
      nh.subscribe("start_point", 1000, &Guidance::start_Point, this); //从这个话题中得到一个坐标，然后回调，存到标准类型的向量里，然后使用。

  ros::Subscriber velocity_sub = 
      nh.subscribe("velocity_set", 1000, &Guidance::set_v_callback, this);

  ros::Subscriber path_sub = 
      nh.subscribe("path_set", 100, &Guidance::path_callback, this);      
  //发布速度
  m_controllerPub =
      nh.advertise<usv_msgs::SpeedCourse>("speed_heading", 1000);

  goal_point_pub =
      nh.advertise<geometry_msgs::PoseStamped>("goal_point", 1000);

  usv_pose_pub = 
      nh.advertise<guidance::usv_pose>("usv/pose",1000); 

  //订阅角度
  ros::Subscriber sub =
      nh.subscribe("/nlink_linktrack_anchorframe0", 1000, &Guidance::tagframe0Callback, this);

  // tf2_ros::Buffer tfBuffer;
  // tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (nh.ok())
  {
    
    std::string str;
    ros::param::get("/OtterController/path_point", str);
    // std::cout << str << std::endl;
    GetFloat(str, m_path);//路径点数变多时对应修改point_num, 在stof中
    // std::cout << m_path[0] << std::endl;
    /*
    在这里订阅UBW进行计算，替换上面三个角度
    */
    //路径方向的切换与
    // std::vector<geometry_msgs::PoseStamped>::iterator closest;
    // for(auto it = m_path.poses.begin(); it != m_path.poses.end(); it++){

    //   double distance = dist(x_0, y_0, (it+1)->pose.position.x, (it+1)->pose.position.y);
    static int i = 0;

    // }
    if(1){

      if(dist(x_0, y_0, m_path[i+2], m_path[i+3]) < 0.8){

        if(i != (point_number-4)) i+=2;
        else{
          
          u = 0;//停船，这个u是期望速度，并且会发布出去，由于当前速度假设是2，因此当这个设置为2时船会停
          // i = 0;//循环航行 如果不需要循环直接注释掉即可。船的航向角就会沿着最后的路线的角度。
        }

        //循环运行可以在这里改逻辑
      }

      // std::cout << "x0 = " << m_path[i+0] << ", y0 = " << m_path[i+1] << std::endl;
      // std::cout << "x1 = " << m_path[i+2] << ", y1 = " << m_path[i+3] << std::endl;

    }

    // 将最终目标点坐标publish
    geometry_msgs::PoseStamped goal_point_;
    goal_point_.pose.position.x = m_path[point_number - 2];
    goal_point_.pose.position.y = m_path[point_number - 1];
    goal_point_pub.publish(goal_point_);

    followPath(x_0, y_0, heading_angle, m_path[i+0], m_path[i+1], m_path[i+2], m_path[i+3]);

    //发布两个点的坐标到tf坐标系中，可以在rviz中显示
    usv_pose.x = x_0;
    usv_pose.y = y_0;
    usv_pose.theta = heading_angle;

    usv_pose.x2 = x_1;
    usv_pose.y2 = y_1;
    usv_pose.theta2 = heading_angle;

    usv_pose_pub.publish(usv_pose);

    ros::spinOnce();
    rate.sleep();
  }
}

Guidance::~Guidance() {}

void Guidance::set_v_callback(const std_msgs::Float32& msg){

  // u = msg.data;

}

void Guidance::tagframe0Callback(const nlink_parser::LinktrackAnchorframe0 &msg){

  x_0 = msg.nodes[0].pos_3d[0];
  y_0 = msg.nodes[0].pos_3d[1];
  x_1 = msg.nodes[1].pos_3d[0];
  y_1 = msg.nodes[1].pos_3d[1];

  double delta_y = msg.nodes[0].pos_3d[1] - msg.nodes[1].pos_3d[1];
  double delta_x = msg.nodes[0].pos_3d[0] - msg.nodes[1].pos_3d[0];

  heading_angle = (atan2(delta_y, delta_x) / 3.14) * 180;

}

void Guidance::path_callback(const std_msgs::Float64MultiArray& msg){

  // for(int i =0; i < 10; i++)
  //   m_path[i] = msg.data[i];

}

void Guidance::newPoint(const geometry_msgs::PoseStamped& point) { 

  // m_path = path; 
  Point_goal = point;
  // std::cout << "订阅的目标点坐标(" << Point_goal.pose.position.x << ", " << Point_goal.pose.position.y << ")" << std::endl;
} 

void Guidance::start_Point(const geometry_msgs::PoseStamped& point) { 

  // m_path = path; 
  Point_start = point;
  // std::cout << "订阅的目标点坐标(" << Point_goal.pose.position.x << ", " << Point_goal.pose.position.y << ")" << std::endl;
} 

//讲m_path替换成我的方法
void Guidance::followPath(double x, double y, double psi, double x_start, double y_start, double x_goal, double y_goal)
// TODO: cuts turns, how to fix?
{

  // Finished? 根据导航点判断是否需要进行下面的计算
  // if (m_path.poses.size() <= 1) //m_path如何获取信息
  // {
  //   usv_msgs::SpeedCourse msg;
  //   msg.speed = 0.0; //发布一个速度0，可能是设定速度，可能是跟随完成后的设定速度 发布给controller
  //   msg.course = psi;
  //   m_controllerPub.publish(msg);
  //   return;
  // }

  //停船情况
  double dist = std::sqrt(std::pow(x_goal - x, 2) +
                      std::pow(y_goal - y, 2));


  // Identify closest point on path 确定路径上最近的一个点
  // std::vector<geometry_msgs::PoseStamped>::iterator closest;
  // double minDist = std::numeric_limits<double>::max();
  // for (auto it = m_path.poses.begin(); it != m_path.poses.end(); it++)
  // {
  //   double dist = std::sqrt(std::pow(x - it->pose.position.x, 2) +
  //                           std::pow(y - it->pose.position.y, 2));
  //   if (dist < minDist)
  //   {
  //     minDist = dist;
  //     closest = it;
  //   }
  // }

  // Store closest 存储最近的点到这个massage中
  // geometry_msgs::PoseStamped pose_d = *closest;

  // Erase previous elements 在存储的路径上擦除之前的位置
  //m_path.poses.erase(m_path.poses.begin(), closest);

  // Path tangential angle 路径切角
  double gamma_p = atan2(y_goal - y_start, x_goal - x_start); 

  double alfafi =  (90 - 180 * (atan2(y_goal - y, x_goal - x) / 3.14));
  
  if(dist < 0){

    u = 0;
    alfafi = alfafi;
    return;
  }

  //对于路径的设置：设定为在更新目标位置那一时刻船的坐标为开始坐标或者就直接手动设定

  // // Cross-track error 横向误差
  double y_e = -(x_goal - x) * std::sin(gamma_p) +
               (y_goal - y) * std::cos(gamma_p);

  // // Time-varying lookahead distance 时变前看距离 ,论文用的是船的长度的2-5倍
  double delta_y_e = 2 * 0.8;//根据地图大小给，在实验室模拟可以用小的
  // double delta_y_e =
  //     (delta_max - delta_min) * std::exp(-delta_k * std::pow(y_e, 2)) +
  //     delta_min;
  // // if turning => small lookahead distance 如果在转弯，需要减小前看距离
  // bool isTurning = false;
  // if ((closest + 1) != m_path.poses.end()) //如果还没到最后一个目标点
  // {
  //   double nextAngle = tf2::getYaw((*(closest + 1)).pose.orientation);
  //   if (std::fabs(gamma_p - nextAngle) > std::numeric_limits<double>::epsilon()) //并且下一条路径转弯很大
  //   {
  //     delta_y_e = delta_min;
  //     isTurning = true;
  //   }
  // }
  // LOS法的实现
  // velocity-path relative angle 速度路径相对角
  double chi_r = 180 * atan2(-y_e, delta_y_e) / 3.14;
  // std::cout << "路径角度： " << chi_r << std::endl;
  // std::cout << "ak角度： " << 90 - (180*gamma_p/3.14) << std::endl;

  // desired course angle 期望航向角
  double chi_d = 90 - (180*gamma_p/3.14) + chi_r;

  // calculate error in heading 计算期望航向角和航向角的误差
  double chi_err = chi_d - psi;

  //偏差映射解决-pi和pi的问题
  while (chi_err > 180)
  {
    chi_err -= 2 * 180;
  }
  while (chi_err < -180)
  {
    chi_err += 2 * 180;
  } //需要用来计算期望速度

  // calculate desired speed  计算期望速度
  // double u = m_maxSpeed * (1 - std::abs(y_e) / 5 - std::abs(chi_err) / M_PI_2);
  // u = std::max(u, m_minSpeed);
  // if (isTurning)
  //   u = m_maxSpeedTurn; //算得期望速度

  // Publish speed and course to controller  将期望速度和期望航向角度的msg发生给控制器 为什么不发偏差呢？
  usv_msgs::SpeedCourse msg;
  msg.speed = u;
  msg.course = chi_d;
  m_controllerPub.publish(msg);  //最终发布期望航向角度和速度给控制器控制

  //ROS_INFO_STREAM("psi_d: " << chi_d << " psi: " << psi);
  //ROS_INFO_STREAM("u_d: " << u);

}

} // namespace otter_coverage
