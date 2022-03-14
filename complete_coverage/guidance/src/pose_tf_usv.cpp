#include "ros/ros.h"
#include "guidance/usv_pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

void dopose(const guidance::usv_pose::ConstPtr& pose){

    static tf2_ros::TransformBroadcaster broadcaster;

    geometry_msgs::TransformStamped tfs;
    geometry_msgs::TransformStamped tfs2;

    tfs.header.frame_id = "world";
    tfs.header.stamp = ros::Time::now();
    tfs2.header.frame_id = "world";
    tfs2.header.stamp = ros::Time::now();

    tfs.child_frame_id = "usv_1";
    tfs2.child_frame_id = "usv_2";

    tfs.transform.translation.x = pose->x;
    tfs.transform.translation.y = pose->y;
    tfs.transform.translation.z = 0.0;

    tfs2.transform.translation.x = pose->x;
    tfs2.transform.translation.y = pose->y;
    tfs2.transform.translation.z = 0.0;

    tf2::Quaternion qnt;
    qnt.setRPY(0, 0, pose->theta);
    tfs.transform.rotation.x = qnt.getX();
    tfs.transform.rotation.y = qnt.getY();
    tfs.transform.rotation.z = qnt.getZ();
    tfs.transform.rotation.w = qnt.getW();

    tf2::Quaternion qnt2;
    qnt2.setRPY(0, 0, pose->theta);
    tfs2.transform.rotation.x = qnt2.getX();
    tfs2.transform.rotation.y = qnt2.getY();
    tfs2.transform.rotation.z = qnt2.getZ();
    tfs2.transform.rotation.w = qnt2.getW();

    broadcaster.sendTransform(tfs);
    broadcaster.sendTransform(tfs2);


}

int main(int argc, char *argv[]){

    ros::init(argc, argv, "usv_pose_tf_node");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<guidance::usv_pose>("usv/pose", 1000, dopose);

    ros::spin();
    
    return 0;

}