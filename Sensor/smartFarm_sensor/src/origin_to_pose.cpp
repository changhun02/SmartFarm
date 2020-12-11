#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

tf::Transform transform;
tf::Quaternion q;
geometry_msgs::Point robot_pos;
double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;

void pose_callback(const geometry_msgs::PoseStampedPtr &pose){
   roll   = pose->pose.orientation.x;
   pitch  = pose->pose.orientation.y;
   yaw    = pose->pose.orientation.z;
}
void odom_callbaack(const nav_msgs::OdometryPtr &odom){
  robot_pos.x = odom->pose.pose.position.x;
  robot_pos.y = odom->pose.pose.position.y;
  robot_pos.z = odom->pose.pose.position.z;
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "origin_pose");
   ros::NodeHandle n("~");
   ros::Subscriber pose_sub = n.subscribe("/slam_out_pos", 1, pose_callback);
   ros::Subscriber odom_sub = n.subscribe("/scanmatch_odom", 1, odom_callbaack);
   ros::Publisher pose_pub = n.advertise<geometry_msgs::Point>("/robot_pos", 1000);
   ros::Rate loop_rate(100);
   while (ros::ok())
   {
      ros::spinOnce();
      pose_pub.publish(robot_pos);
      loop_rate.sleep();
      //ROS_INFO("X : %f Y : %f Z : %f", x, y, z);
   }
}
