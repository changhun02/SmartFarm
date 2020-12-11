// This Package will not use

#include <ros/ros.h>
#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/LaserScan.h"

sensor_msgs::PointCloud2 changed_pc2;

sensor_msgs::PointCloud2 laser2cloudmsg(sensor_msgs::LaserScan laser){
   static laser_geometry::LaserProjection projector;
   sensor_msgs::PointCloud2 pc2_dst;
   projector.projectLaser(laser, pc2_dst,-1,laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
   pc2_dst.header.frame_id = "map";

   return pc2_dst;
}

void lidar_callback(const sensor_msgs::LaserScan laser){
  changed_pc2 = laser2cloudmsg(laser);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_to_pc2");
  ros::NodeHandle nh;
  ros::Subscriber lidar_sub = nh.subscribe("/scan_filtered", 1, lidar_callback);
  ros::Publisher pc2_pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1000);

  ros::Rate loop_rate(10);
  while(ros::ok()){
    pc2_pub.publish(changed_pc2);
  }
}
