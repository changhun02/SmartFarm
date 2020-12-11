// subscribe /scan_obj_filtered and find out what is object
// PointCloub & LaserScan size gap >> range_data made by PointCloud


#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/Point.h"
#include "math.h"
#include <vector>
#include <algorithm>

using namespace std;

laser_geometry::LaserProjection projector_;
sensor_msgs::PointCloud cloud;

vector<double>range_data;
double min_range = 0;
int min_range_idx = 0;;

void range_callback(const sensor_msgs::LaserScan &ptr){
  projector_.projectLaser(ptr, cloud);
}

void range_calc(sensor_msgs::PointCloud pc){
  range_data.clear();
  for(unsigned long i = 0 ; i < pc.points.size() ; i++){
    range_data.push_back(sqrt(pow(pc.points[i].x,2) + pow(pc.points[i].y,2)));
  }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_object_detect");
  ros::NodeHandle nh;
  ros::Subscriber range_sub = nh.subscribe("/scan_obj_filtered", 1, range_callback);
  ros::Publisher obj_pub = nh.advertise<geometry_msgs::Point32>("obj_pos", 1000);;
  ros::Rate rate(10);

  while(ros::ok()){
    ros::spinOnce();

    if(cloud.points.size() != 0){
      range_calc(cloud);
      ROS_INFO("%lu, %lu", cloud.points.size(), range_data.size());
      min_range = *min_element(range_data.begin(), range_data.end());
      min_range_idx = min_element(range_data.begin(), range_data.end()) - range_data.begin();
      ROS_INFO("Min dist : %f, Index : %d", min_range, min_range_idx);
      ROS_INFO("Min dist's X : %f, Min dist's Y : %f", cloud.points[min_range_idx].x, cloud.points[min_range_idx].y);
      obj_pub.publish(cloud.points[min_range_idx]);
    }
    rate.sleep();
  }
}
