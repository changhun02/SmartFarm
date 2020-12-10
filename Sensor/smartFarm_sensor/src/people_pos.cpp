#include <ros/ros.h>
#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <geometry_msgs/Point.h>

#include <algorithm>
#include <vector>
#include <math.h>

std::vector<geometry_msgs::Point> v_pos(4);
std::vector<geometry_msgs::Point> all_pos;

bool compare(geometry_msgs::Point a, geometry_msgs::Point b){
  return (pow(a.x, 2) + pow(a.y, 2)) < (pow(b.x, 2) + pow(b.y, 2));
}

// Filter the Numbers of people
void pose_callback(const people_msgs::PositionMeasurementArray &ptr){
  ROS_INFO("%lu", ptr.people.size());

  for(unsigned long i = 0 ; i < ptr.people.size() ; i++){
    all_pos.push_back(ptr.people[i].pos);
  }
  sort(all_pos.begin(), all_pos.end(), compare);
  all_pos.resize(4);
  v_pos = all_pos;

  for(unsigned long i  = 0 ; i < v_pos.size() ; i++)
    ROS_INFO("%f, %f", v_pos[i].x, v_pos[i].y);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "people_pos");
  ros::NodeHandle nh;
  ros::Subscriber people_pos = nh.subscribe("/leg_tracker_measurements", 1, pose_callback);
  ros::Rate rate(100);
  while (ros::ok())
  {
     ros::spinOnce();
     rate.sleep();

  }
}
