#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "math.h"

geometry_msgs::Point person_, obj_, robot_;
geometry_msgs::Twist cmd_;

// #################### Pose CallBack (Robot, People, Object)
void robot_pos_callback(const geometry_msgs::Point &ptr){
  robot_ = ptr;
}
void people_pos_callback(const geometry_msgs::Point &ptr){
  person_ = ptr;
}
void obj_pos_callback(const geometry_msgs::Point &ptr){
  obj_ = ptr;
}

// #################### Distance callback between robot and others(people, object)
double dist_(geometry_msgs::Point p){
  return sqrt(pow(p.x - robot_.x, 2) + pow(p.y - robot_.y, 2));
}
bool threshold(double p_dist, double o_dist){
  double th = 0.05;

  if((p_dist - o_dist) < th)
    return true;
  else if((p_dist - o_dist) < 0.0)
    return true;
  else
    return false;

}
bool Twist_callback(){
  double obj_dist = dist_(obj_);
  double people_dist = dist_(person_);
  if(threshold(people_dist, obj_dist)){
    // People and obj is almost same, No obj or obj is far
    cmd_.linear.x = 1.0; // Go
  }
  else{
    cmd_.linear.x = 0.0; // Stop
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "total");
  ros::NodeHandle nh;

  ros::Subscriber people_pos = nh.subscribe("/people_pos", 1, people_pos_callback);
  ros::Subscriber obj_pos = nh.subscribe("/obj_pos", 1, obj_pos_callback);
  ros::Subscriber robot_pos = nh.subscribe("/robot_pos", 1, robot_pos_callback);
  ros::Subscriber camera_people_pos;      // Later update
  ros::Subscriber camera_obj_pos;         // Later update

  ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  ros::Rate rate(50);
  while(ros::ok()){
    ros::spinOnce();
    Twist_callback();
    cmd_pub.publish(cmd_);
    rate.sleep();
  }
}
