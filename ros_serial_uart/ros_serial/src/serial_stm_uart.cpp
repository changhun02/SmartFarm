// By serial uart comm, read Bluetooth RSSI data & Sonar array data

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "serial/serial.h"
#include "cstdlib"
#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "stdio.h"

serial::Serial ser;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_stm_uart");
  ros::NodeHandle nh;

  std::string port;
  nh.param<std::string>("port", port, "/dev/ttyUSB0");

  try
  {
      ser.setPort(port);
      ser.setBaudrate(115200);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();
  }
  catch (serial::IOException& e)
  {
      ROS_ERROR_STREAM("Unable to open port ");
      return -1;
  }

  if(ser.isOpen())
  {
      ROS_INFO_STREAM("Serial Port initialized");
  }
  else
  {
      return -1;
  }

  ros::Rate loop_rate(20);
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
}
