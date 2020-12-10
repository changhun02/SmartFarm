// By serial uart comm, read Bluetooth RSSI data & Sonar array data

#include <ros/ros.h>
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

#define COM_STATE_CONNECTION    1
#define COM_STATE_UNCONNECTION  0
#define BUF_SIZE                11
#define RSSI_BUF_SIZE           11
#define SONAR_BUF_SIZE          11
serial::Serial ser;

unsigned int start_bit_1 = 0xF0, start_bit_2 = 0xF1;
unsigned int Sonar_buf[SONAR_BUF_SIZE];
unsigned char RSSI[RSSI_BUF_SIZE];
uint8_t buf[BUF_SIZE];

// Check if Serial Communication is Successful
bool Serial_check(uint8_t *a){
  unsigned int sum = 0;
  if(a[0] == start_bit_1){
    if(a[1] == start_bit_2){
      if(a[2] == COM_STATE_CONNECTION){
        for(int i = 3 ; i <= 8 ; i++){
          sum += a[i];
        }

        if(sum == a[9])
          return true;
        else
          return false;
      }
      else{
        return false;
      }
    }
    else{
      return false;
    }
  }
  else {
    return false;
  }
}
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
    // Read data from stm32(RSSI, Sonar)
    ser.read(buf, BUF_SIZE);

    // if Serial is Successful
    if(Serial_check(buf)){
      int cnt = 0;

      // RSSI data Passing(ASCII)
      for(int i = 3 ; i <= 6 ; i++){
        RSSI[cnt] = (unsigned char)buf[i];
        cnt ++;
      }

      cnt = 0;

      // Sonar data Passing(unsigned int)
      for(int j = 7 ; j <= 9 ; j++){
        Sonar_buf[cnt] = buf[j];
        cnt ++;
      }

    }

    loop_rate.sleep();
  }
}
