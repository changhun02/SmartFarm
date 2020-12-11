// By serial uart comm, read Bluetooth RSSI data & Sonar array data
// Must be solve this Problem

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

//Seiral Comm
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/poll.h>
#include <termios.h>
#include <fcntl.h>

using namespace std;

#define COM_STATE_CONNECTION    1
#define COM_STATE_UNCONNECTION  0
#define BUF_SIZE                12
#define RSSI_BUF_SIZE           3
#define SONAR_BUF_SIZE          6

serial::Serial ser;

unsigned int start_bit_1 = 0xF0, start_bit_2 = 0xF1;
unsigned int Sonar_buf[SONAR_BUF_SIZE];
int Sonar[3];
unsigned int RSSI_buf[RSSI_BUF_SIZE];
int RSSI = 0;
int start_cnt = 0;

uint8_t buf[BUF_SIZE];
unsigned buff[BUF_SIZE];
vector<uint8_t> v_buf(12);

// Check if Serial Communication is Successful
void Serial_check(){
  ser.read(buf, 12);
  for(int i = 0 ; i < 12 ; i++){
    if((unsigned int)buf[i] == start_bit_1)
      start_cnt = i;
  }
  for(int j = 0 ; j < 12 ; j++){
    if(start_cnt < 12){
      buff[j] = buf[start_cnt];
      start_cnt++;
    }
    else{
      buff[j] = buf[start_cnt - 12];
      start_cnt++;
    }
  }
  start_cnt = 0;
  for(int k = 0 ; k < 12 ; k++){
    cout << buff[k] << " ";
  }
  cout << endl;
}
unsigned int array_checksum(){
  unsigned int sum = 0;
  for(int i = 4 ; i <= 10 ; i++){
    sum += buff[i + 8];
  }
  if(buff[11] == sum)
    return true;
  else {
    return false;
  }
}

bool passing(){
  if(buff[0] == start_bit_1){
    if(buff[1] == start_bit_2){
      if(array_checksum()){
        cout << "Correct CheckSum" << endl;
        for(int i = 0 ; i < 3 ; i ++){
          RSSI_buf[i] = buff[i + 2];
        }
        RSSI = atoi(RSSI_buf);
        for(int j = 0 ; j < 6 ; j++){
          Sonar_buf[j] = buff[j + 5];
        }
      }
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_stm_uart");
  ros::NodeHandle nh;

  std::string port;
  nh.param<std::string>("port", port, "/dev/ttyUSB1");
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

  // The Hz must be less than 50
  ros::Rate loop_rate(20);
  string str;
  while(ros::ok()){
    ros::spinOnce();

    // Read data from stm32(RSSI, Sonar)

    Serial_check();

   /* ser.read(buf, 12);
    //ser.read(str, 12);
    //ser.read(v_buf, 12);
    for(int i = 0 ; i < 12 ; i++){
      cout << (unsigned int)buf[i] << " " ;
    }

    cout << endl;*/
    loop_rate.sleep();
  }
}
