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

#define ROBOT_STATE_FORWARD   1
#define ROBOT_STATE_STOP      0
#define STEERING_DATA 11
#define SPEED_DATA    11

serial::Serial ser;
unsigned char STX = '$';

void write_callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    int x;                                 // 2:UP     -2:DOWN
    int z;                                 // 2:LEFT   -2:RIGHT
    x = (int)msg->linear.x * 10;
    z = (int)msg->angular.z * 20;
    printf("linear  x : %d\n", x);
    printf("angular z : %d\n", z);

    uint8_t cmdBuff[8];
    unsigned short check_sum;

    cmdBuff[0] = '$';
    cmdBuff[1] = 3;                           //Binary "0000 0011": 로봇 스피드와 스티어링 데이터 사용
    cmdBuff[2] = 5;                           // Data Payload Length + Checksum Length
    cmdBuff[3] = 1;            // 0: StandBy, 1: Move
    cmdBuff[4] = z;                           // -100 ~ 100 [%]
    cmdBuff[5] = x;                          // -100 ~ 100 [%]
    for (int i = 1; i <= 5; i++)
        check_sum += cmdBuff[i];

    cmdBuff[6] = (check_sum >> 8) & 0x00FF;
    cmdBuff[7] = check_sum & 0x00FF;

    for(int i = 0; i < 8 ; i++)
        printf("%x,", cmdBuff[i]);
    printf("\r\n----------------\n");
    
    ser.write(cmdBuff,sizeof(cmdBuff));
//    std::string cmd(cmdBuff);
//    ser.write(cmd);
    ser.write("\r\n");
}



int main (int argc, char** argv)
{
    ros::init(argc, argv, "ros_serial");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

    ros::Subscriber goal_vel = nh.subscribe("cmd_vel",1000,cmd_vel_callback);
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

    while(ros::ok())
    {

        ros::spinOnce();

        if(ser.available())
        {
//            printf("Reading from serial port\n");
//            std_msgs::String result;
//            result.data = ser.read(ser.available());

//            for(int i=0; i<sizeof(result); i++)
//              printf("%x,",result.data[i]);
//            read_pub.publish(result);
//            printf("\r\n----------------\n");
        }
        loop_rate.sleep();
    }
}
