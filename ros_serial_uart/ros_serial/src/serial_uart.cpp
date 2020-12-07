#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "serial/serial.h"
#include "cstdlib"
#include "stdlib.h"
#include "math.h"
#include "stdio.h"

#define ROBOT_STATE   0
#define STEERING_DATA 11
#define SPEED_DATA    11

serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    double x;                                 // 2:UP     -2:DOWN
    double z;                                 // 2:LEFT   -2:RIGHT
    unsigned char cmdBuff[8];
    unsigned short check_sum;

// cmdBuff >> BYTE
//
    cmdBuff[0] = '$';
    cmdBuff[1] = '3';                           //Binary "0000 0011": 로봇 스피드와 스티어링 데이터 사용
    cmdBuff[2] = '5';                           // Data Payload Length + Checksum Length
    cmdBuff[3] = '1';                           // 0: StandBy, 1: Move
    cmdBuff[4] = 60;                          // -100 ~ 100 [%]
    cmdBuff[5] = 60;                          // -100 ~ 100 [%]
    for (int i = 1; i <= 5; i++)
        check_sum += cmdBuff[i];

    cmdBuff[6] = (check_sum >> 8) & 0x00FF;
    cmdBuff[7] = check_sum & 0x00FF;


    x = msg->linear.x;
    z = msg->angular.z;
    printf("linear  x : %.1f\n", msg->linear.x);
    printf("angular z : %.1f\n", msg->angular.z);
    for(int i=0; i<sizeof(cmdBuff)/sizeof(cmdBuff[0]); i++)
        printf("%x,", cmdBuff[i]);
    printf("\n---------\r\n");
    
    std::string cmd(reinterpret_cast<char const*>(cmdBuff));

    ser.write(cmd);
    ser.write("\r\n");
}



int main (int argc, char** argv)
{
    ros::init(argc, argv, "ros_serial");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    //ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

    ros::Subscriber goal_vel = nh.subscribe("cmd_vel",1000,cmd_vel_callback);

    try
    {
        ser.setPort("/dev/ttyUSB1");
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

    ros::Rate loop_rate(5);

    while(ros::ok())
    {

        ros::spinOnce();

        if(ser.available())
        {
            //ROS_INFO_STREAM("Reading from serial port");
            //std_msgs::String result;
            //result.data = ser.read(ser.available());

            //ROS_INFO_STREAM("Read: " << result.data);
            //read_pub.publish(result);
        }
        loop_rate.sleep();
    }
}