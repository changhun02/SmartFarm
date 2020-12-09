#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_Q 0x71
// #define KEYCODE_B 0x62
// #define KEYCODE_C 0x63
// #define KEYCODE_D 0x64
// #define KEYCODE_E 0x65
// #define KEYCODE_F 0x66
// #define KEYCODE_G 0x67
// #define KEYCODE_R 0x72
// #define KEYCODE_T 0x74
// #define KEYCODE_V 0x76

class KeyboardReader
{
public:
    KeyboardReader() : kfd(0) {
        tcgetattr(kfd, &cooked);
        struct termios raw;
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);
    }

    void readOne(char *c){
        int rc = read(kfd, c, 1);
        if (rc < 0)
        {
            throw std::runtime_error("read failed");
        }
    }

    void shutdown(){
        tcsetattr(kfd, TCSANOW, &cooked);
    }

private:
    int kfd;
    struct termios cooked;
};






KeyboardReader input;

class Teleop
{
public:
    Teleop();
    void keyLoop();

private:
    ros::NodeHandle nh_;
    double linear_, angular_, l_scale_, a_scale_;
    ros::Publisher twist_pub_;
};

Teleop::Teleop() : linear_(0),
                   angular_(0),
                   l_scale_(2.0),
                   a_scale_(2.0)
{
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);

    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

void quit(int sig){
    (void)sig;
    input.shutdown();
    ros::shutdown();
    exit(0);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_keyboard");
    Teleop teleop_turtle;

    signal(SIGINT, quit);

    teleop_turtle.keyLoop();
    quit(0);

    return (0);
}



void Teleop::keyLoop()
{
    char c;

    puts("Reading from keyboard");
    puts("----------------------------------------------");
    puts("Use arrow keys to move the robot. 'q' to quit.");

    linear_ = angular_ = 0;
    for (;;)
    {
        try
        {
            input.readOne(&c);
        }
        catch (const std::runtime_error &)
        {
            perror("read():");
            return;
        }

        ROS_DEBUG("value: 0x%02X\n", c);

        switch (c)
        {
        case KEYCODE_LEFT:
            ROS_DEBUG("LEFT");
            angular_ = 1.0;
            break;
        case KEYCODE_RIGHT:
            ROS_DEBUG("RIGHT");
            angular_ = -1.0;
            break;
        case KEYCODE_UP:
            ROS_DEBUG("UP");
            linear_ = 1.0;
            break;
        case KEYCODE_DOWN:
            ROS_DEBUG("DOWN");
            linear_ = -1.0;
            break;
        case KEYCODE_Q:
            ROS_DEBUG("quit");
            return;
        }

        geometry_msgs::Twist twist;
        twist.angular.z = a_scale_ * angular_;
        twist.linear.x = l_scale_ * linear_;

        twist_pub_.publish(twist);
    }
    return;
}
