#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
//#include <wiringPi.h>

class Vel_sub{                             
    public:
        Vel_sub(ros::NodeHandle *nh){
            left = 0;
            right = 0;
	    std::cout << "inside init \n";
            publish_interval = 0.5;
            vel_sub = nh->subscribe("cmd_vel", 10, &Vel_sub::motorCallback, this);
            left_pub = nh->advertise<std_msgs::Float64>("/left_motor", 10);
            right_pub = nh->advertise<std_msgs::Float64>("/right_motor", 10);
            timer_pub = nh->createTimer(ros::Duration(publish_interval), &Vel_sub::timerCallback, this);
        }
        void motorCallback(const geometry_msgs::Twist &msg){
            std_msgs::Float64 left_msg;
            std_msgs::Float64 right_msg;
            std::cout << "inside callback \n";
	    std::cout << msg;

            left = (msg.linear.x-msg.angular.z)/2;
            left = msg.linear.x*10;
            left_msg.data = left;
	    std::cout << msg.linear.x;

            right = (msg.linear.x+msg.angular.z)/2;
            right_msg.data = right;

            left_pub.publish(left_msg);
            right_pub.publish(right_msg);
        //   
        };
        void timerCallback(const ros::TimerEvent &event){
            std_msgs::Float64 left_msg;
            std_msgs::Float64 right_msg;

            left = 0;
            left_msg.data = left;


            right = 0;
            right_msg.data = right;

            left_pub.publish(left_msg);
            right_pub.publish(right_msg);
        };

    private:
        int left;
        int right;
        double publish_interval;
        ros::Subscriber vel_sub;
        ros::Publisher left_pub;
        ros::Publisher right_pub;
        ros::Timer timer_pub;
};

int main(int argc,char **argv){

    std::cout << "before subscribe\n";
    ros::init(argc, argv, "vel_manual");
    ros::NodeHandle nh;
    Vel_sub vel_manual = Vel_sub(&nh);
    ros::spin();

    return 0;
}
