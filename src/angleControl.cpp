#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include <math.h>

class angleControl{                             
    public:
        angleControl(ros::NodeHandle *nh){
            left = 0;
            right = 0;
            publish_interval = 0.5;
            angle_reference = 90;
            tolerance = sin(0.1);
            angle_sub = nh->subscribe("/gyroAngle", 5, &angleControl::motorCallback, this);
            left_pub = nh->advertise<std_msgs::Float64>("/left_motor", 10);
            right_pub = nh->advertise<std_msgs::Float64>("/right_motor", 10);
            timer_pub = nh->createTimer(ros::Duration(publish_interval), &angleControl::timerCallback, this);
        }
        void motorCallback(const std_msgs::Float64 &msg){
            dAngle = (angle_reference-msg.data)/2;
            ROS_INFO("DIFERENCA ANGULAR: %f", (2*dAngle)); 

        };
        void timerCallback(const ros::TimerEvent &event){
            std_msgs::Float64 left_msg;
            std_msgs::Float64 right_msg;
            S = sin(dAngle*3.14/360);
            if (fabs(S)>tolerance){
                if (fabs(S)<0.4){
                    left = copysign(0.4,-S);
                    right = copysign(0.4,S);
                }
                else {
                    left = -S;
                    right = S;
                    } 
            }
            else{
                left = 0;
                right = 0;}
            left_msg.data = left;
            right_msg.data = right;
            left_pub.publish(left_msg);
            right_pub.publish(right_msg);};
    private:
        double left;
        double right;
        double publish_interval;
        double angle_reference;
        double dAngle;
        double tolerance;
        double right_sign;
        double left_sign;
        double S;
        ros::Subscriber angle_sub;
        ros::Publisher left_pub;
        ros::Publisher right_pub;
        ros::Timer timer_pub;
};

int main(int argc,char **argv){

    ros::init(argc, argv, "angleControl");
    ros::NodeHandle nh;
    angleControl angle_control = angleControl(&nh);
    ros::spin();

    return 0;
}
