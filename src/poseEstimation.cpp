#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"
#include <math.h>

class poseEstimator{
    public:
        poseEstimator(ros::NodeHandle *nh){
            dt = 1/60;
            x = 0;
            y = 0;
            theta = 0;
            V = 0;
            velSub = nh->subscribe("/VelState", 10, &poseEstimator::updateVel, this);
            angleSub = nh->subscribe("/gyroAngle", 10, &poseEstimator::updateAngle, this);
            timerPub = nh->createTimer(ros::Duration(dt), &poseEstimator::timerCallback, this);
            pointPub = nh->advertise<geometry_msgs::Point>("/Pose", 10);
        }
        void updateVel(const geometry_msgs::Point &msg){
            V = msg.x;
        }
        void updateAngle(const std_msgs::Float64 &msg){
            theta = msg.data;
        }
        void timerCallback(const ros::TimerEvent &event){
            geometry_msgs::Point newMsg;
            newMsg.x = x + V*dt*cos(theta);
            newMsg.y = y + V*dt*sin(theta);
            newMsg.z = theta;
        }
    private:
        double x;
        double y;
        double theta;
        double dt;
        double V;
        ros::Subscriber velSub;
        ros::Subscriber angleSub;
        ros::Publisher pointPub;
        ros::Timer timerPub;
};

int main(int argc,char **argv){
    ros::init(argc, argv, "PositionIntegrator");
    ros::NodeHandle nh;
    poseEstimator pose = poseEstimator(&nh);
    ros::spin();
    return 0;
}
