#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"

class distanceControl{
    public:
        distanceControl(ros::NodeHandle *nh){
            Vx = 0;
            dt = 0.1;
            dx = 0;
            P = 1;
            U = 0;
            publishInterval = 0.1;
            velSub = nh->subscribe("/velState", 10, &distanceControl::updateVel, this);
            distSub = nh->subscribe("/distState", 10, &distanceControl::updateDist, this);
            leftPub = nh->advertise<std_msgs::Float64>("/left_motor", 10);
            rightPub = nh->advertise<std_msgs::Float64>("/right_motor", 10);
            timerControl = nh->createTimer(ros::Duration(publishInterval), &distanceControl::control, this);
        }
        void control(const ros::TimerEvent &event){
            U = -(2/(dt*dt*P))*dx-(4*(1-P)/(dt*dt*dt*P*P*P)+2/(dt*P))*Vx;
            std_msgs::Float64 pwm;
            pwm.data = U;
            leftPub.publish(pwm);
            rightPub.publish(pwm);
        }
        void updateVel(const std_msgs::Float64 &msg){
            Vx = msg.data;
        }
        void updateDist(const std_msgs::Float64 &msg){
            dx = msg.data;
        }
    private:
        double Vx;
        double dt;
        double dx;
        double P;
        double U;
        double publishInterval;
        ros::Subscriber velSub;
        ros::Subscriber distSub;
        ros::Timer timerControl;
        ros::Publisher leftPub;
        ros::Publisher rightPub;
};

int main(int argc,char **argv){
    ros::init(argc, argv, "distanceController");
    ros::NodeHandle nh;
    distanceControl distancecontrol = distanceControl(&nh);
    ros::spin();
    return 0;
}
