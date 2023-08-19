#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"
#include <math.h>

class controller{
    public:
        controller(ros::NodeHandle *nh){
            ax = 0;
            ay = 0;
            atheta = 0;
            dx = 0;
            dy = 0;
            dtheta = 0;
            publishInterval = 0.1;
            desiredPoseSub = nh->subscribe("/futurePose", 10, &controller::updateDesiredPose, this);
            actualPoseSub = nh->subscribe("/Pose", 10, &controller::updateActualPose, this);
            distPub = nh->advertise<std_msgs::Float64>("/distState", 10);
            anglePub = nh->advertise<std_msgs::Float64>("/angleState", 10);
            timerPub = nh->createTimer(ros::Duration(publishInterval), &controller::timerCallback, this);
        }
        void updateDesiredPose(const geometry_msgs::Point &msg){
            dx = msg.x;
            dy = msg.y;
            dtheta = msg.z;
        }
        void updateActualPose(const geometry_msgs::Point &msg){
            ax = msg.x;
            ay = msg.y;
            atheta = msg.z;
        }
        void timerCallback(const ros::TimerEvent &event){
            std_msgs::Float64 dist;
            std_msgs::Float64 angle;
            dist.data = sqrt(pow(dx-ax,2)+pow(dy-ay,2)); 
            angle.data = dtheta-atheta;
            distPub.publish(dist);
            anglePub.publish(angle);
        }
    private:
        double ax;
        double ay;
        double atheta;
        double dx;
        double dy;
        double dtheta;
        double publishInterval;
        ros::Subscriber desiredPoseSub;
        ros::Subscriber actualPoseSub;
        ros::Publisher distPub;
        ros::Publisher anglePub;
        ros::Timer timerPub;
};

int main(int argc,char **argv){
    ros::init(argc, argv, "Controller");
    ros::NodeHandle nh;
    controller Controller = controller(&nh);
    ros::spin();
    return 0;
}
