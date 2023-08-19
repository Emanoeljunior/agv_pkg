#include "ros/ros.h"
#include "geometry_msgs/Point.h"

class VelIntegrator{
    public:
        VelIntegrator(ros::NodeHandle *nh){
            dt = 1/60;
            x = 0;
            y = 0;
            z = 0;
            q_size = 60;
            velSub = nh->subscribe("/VelState", q_size, &VelIntegrator::updateSensorPoint, this);
            pointPub = nh->advertise<geometry_msgs::Point>("/PositionState", q_size);
        }
        void updateSensorPoint(const geometry_msgs::Point &msg){
            x = x + 100*msg.x/60;
            y = msg.y*dt;
            z = (msg.z-9.81)*dt;
            geometry_msgs::Point newMsg;
            newMsg.x = x;
            newMsg.y = y;
            newMsg.z = z;
            pointPub.publish(newMsg);
            //ROS_INFO("Aceleração x: %f", msg.x);
            //ROS_INFO("Velocidade x: %f", newMsg.x);
        }
    private:
        double x;
        double y;
        double z;
        double dt;
        int q_size;
        ros::Subscriber velSub;
        ros::Publisher pointPub;
};

int main(int argc,char **argv){
    ros::init(argc, argv, "PositionIntegrator");
    ros::NodeHandle nh;
    VelIntegrator vel_integrator = VelIntegrator(&nh);
    ros::spin();
    return 0;
}
