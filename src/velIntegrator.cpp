#include "ros/ros.h"
#include "geometry_msgs/Point.h"

class VelIntegrator{
    public:
        VelIntegrator(ros::NodeHandle *nh){
            Vx = 0;
            Vy = 0;
            Vz = 0;
            dt = 1/60;
            accSub = nh->subscribe("/accelerometer_publisher", 60, &VelIntegrator::updateSensorPoint, this);
            sensorPointPub = nh->advertise<geometry_msgs::Point>("/VelState", 60);
        }
        void updateSensorPoint(const geometry_msgs::Point &msg){
            Vx = Vx + msg.x/60;
            Vy = Vy + msg.y * dt;
            Vz = Vz + (msg.z-9.81) * dt;
            geometry_msgs::Point newMsg;
            newMsg.x = Vx;
            newMsg.y = Vy;
            newMsg.z = Vz;
            sensorPointPub.publish(newMsg);
            ROS_INFO("Aceleracao x: %f", msg.x);
            ROS_INFO("Velocidade x: %f", newMsg.x);
        }
    private:
        double Vx;
        double Vy;
        double Vz;
        double dt;
        ros::Subscriber accSub;
        ros::Publisher sensorPointPub;
};

int main(int argc,char **argv){
    ros::init(argc, argv, "VelIntegrator");
    ros::NodeHandle nh;
    VelIntegrator vel_integrator = VelIntegrator(&nh);
    ros::spin();
    return 0;
}
