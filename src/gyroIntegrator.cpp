#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"
#include <math.h> 

class gyroIntegrator{
    public:
        gyroIntegrator(ros::NodeHandle *nh){
            angleZ = 0.0;
            speedSub = nh->subscribe("/movingAverageGyroscope", q_size, &gyroIntegrator::updateGyroAngle, this);
            gyroAnglePub = nh->advertise<std_msgs::Float64>("/gyroAngle", q_size);
        }
        void updateGyroAngle(const geometry_msgs::Point &msg){
            std_msgs::Float64 newMsg;
            angleZ = (angleZ + 0.1*msg.z);
            
            newMsg.data = fmod(angleZ, 50)*7.2;
            //if (newMsg.data < 0){
            //	newMsg.data = newMsg.data + 360;}
            gyroAnglePub.publish(newMsg);
        }
    private:
        double angleZ;
        int q_size;
        ros::Subscriber speedSub;
        ros::Publisher gyroAnglePub;

};

int main(int argc,char **argv){
    ros::init(argc, argv, "GyroIntegrator");
    ros::NodeHandle nh;
    gyroIntegrator gyro_integrator = gyroIntegrator(&nh);
    ros::spin();
    return 0;
}
