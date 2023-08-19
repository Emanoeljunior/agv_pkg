#include "ros/ros.h"
#include "geometry_msgs/Point.h"

class calibration{
    public:
        calibration(ros::NodeHandle *nh){
            counterA = 0;
            counterG = 0;
            N = 200;
            for(i=0;i < 3;i++){
                averageAcc[i] = 0;
                averageGyro[i] = 0;}
            accSub = nh->subscribe("/accelerometer_publisher", 60, &calibration::updateAverageAcc, this);
            gyroSub = nh->subscribe("/gyroscope_publisher", 60, &calibration::updateAverageGyro, this);
        }
        void updateAverageAcc(const geometry_msgs::Point &msg){
            if (calibrationA){
                if(counterA<N){
                    averageAcc[0] = averageAcc[0]+ msg.x;
                    averageAcc[1] = averageAcc[0]+ msg.y;
                    averageAcc[2] = averageAcc[0]+ msg.z;
                    counterA ++;}
                else{
                    geometry_msgs::Point newMsg;
                    newMsg.x = averageAcc[0]/N;
                    newMsg.y = averageAcc[1]/N;
                    newMsg.z = averageAcc[2]/N;
                    calibrationA = false;
                    ROS_INFO("Accelerometer calibration complete: %f", averageAcc[0]);

            }}
            else{
                ROS_INFO("Accelerometer calibration complete: %f", averageAcc[0]);
            }
        }
        void updateAverageGyro(const geometry_msgs::Point &msg){
            if (calibrationG){
                if(counterG<N){
                    averageGyro[0] = averageGyro[0]+ msg.x;
                    averageGyro[1] = averageGyro[0]+ msg.y;
                    averageGyro[2] = averageGyro[0]+ msg.z;
                    counterG ++;}
                else{
                    geometry_msgs::Point newMsg;
                    newMsg.x = averageGyro[0]/N;
                    newMsg.y = averageGyro[1]/N;
                    newMsg.z = averageGyro[2]/N;
                    calibrationG = false;
                    ROS_INFO("Gyroscope calibration complete: %f", averageGyro);

            }}
            else{
                ROS_INFO("Gyroscope calibration complete: %f", averageGyro);
            }
        }

    private:
        int i;
        bool calibrationA;
        bool calibrationG;
        double N;
        long double counterA;
        long double counterG;
        double averageAcc[3];
        double averageGyro[3];
        ros::Subscriber accSub;
        ros::Subscriber gyroSub;
};

int main(int argc,char **argv){

    ros::init(argc, argv, "calibration");
    ros::NodeHandle nh;
    calibration Calibration = calibration(&nh);
    ros::spin();

    return 0;
}
