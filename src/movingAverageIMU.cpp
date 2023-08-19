#include "ros/ros.h"
#include "geometry_msgs/Point.h"

class movingAverage{
    public:
        movingAverage(ros::NodeHandle *nh){
            N = 4;
            meanAx = 0.0;
            meanAy = 0.0;
            meanAz = 0.0;
            meanGx = 0.0;
            meanGy = 0.0;
            meanGz = 0.0;
            accSub = nh->subscribe("/accelerometer_publisher", 10, &movingAverage::movingAverageAccCallback, this);
            gyroSub = nh->subscribe("/gyroscope_publisher", 10, &movingAverage::movingAverageGyroCallback, this);
            meanAccPub = nh->advertise<geometry_msgs::Point>("/movingAverageAccelerometer", 10);
            meanGyroPub = nh->advertise<geometry_msgs::Point>("/movingAverageGyroscope", 10);
            for(i=0;i<11;i++){
                    samplesAx[i] = 0;
                    samplesAy[i] = 0;
                    samplesAz[i] = 0;
                    samplesGx[i] = 0;
                    samplesGy[i] = 0;
                    samplesGz[i] = 0;}
                    
        }
        void  movingAverageAccCallback(const geometry_msgs::Point &msg){
            Ax = msg.x;
            Ay = msg.y;
            Az = msg.z;
            double tempAx = samplesAx[0];
            double tempAy = samplesAy[0];
            double tempAz = samplesAz[0];
            for(i=0;i<(N-1);i++){
                samplesAx[i] = samplesAx[i+1];
                samplesAy[i] = samplesAy[i+1];
                samplesAz[i] = samplesAz[i+1];}
            samplesAx[N] = Ax;
            samplesAy[N] = Ay;
            samplesAz[N] = Az;
            meanAx = meanAx + (Ax-tempAx)/10;
            meanAy = meanAy + (Ay-tempAy)/10;
            meanAz = meanAz + (Az-tempAz)/10;
            geometry_msgs::Point mAverage;
            mAverage.x = meanAx;
            mAverage.y = meanAy;
            mAverage.z = meanAz;
            meanAccPub.publish(mAverage);
        }
            
        void movingAverageGyroCallback(const geometry_msgs::Point &msg){
            Gx = msg.x;
            Gy = msg.y;
            Gz = msg.z;
            if (Gx<0.1 && Gx>-0.1){
                Gx = 0.00;
            }
            if (Gy<0.1 && Gy>-0.1){
                Gy = 0.00;
            }
            if (Gz<0.1 && Gz>-0.1){
                Gz = 0.00;
            }
            double tempGx = samplesGx[0];
            double tempGy = samplesGy[0];
            double tempGz = samplesGz[0];
            for(i=0;i<10;i++){
                samplesGx[i] = samplesGx[i+1];
                samplesGy[i] = samplesGy[i+1];
                samplesGz[i] = samplesGz[i+1];}
            samplesGx[10] = Gx;
            samplesGy[10] = Gy;
            samplesGz[10] = Gz;
            meanGx = meanGx + (Gx-tempGx)/10;
            meanGy = meanGy + (Gy-tempGy)/10;
            meanGz = meanGz + (Gz-tempGz)/10;
            if (meanGx<0.01 && meanGx>-0.01){
                meanGx = 0.00;
            }
            if (meanGy<0.01 && meanGy>-0.01){
                meanGy = 0.00;
            }
            if (meanGz<0.01 && meanGz>-0.01){
                meanGz = 0.00;
            }
            geometry_msgs::Point mAverageG;
            mAverageG.x = meanGx;
            mAverageG.y = meanGy;
            mAverageG.z = meanGz;
            meanGyroPub.publish(mAverageG);
        }
    private:
        ros::Subscriber accSub;
        ros::Subscriber gyroSub;
        ros::Publisher meanAccPub;
        ros::Publisher meanGyroPub;
        int i;
        int N;
        double meanAx;
        double meanAy;
        double meanAz;
        double meanGx;
        double meanGy;
        double meanGz;
        double Ax;
        double Ay;
        double Az;
        double Gx;
        double Gy;
        double Gz;
        double samplesAx [5];
        double samplesAy [5];
        double samplesAz [5];
        double samplesGx [11];
        double samplesGy [11];
        double samplesGz [11];
};

int main(int argc,char **argv){
    ros::init(argc, argv, "AverageFilterIMU");
    ros::NodeHandle nh;
    movingAverage movingaverage = movingAverage(&nh);
    ros::spin();
    return 0;
}
