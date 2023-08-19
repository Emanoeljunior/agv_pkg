#include "ros/ros.h"
#include "geometry_msgs/Point.h"


class movingAverage{
    public:
        movingAverage(ros::NodeHandle *nh){
            N = 20;
            q_size = 60;
            meanAx = 0.0;
            meanAy = 0.0;
            meanAz = 0.0;
            tempAx= 0.0;
            tempAy= 0.0;
            tempAz= 0.0;

            //publish_interval = 0.1;
            accSub = nh->subscribe("/accelerometer_publisher", q_size, &movingAverage::movingAverageAccCallback, this);
            meanAccPub = nh->advertise<geometry_msgs::Point>("/movingAverageAccelerometer20", q_size);
            //timerPub = nh->createTimer(ros::Duration(publish_interval), &movingAverage::timerCallback, this);
            for(i=0;i<(N+1);i++){
                    samplesAx[i] = 0;
                    samplesAy[i] = 0;
                    samplesAz[i] = 0;}
                    
        }
        void  movingAverageAccCallback(const geometry_msgs::Point &msg){
            Ax = msg.x;
            Ay = msg.y;
            Az = msg.z;
            tempAx = samplesAx[0];
            tempAy = samplesAy[0];
            tempAz = samplesAz[0];
            for(i=0;i<(N-1);i++){
                samplesAx[i] = samplesAx[i+1];
                samplesAy[i] = samplesAy[i+1];
                samplesAz[i] = samplesAz[i+1];}
            samplesAx[N] = Ax;
            samplesAy[N] = Ay;
            samplesAz[N] = Az;
            meanAx = meanAx + (Ax-tempAx)/N;
            meanAy = meanAy + (Ay-tempAy)/N;
            meanAz = meanAz + (Az-tempAz)/N;
            geometry_msgs::Point mAverage;
            mAverage.x = meanAx;
            mAverage.y = meanAy;
            mAverage.z = meanAz;
            meanAccPub.publish(mAverage);
        }
        void timerCallback(const ros::TimerEvent &event){

        }
    private:
        ros::Subscriber accSub;
        ros::Publisher meanAccPub;
        //ros::Timer timerPub;
        int i;
        int N;
        int q_size;
        //double publish_interval;
        double meanAx;
        double meanAy;
        double meanAz;
        double Ax;
        double Ay;
        double Az;
        double samplesAx [20];
        double samplesAy [20];
        double samplesAz [20];
        double tempAx;
        double tempAy;
        double tempAz;
};

int main(int argc,char **argv){
    ros::init(argc, argv, "AverageFilterAcc");
    ros::NodeHandle nh;
    movingAverage movingaverage = movingAverage(&nh);
    ros::spin();
    return 0;
}
