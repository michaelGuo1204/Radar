#include <cstdio>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "mapwork/Radar.hpp"
#include "serial_com/comm.h"

int main(int argc, char ** argv){
    cv::setNumThreads(2);
    ros::init(argc, argv, "main_frame");
    ros::NodeHandle nh;
    ros::Publisher stm32_pub ;
    stm32_pub = nh.advertise<serial_com::comm>("cameraData", 1);
    ros::Rate loop_rate(10);
    Radar radar;
    do{
        radar.getframe();
        radar.Radarwork();
        stm32_pub.publish(radar.g_msg);

    } while(ros::ok());
    


}