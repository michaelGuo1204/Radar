/*
串口通信模块
作者：hqy
*/

#ifndef _SERIAL_COM_HPP
#define _SERIAL_COM_HPP
/** TODO:改了通信协议，所以有很多地方要改*/
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <std_msgs/String.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/types.h>
#include <opencv2/highgui.hpp>
#include "serial_com/comm.h"
// #define SERIAL_DEBUG
#ifdef SERIAL_DEBUG
    #define serial_debug printf
    #define ros_debug ROS_INFO
#else
    #define serial_debug(...)
    #define ros_debug(...)
#endif
#define MAX_INFO_BYTES 1024*1024
//#define POSITION					//使用位置式的标签,默认取消，位置式云台会抽搐

class SerialCom{
public: 
    SerialCom();                                    //构造函数，在此打开串口
    ~SerialCom();                                   //析构
    ros::NodeHandle nh;                             //节点管理器
    ros::Subscriber para_sub;                       //接收
    ros::Publisher para_pub;                        //发送云台信息给图像处理节点        
    serial::Serial ser;                             
public:


    /**
     * @brief 数据转为 uint_8数组（即char数组），以进行云台板数据收发
     * @param buffer 入参/输出:用于储存转换后数据的容器
     * @param x 最优装甲板位置x
     * @param y 最优装甲板位置y
     * @param z 最优装甲板位置z
     * @param status 状态码
    */
    void getBuffer(uint8_t *buffer, float x, float y, int cnt);
    void infoExchange(const serial_com::comm::ConstPtr &msg); //回调函数，负责与云台板进行收发

private:
    int serialOK(char *output);                                     //查找可用设备，查找到将返回0

};
#endif //_SERIAL_COM_HPP
