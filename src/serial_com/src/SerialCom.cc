#include "serial_com/SerialCom.hpp"

SerialCom::SerialCom(){
    try{
        char path[128]="/dev/serial/by-id/";
        if(!serialOK(path)==0){
            ROS_ERROR_STREAM("Unable to open port. Not exception. Possible null path detected.");
            exit(-1);
        }
        ser.setPort(path);
        std::cout<< path <<std::endl;     
        serial::Timeout to = serial::Timeout::simpleTimeout(10);
        ser.setBaudrate(115200);
        //串口设置timeout
        ser.setTimeout(to);
        ser.open();
        printf("Serial port opened.\n");
    }
    catch (serial::IOException &e){
        ROS_ERROR_STREAM("Unable to open port with Exception.");
        exit(-1);
    }
    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized.");
    }
    else{
        ROS_ERROR_STREAM("Open serial port failed.");
        exit(-1);
    }
    para_sub = nh.subscribe("cameraData", 4, &SerialCom::infoExchange, this);
    ROS_INFO_STREAM("Topic subscribed.\n");
}

SerialCom::~SerialCom(){
    ser.close();
}

//把需要的数据转换成uint8_t
// 工控机 -> 云台版数据转换
void SerialCom::getBuffer(uint8_t *buffer, float x, float y, int cnt){
    unsigned int f1 = (*((unsigned int *)&(x)));
    unsigned int f2 = (*((unsigned int *)&(y)));
    //buffer是8位unsigned int 数组，每一位代表一字节
    for (int i = cnt; i < cnt + 4; ++i){                //4个字节的float x转换为buffer中的4位
        uint8_t tmp = (f1 >> (8 * i)) & 0xff;
        buffer[i] = tmp;                    
    }
    for (int i = cnt; i < cnt + 4; ++i){                //4个字节的float y转换为buffer中的4位
        uint8_t tmp = (f2 >> (8 * i)) & 0xff;
        buffer[i + 4] = tmp;
    }
}

//串口收发数据测试
//此函数是subscriber的回调函数
void SerialCom::infoExchange(const serial_com::comm::ConstPtr &msg){
    uint8_t buffer[65];
    size_t num = 0;
    // serial_debug("Msg from Frame: x, y, z: %f, %f, %f\n", msg->x, msg->y, msg->z);
    // float _x = 0.0, _y = 0.0, _z = 0.0;
    getBuffer(buffer, msg->x1, msg->y1, 0);                 // 视觉->电控 数据转换
    getBuffer(buffer, msg->x2, msg->y2, 8); 
    getBuffer(buffer, msg->x3, msg->y3, 16); 
    getBuffer(buffer, msg->x4, msg->y4, 24); 
    getBuffer(buffer, msg->x5, msg->y5, 32); 
    getBuffer(buffer, msg->x6, msg->y6, 40); 
    getBuffer(buffer, msg->x7, msg->y7, 48); 
    getBuffer(buffer, msg->x7, msg->y7, 56);
    buffer[64] = msg->color;

    num = ser.write(buffer, 65);
    ros_debug("Sent comm to serial port. %lu bits.", num);
}



//查找需要的串口
int SerialCom::serialOK(char *output){
    std::cout<<"Directory dev/serial exists."<<std::endl;
    DIR *dir = opendir("/dev/serial/by-id");
    if(!dir){                                                   //是否能正常打开目录
        std::cout<<"Open directory error."<<std::endl;
        return -1;
    }
    struct dirent* files;
    while((files = readdir(dir))){                              //查找名称长度大于5的为USB设备
        for(int i = 0; i < 256 && files->d_name[i] > 0; ++i){
            if(i>5){
                strcat(output, files->d_name);                  // 云台板断电，重新给予权限
                std::string command = "echo \"bfjg\" | sudo -S chmod 777 " + std::string(output);
                system(command.c_str());
                return 0;
            }
        }
    }
    return -1;                                                  //所有设备名称不符合要求
}


