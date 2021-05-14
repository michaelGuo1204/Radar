#pragma once
#include <iostream>
#include<opencv2/opencv.hpp>
#include <vector>
#include "Locationtransform.h"
#include "../background/background.hpp"
#include "serial_com/comm.h"
#include "../mapcam/CameraCtl.hpp"
#include "../Armor/LightMatch.hpp"
#include "../Armor/ArmorPlate.hpp"
class Radar{
    public:
        LocationTransform trans; // LocationTransform from CameraP to WorldP
        cv::Point CameraP; //2d Position in camera coordinate system
        cv::Point WorldP; // 2d Position in world coordinate system
        cv::Mat gp; // Gaussian result
        cv::Mat frame; //origin mat 
        Background* backg;
        std::vector<int> cars;
        std::vector<Point> WorldPs; // vector of WorldP
        serial_com::comm g_msg;
        // cm::CameraCtl cap;
        cv::VideoCapture cap;
        LightMatch mch;
        ArmorPlate amp;
    public:
        Radar();
        ~Radar(){;}
        void getframe();
        void initframe();
        void Radarwork();
        void initmsg(); //init g_msg
        void tomsg(); // transform WorldPs and Isblues to g_msg

    
};