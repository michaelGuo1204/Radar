#include <iostream>
#include<opencv2/opencv.hpp>
#include <vector>
#include "Locationtransform.h"
#include "Gp.hpp"

#include "serial_com/comm.h"
#include "../mapcam/CameraCtl.hpp"
class Radar{
    public:
        LocationTransform trans; // LocationTransform from CameraP to WorldP
        cv::Point CameraP; //2d Position in camera coordinate system
        cv::Point WorldP; // 2d Position in world coordinate system
        cv::Mat gp; // Gaussian result
        cv::Mat frame; //origin mat 
        Gp gmmbg; // Gaussian process
        std::vector<bool> Isblues;
        std::vector<Point> WorldPs; // vector of WorldP
        serial_com::comm g_msg;
        // cm::CameraCtl cap;
        cv::VideoCapture cap;
    public:
        Radar();
        ~Radar(){;}
        void getframe();
        void initframe();

        //use Gaussian to measure car position in camera coordinate system 
        void GmmBackGround();

        void Radarwork();
        void initmsg(); //init g_msg
        void tomsg(); // transform WorldPs and Isblues to g_msg

    
};