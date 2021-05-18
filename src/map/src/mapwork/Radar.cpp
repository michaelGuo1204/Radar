#include "mapwork/Radar.hpp"

Radar::Radar()
{
    //Initiate the stream
    //cap.open("/home/bili/cv_output4065.avi");
    cap.startGrabbing();
    frame=cap.getOpencvMat();
    if (frame.empty())
    {
        std::cout << "Unable to open source" << std::endl;
        return;
    }
    //Initiate the msg
    initmsg();
    //Set the enemy's color for reco
    mch.setEnemyColor(true,80,60);
}

void Radar::initmsg()
{
    g_msg.x1 = 0;
    g_msg.y1 = 0;
    g_msg.x2 = 0;
    g_msg.y2 = 0;
    g_msg.x3 = 0;
    g_msg.y3 = 0;
    g_msg.x4 = 0;
    g_msg.y4 = 0;
    g_msg.x5 = 0;
    g_msg.y5 = 0;
}


void Radar::getframe()
{
    if(isHigh){
        cap.setExposureTime(30000);
    }else{
        cap.setExposureTime(400);
    }
    frame=cap.getOpencvMat();
    if (frame.empty()){
        std::cout << "Unable to get frame from stream" << std::endl;
        return;
        }
    
}


void Radar::tomsg()
{
    std::vector<int> chooser(10,1); 
    std::vector<cv::Point> pointer(10);
    //std::cout << pointer.size() << std::endl;
    for(size_t i = 0; i < WorldPs.size(); i++){
        if(!chooser[cars[i]]){
            ++chooser[cars[i]];
            pointer[cars[i]]=WorldPs[i];
        }else{
            pointer[cars[i]]=(WorldPs[i]+pointer[cars[i]])/2;
        }
    }
    g_msg.x1 = pointer[4].x;
    g_msg.y1 = pointer[4].y;
    g_msg.x2 = pointer[3].x;
    g_msg.y2 = pointer[3].y;
    g_msg.x3 = pointer[2].x;
    g_msg.y3 = pointer[2].y;
    g_msg.x4 = pointer[1].x;
    g_msg.y4 = pointer[1].y;
    g_msg.x5 = pointer[0].x;
    g_msg.y5 = pointer[0].y;
    pointer.clear();
    WorldPs.clear();
    cars.clear();
}

void Radar::Radarwork()
{       
    if(!isLowExposure(frame)){
        cv::imshow("high", frame);
        frame.release();
        char key = cv::waitKey(1);
        // if (key == 27) {
        // return;
        // }
    }else{
    mch.saveImg(frame); 
    mch.findPossible();
    std::vector<int> possible_cars(mch.possibles.size());
    for (size_t i = 0; i < possible_cars.size(); i++) {
        possible_cars[i] = 1;
    }
    std::vector<aim_deps::Armor> tar_list;
    amp.matchAll(possible_cars, mch.matches, mch.possibles, tar_list);
    
    if(tar_list.size()){
        cv::Mat gray_img;
        cv::cvtColor(frame, gray_img, cv::COLOR_BGR2GRAY);
        for(int i=0;i<tar_list.size();i++) {
            tar_list[i].armor_number = -1;
        }
        for(auto tar:tar_list){
            WorldPs.push_back(trans.solve(tar.vertex[0]));
            recog.reco(tar, gray_img);
            std::cout << tar.armor_number << std::endl;
            cars.push_back(tar.armor_number);
        }
    }
    // amp.drawArmorPlates(frame, tar_list, 0);
    // mch.drawLights(frame);
    // cv::imshow("disp", frame);
    // frame.release();
    // char key = cv::waitKey(100);
    //     if (key == 27) {
    //     return;
    //     }
    }
    isHigh=!isHigh;
}
bool Radar::isLowExposure(cv::Mat &src) {
	cv::Scalar mean_values;
    cv::Mat gray;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    int row_sampling = 16;
    int col_sampling = 16;
    int sampling_sum = 0;
    for(int i=0;i<row_sampling;++i){
        for(int j=0;j<col_sampling;++j){
            int pix_value = gray.at<uchar>(gray.rows*0.1+i*gray.rows*0.8/row_sampling, gray.cols*0.1+j*gray.cols*0.8/col_sampling);
            if(pix_value>30) ++sampling_sum;
            // cv::Point2f smp = cv::Point2f(gray.cols*0.1+i*gray.cols*0.8/col_sampling, gray.rows*0.1+j*gray.rows*0.8/row_sampling);
            // cv::circle(src, smp, 3, cv::Scalar(0, 0, 255), -1);
        }
    }
    if(sampling_sum < row_sampling*col_sampling*0.3)
	    return true;
    else
        return false;
}
