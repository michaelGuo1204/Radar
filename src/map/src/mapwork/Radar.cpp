#include "mapwork/Radar.hpp"

Radar::Radar()
{
    //Initiate the stream
    cap.open("/home/bili/cv_output110.avi");
    cap >> frame;
    if (frame.empty())
    {
        std::cout << "Unable to open source" << std::endl;
        return;
    }
    //Initiate the msg
    initmsg();
    //Set the enemy's color for reco
    mch.setEnemyColor(false,80,60);
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
    g_msg.x6 = 0;
    g_msg.y6 = 0;
    g_msg.x7 = 0;
    g_msg.y7 = 0;
    g_msg.x8 = 0;
    g_msg.y8 = 0;
    g_msg.color = 15; //default blue
}


void Radar::getframe()
{

    cap >> frame;

    if (frame.empty())
    {
        std::cout << "Unable to get frame from stream" << std::endl;
        return;
    }
}


void Radar::tomsg()
{
    std::vector<int> chooser(1,5); 
    std::vector<cv::Point> pointer;
    std::cout << cars[0] << std::endl;
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
    WorldPs.clear();
    cars.clear();
}

void Radar::Radarwork()
{       
    
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
            cars.push_back(tar.armor_number);
        }
    }
    amp.drawArmorPlates(frame, tar_list, 0);
    mch.drawLights(frame);
    cv::imshow("disp", frame);
    char key = cv::waitKey(0);
    if (key == 27) {
        return;
    }
}