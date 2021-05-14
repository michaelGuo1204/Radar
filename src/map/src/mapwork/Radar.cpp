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
    //Initiate the background detection algorithm
    backg=new Background(frame.cols,frame.rows);
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
    g_msg.x1 = WorldPs[0].x;
    g_msg.y1 = WorldPs[0].y;
    g_msg.x2 = WorldPs[1].x;
    g_msg.y2 = WorldPs[1].y;
    g_msg.x3 = WorldPs[2].x;
    g_msg.y3 = WorldPs[2].y;
    g_msg.x4 = WorldPs[3].x;
    g_msg.y4 = WorldPs[3].y;
    g_msg.x5 = WorldPs[4].x;
    g_msg.y5 = WorldPs[4].y;
    uint8_t k;
    for (uint8_t i = 1, j = 0; j < 8; i = i << 1, j++)
    {
        if (cars[j])
        {
            g_msg.color |= i;
        }
        else
        {
            k = ~i;
            g_msg.color &= k;
        }
    }
    WorldPs.clear();
    cars.clear();
}

void Radar::Radarwork()
{   
    cv::Mat ba_detect;
    cv::resize(frame,ba_detect,frame.size()/4);
    backg->backgroundProcess(ba_detect,gp);
    cv::Mat gp_copy = gp.clone();
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Rect> box;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(gp_copy, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    //cv::Mat background = cv::Mat::zeros(cv::Size(gp_copy.cols, gp_copy.rows),CV_8U);
    for (int i = 0; i < contours.size(); i++)
    {
        float area = cv::contourArea(contours[i]);
        if (area > 10)
        {
            cv::Rect bbox = cv::boundingRect(contours[i]);
            //Point center((bbox.tl().x))
            bbox.height *= 1.2;
            bbox.width *= 2;
            cv::rectangle(gp, bbox, aim_deps::RED, 5, cv::LINE_8, 0);
            box.push_back(bbox);
            //cv::fillConvexPoly(gp, bbox, cv::Scalar(255));
            //cv::Point center((bbox.tl().x + bbox.br().x)/2,(bbox.tl().y + bbox.br().y)/2);
            //cv::circle(orig_img, center, 5, cv::Scalar(0, 255, 0), -1);
        }
    }
    std::cout<<box.size()<<std::endl;
    cv::groupRectangles(box,1,1);
    std::cout<<box.size()<<std::endl;
    for(auto con:box)
    {
        cv::Mat roi=frame(con);
        mch.saveImg(roi);
        mch.findPossible();
        std::vector<int> cars(mch.possibles.size());
        for (size_t i = 0; i < cars.size(); i++) {
            cars[i] = 1;
        }
        std::vector<aim_deps::Armor> tar_list;
        amp.matchAll(cars, mch.matches, mch.possibles, tar_list);
        for(auto tar:tar_list){
            WorldPs.push_back(trans.solve(Point2f(con.tl())+tar.vertex[0]));
            cars.push_back(tar.armor_number);
        }
        amp.drawArmorPlates(roi, tar_list, 0);
        mch.drawLights(roi);
        cv::imshow("disp", roi);
        char key = cv::waitKey(0);
        if (key == 27) {
            return;
        }
    }
    cv::imshow("gp", gp);
        char key = cv::waitKey(0);
        if (key == 27) {
            return;
        }
    
    
    
}