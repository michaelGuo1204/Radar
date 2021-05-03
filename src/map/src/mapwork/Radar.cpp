#include "mapwork/Radar.hpp"

Radar::Radar()
{
    initmsg();
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

void Radar::initframe()
{
    // int read = cap.startGrabbing();
    // if (read)
    // {
    // 	std::cout << "Read video failed!" << std::endl;
    //     return;
    // }
    // frame = cap.getOpencvMat();
    // if (frame.empty()){
    // 	std::cout << "empty img!" << std::endl;
    //     return;
    // }

    cap.open("/home/cv_output106.avi");
    cap >> frame;
    if (frame.empty())
    {
        std::cout << "empty img!1" << std::endl;
        return;
    }

}

void Radar::getframe()
{
    // frame = cap.getOpencvMat();
    cap >> frame;

    if (frame.empty())
    {
        std::cout << "empty img!" << std::endl;
        return;
    }
}

void Radar::GmmBackGround()
{
	waitKey(10);
    gmmbg.Gpmain(frame,gp);
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
    g_msg.x6 = WorldPs[5].x;
    g_msg.y6 = WorldPs[5].y;
    g_msg.x7 = WorldPs[6].x;
    g_msg.y7 = WorldPs[6].y;
    g_msg.x8 = WorldPs[7].x;
    g_msg.y8 = WorldPs[7].y;
    uint8_t k;
    for (uint8_t i = 1, j = 0; j < 8; i = i << 1, j++)
    {
        if (Isblues[j])
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
    Isblues.clear();
}

void Radar::Radarwork()
{
    GmmBackGround();
    cv::Mat gp_copy = gp.clone();
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(gp_copy, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat background = cv::Mat::zeros(cv::Size(gp_copy.cols, gp_copy.rows),CV_8U);
    // printf("%d %d\n",gp_copy.rows, gp_copy.cols);
    // (gp_copy.rows, gp_copy.cols, CV_8UC1, cv::Scalar(0));
    for (int i = 0; i < contours.size(); i++)
    {
        float area = cv::contourArea(contours[i]);
        if (area > 1000)
        {
            cv::Rect bbox = cv::boundingRect(contours[i]);
            //Point center((bbox.tl().x))
            bbox.height *= 1.2;
            bbox.width *= 1.2;
            cv::rectangle(background, bbox, cv::Scalar(255), 1, cv::LINE_8, 0);
            std::vector<Point> bbcontour;
            bbcontour.push_back(bbox.tl());
            bbcontour.push_back(Point(bbox.tl().x + bbox.width, bbox.tl().y));
            bbcontour.push_back(Point(bbox.tl().x + bbox.width, bbox.tl().y + bbox.height));
            bbcontour.push_back(Point(bbox.tl().x, bbox.tl().y + bbox.height));
            cv::fillConvexPoly(background, bbcontour, cv::Scalar(255));
            //cv::Point center((bbox.tl().x + bbox.br().x)/2,(bbox.tl().y + bbox.br().y)/2);
            //cv::circle(orig_img, center, 5, cv::Scalar(0, 255, 0), -1);
        }
    }
    cv::imshow("bbc", background);
    // waitKey(10);
    // std::vector<std::vector<cv::Point>> BCcontours;
    // std::vector<cv::Vec4i> BChierarchy;
    // cv::findContours(background, BCcontours, BChierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point());
    // for (int i = 0; i < BCcontours.size(); i++)
    // {
    //     float area = cv::contourArea(BCcontours[i]);
    //     if (area > 1000)
    //     {
    //         cv::Rect bbox = cv::boundingRect(BCcontours[i]);
    //         cv::rectangle(gp_copy, bbox, cv::Scalar(255, 0, 0), 1, cv::LINE_8, 0);
    //         cv::Point center((bbox.tl().x + bbox.br().x) / 2, (bbox.tl().y + bbox.br().y) / 2);
    //         cv::circle(gp_copy, center, 5, cv::Scalar(0, 255, 0), -1);
    //         cv::Mat roi = frame(bbox);
    //         int bluesum = 0;
    //         int redsum = 0;
    //         bool isblue;
    //         for (int i = 0; i < roi.rows; i++)
    //         {
    //             for (int j = 0; j < roi.cols; j++)
    //             {
    //                 bluesum += roi.at<cv::Vec3b>(i, j)[0];
    //                 redsum += roi.at<cv::Vec3b>(i, j)[2];
    //             }
    //         }
    //         if (bluesum > redsum)
    //         {
    //             isblue = true;
    //         }
    //         else
    //         {
    //             isblue = false;
    //         }

    //         WorldPs.push_back(trans.solve(center));
    //         Isblues.push_back(isblue);
    //     }
    // }
    // cv::imshow("s",orig_img);
}