#include<bits/stdc++.h>
#include<opencv2/opencv.hpp>
#include <mapwork/Path.hpp>
class LocationTransform{
        static cv::Mat img;
        static cv::Point2f group[30];
        cv::Mat ColorPicture;
        float TransformMatrix[7][3][3];
        cv::Point3f team[30];
        cv::Point2f src[4],dst[4];
        int cnt;
        int color[7][3]={
            204,72,63,//blue
            36,28,236,//red
            185,62,183,//purple
            0,242,255,//yellow
            200,174,255,//pink
            39,127,255,//orange
            251,255,140//ching
};

    public:
        LocationTransform();
        void ChangeTheMatrix();
        cv::Point solve(cv::Point before);
        float AtTheSameFlat(int ThisColor,float x,float y);
        friend void on_mouse(int event,int x,int y,int flags,void* ustc);
};