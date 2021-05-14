#include<bits/stdc++.h>
#include<opencv2/opencv.hpp>
#include <mapwork/Locationtransform.h>

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

cv::Point2f LocationTransform::group[30];
cv::Mat LocationTransform::img;
void on_mouse(int event,int x,int y,int flags,void* ustc){
     if(event == CV_EVENT_LBUTTONDOWN){
        LocationTransform::group[cnt++]=cv::Point(x,y);
        //std::cout<<x<<" "<<y<<std::endl;
        cv::circle(LocationTransform::img,cv::Point(x,y),2,cv::Scalar(0,0,0),-1);
        cv::imshow("picture",LocationTransform::img);
    }
}
LocationTransform::LocationTransform(){

    std::string picture_path = model_path + "picture.png";
    ColorPicture=cv::imread(picture_path);
    img=cv::imread(picture_path);//TODO:Image should got from cam 
    std::fstream fmatrix(model_path + "Mat.txt",std::ios::in);
    std::fstream f3d(model_path + "3D.txt",std::ios::in);
    for(int i=0;i<7;i++)    for(int j=0;j<3;j++)    for(int k=0;k<3;k++)
        fmatrix>>TransformMatrix[i][j][k];
    for(int i=0;i<28;i++)
        f3d>>team[i].x>>team[i].y>>team[i].z;
}

void LocationTransform::ChangeTheMatrix(){
    cv::namedWindow("picture");
    cv::setMouseCallback("picture",on_mouse);
    cv::imshow("picture",img);
    std::fstream fout("NewMat.txt",std::ios::out);
    while(cnt<28);
    cv::destroyAllWindows();
    for(int i=0;i<7;i++){
        for(int j=0;j<4;j++){
            src[j].x=group[i*4+j].x;
            src[j].y=group[i*4+j].y;
            dst[j].x=team[i*4+j].x;
            dst[j].y=team[i*4+j].y;
        }
        cv::Mat matrix = getPerspectiveTransform(src,dst);
        for(int j=0;j<3;j++){
            for(int k=0;k<3;k++){
                TransformMatrix[i][j][k]=(float)matrix.at<uchar>(k,j);
                fout<<TransformMatrix[i][j][k]<<" ";
            }
            fout<<std::endl;
        }
    }

}
float LocationTransform::AtTheSameFlat(int ThisColor,float x,float y){
    float d1[3],d2[3],dx,dy,dz;
    d1[0]=team[ThisColor*4+1].x-team[ThisColor*4].x;
    d1[1]=team[ThisColor*4+1].y-team[ThisColor*4].y;
    d1[2]=team[ThisColor*4+1].z-team[ThisColor*4].z;
    d2[0]=team[ThisColor*4+2].x-team[ThisColor*4].x;
    d2[1]=team[ThisColor*4+2].y-team[ThisColor*4].y;
    d2[2]=team[ThisColor*4+2].z-team[ThisColor*4].z;
    dx=x-team[ThisColor*4].x;
    dy=y-team[ThisColor*4].y;
    dz=((d1[0]*d2[2]-d1[2]*d2[0])*dy-(d1[1]*d2[2]-d1[2]*d2[1])*dx)/(d1[0]*d2[1]-d1[1]*d2[0]);
    return dz+team[ThisColor*4].z;
}
cv::Point LocationTransform::solve(cv::Point before){
    cv::Point after;
    int ThisColor=-1,b,g,r,x,y;
    x=before.x; y=before.y;
    b=ColorPicture.ptr<uchar>(y,x)[0];
    g=ColorPicture.ptr<uchar>(y,x)[1];
    r=ColorPicture.ptr<uchar>(y,x)[2];
    for(int i=0;i<7;i++)
        if(color[i][0]==b&&color[i][1]==g&&color[i][2]==r){
            ThisColor=i;
            std::cout<<ThisColor<<std::endl;
            break;
        }
    if(ThisColor==-1){
        std::cout<<"It beyond the boundary."<<std::endl;
        return cv::Point(0,0);
    }
    after.x=
    (TransformMatrix[ThisColor][0][0]*x+TransformMatrix[ThisColor][0][1]*y
    +TransformMatrix[ThisColor][0][2])/(TransformMatrix[ThisColor][2][0]*x
    +TransformMatrix[ThisColor][2][1]*y+TransformMatrix[ThisColor][2][2]);
    after.y=
    (TransformMatrix[ThisColor][1][0]*x+TransformMatrix[ThisColor][1][1]*y
    +TransformMatrix[ThisColor][1][2])/(TransformMatrix[ThisColor][2][0]*x
    +TransformMatrix[ThisColor][2][1]*y+TransformMatrix[ThisColor][2][2]);
    // after.x=
    // (TransformMatrix[ThisColor][0][0]*x+TransformMatrix[ThisColor][1][0]*y
    // +TransformMatrix[ThisColor][2][0])/(TransformMatrix[ThisColor][0][2]*x
    // +TransformMatrix[ThisColor][1][2]*y+TransformMatrix[ThisColor][2][2]);
    // after.y=
    // (TransformMatrix[ThisColor][0][1]*x+TransformMatrix[ThisColor][1][1]*y
    // +TransformMatrix[ThisColor][2][1])/(TransformMatrix[ThisColor][0][2]*x
    // +TransformMatrix[ThisColor][1][2]*y+TransformMatrix[ThisColor][2][2]);
    // after.z=AtTheSameFlat(ThisColor,after.x,after.y);
    return after;
}

// 蓝色:	204 72 63
// 3.0052	4.6718	 0.054631
// 2.8356	 4.836	 0.054635
// 3.0013 	2.758	 0.054629
// 2.8326	 2.5909	 0.054628

// 红色:	36 28 236
// 4.4392	1.705	0.25463
// 4.1542	2.1111	0.25463
// 5.2785	6.9278	0.25463
// 4.7339	1.2818	0.25463

// 紫色:	185 62 183
// 6.2346	6.2692	0.054632
// 6.6695	6.8922	0.15421
// 5.4856	6.2702	0.054684
// 5.7094	6.8954	0.15513

// 黄色:	0 242 255
// 5.4369	0.58826	0.47963
// 5.4549	0.84711	0.47995
// 5.2982	1.071	0.48021
// 4.6629	1.0688	0.47963

// 粉色:	200 174 255
// 6.7865	5.8073 	0.35463
// 5.9253	4.557	0.35463
// 5.923	3.5738	0.35463
// 6.2702	3.0782	0.35463

// 橙色:	39 127 255
// 8.6173	4.9612	0.099634
// 9.4313	3.7879	0.099635
// 9.4354	2.9453	0.099635
// 9.1049	2.4687	0.099633

// 青色:	251 255 140
// 7.72	6.8934	0.15464
// 8.3097	6.9	0.15464
// 8.3181	7.4638	0.15464
// 6.7162	6.9364	0.15464