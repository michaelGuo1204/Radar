/**=============Number Reccognition============
 * @author GQR
 * @date 6/2/2020
 * modified version by hqy
 * 最后修改 5.1.2020 删除了部分函数，修改了某些参数定义
*/



#ifndef _NUM_REC_HPP
#define _NUM_REC_HPP

#include <cstdlib>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ml.hpp>
#include "Armor/AimDeps.hpp"

class NumRec{
private:                                            //Params
     cv::PCA _pca_model;                            //PCA_model to recognize the aim_deps::Armor
     cv::Ptr<cv::ml::KNearest> knn;                 //Knn model to recognize the aim_deps::Armor
     cv::Mat _transformed_Img;                      //the Image of the aim_deps::Armor
     //===============temporary==============//
     int img_cnt;
     char str[48];
private:
    /**
     * @biref 获得放大的装甲板点数组，放大灯条，以获得更大区域的切割图像
     * @param r1 灯条1(left_light)
     * @param r2 灯条2(right_light)
     * @param pts 装甲板点数组（输出）
     */
    inline void getExtendedVex(aim_deps::LightBox l1, aim_deps::LightBox l2, cv::Point2f pts[]) const;
public:
     NumRec();
     ~NumRec(){ knn->clear(); } 
     ///@param:all the armors waiting for evaluation
     ///@param:the whole image (COLOR_BGR2GRAY)
     void reco(aim_deps::Armor &Input_armor, const cv::Mat &grayImg, int type=0);               //recognize the number
};
#endif //_NUM_REC_HPP