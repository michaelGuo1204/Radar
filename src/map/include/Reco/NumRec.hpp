/**=============Number Reccognition============
 * @author GQR
 * @date 6/2/2020
 * modified version by hqy
 * 最后修改 5.1.2020 删除了部分函数，修改了某些参数定义
*/


#include <cstdlib>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <torch/torch.h>
#include <torch/script.h>
#include "../Armor/AimDeps.hpp"

class NumRec{
private:                                            //Params
     torch::jit::script::Module cnn;                //Cnn model to recognize the aim_deps::Armor
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
    float Classfier(cv::Mat &image);
public:
     NumRec();
     ~NumRec(){} 
     ///@param:all the armors waiting for evaluation
     ///@param:the whole image (COLOR_BGR2GRAY)
     void reco(aim_deps::Armor &Input_armor, const cv::Mat &grayImg, int type=0);               //recognize the number
};
