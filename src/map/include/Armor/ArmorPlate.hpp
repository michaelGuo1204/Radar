/**============新的MatchLight(明暗交替)============
 * 作者：hqy
 * 创建时间：2020.2.9 00:27
 * 主要思路：设置几个判断条件：（2020.1.18日写）
 * 9/2/2020 修改：
 *      drawArmorPlate需要通过数字判断其是否valid
 */
#ifndef _ARMOR_PLATE_HPP
#define _ARMOR_PLATE_HPP

#include <sys/stat.h>
#include <sys/types.h>
#include <map>
#include <array>
#include <random>
#include <vector>
#include <chrono>
#include <sstream>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ml.hpp>
#include "AimDeps.hpp"
// #define ARMORPLATE_DEBUG     // ArmorPlate   装甲板匹配模块       装甲板配对信息显示
#ifdef ARMORPLATE_DEBUG
    #define amp_debug rmlog::LOG::printc        //彩色输出
#else
    #define amp_debug(...)
#endif
#define _MAX_CAR_NUM 2          // 最大车数
#define _FEAT_NUM 5             // MLP特征维度
#define BUFFER_SIZE 25

typedef aim_deps::Light* LightPtr;
typedef std::vector<LightPtr> CarLight;

union transBuffer{
    float data[BUFFER_SIZE];
    char buf[4 * BUFFER_SIZE];
};

class ArmorPlate{
public:
    ArmorPlate();         
    ~ArmorPlate();
public:

    /**
     * @brief 根据与匹配情况匹配所有的灯条
     * @param matches cv::Point2f 与匹配对，两个值是两个灯条在lights中的索引
     * @param lights 灯条容器，由LightMatch类传入
     * @param tar_list 入参/输出，由更高层的AimDistance类传入
     */
    void matchAll(
        const std::vector<int>& car,
        const std::vector<std::pair<int, int> >& matches,
        std::vector<aim_deps::Light> &lights, 
        std::vector<aim_deps::Armor> &tar_list);

    void matchAll(
        const std::vector<std::pair<int, int> >& matches,
        std::vector<aim_deps::Light> &lights, 
        std::vector<aim_deps::Armor> &tar_list);

    void infoExchange(
        const std::vector<std::pair<int, int> >& matches, 
        const std::vector<aim_deps::Light> &lights
    );
    
    /**
     * @brief 绘制装甲板于图像
     * @param src 输入/输出,用于绘制的图像
     * @param tar_list 装甲板列表
     * @param optimal 最优装甲板的位置(使用绿色绘制)
     */
    void drawArmorPlates(cv::Mat &src, 
        const std::vector<aim_deps::Armor>& tar_list, const int optimal) const;                                            //消息发布

    // ========================= MLP 相关 =============================
    void getArmorDataSet(
        const std::vector<aim_deps::Light>& lights,
        const std::vector<std::pair<int, int> >& matches,
        const std::vector<aim_deps::Armor>& tar_List,
        std::vector<std::array<float, 5> >& output,
        std::vector<int>& label
    ) const;                                       //装甲板数据集   

    bool saveToFile(
        std::vector<std::array<float, 5> >& output,
        std::vector<int>& label,
        std::string path = "../data/", int number = 1
    ) const;

    void initMLP(int hidden_layer = 8);

    void loadAndTrain(std::string path = "../data/", std::string opath = "../model/mlp.xml");

    static void loadFromFile(
        std::vector<std::array<float, _FEAT_NUM> >& train_raw_true,
        std::vector<std::array<float, _FEAT_NUM> >& train_raw_false,
        std::vector<int>& label_raw_true,
        std::vector<int>& label_raw_false,
        std::string path = "../data/"
    );
private:
    /**
     * @brief 对两个灯条的RotatedRect进行匹配，直接在tar_list中emplace_back符合条件的装甲板
     * @param l1 灯条1
     * @param l2 灯条2
    */
    bool isMatch(const aim_deps::Light &l1, const aim_deps::Light &l2);
    bool getArmorPlate(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2);         //灯条匹配装甲板

    void filter(
        const std::vector<int> &car,
        std::vector<aim_deps::Armor> &tar_list,
        std::vector<aim_deps::Light> &lights
    );                      //进一步过滤装甲板容器

    // 2v2 简单完全二分图最大权算法
    void simpleKM(const CarLight& lights, int* lut);

    template <bool use_angle = false>
    void GrubbsIteration(CarLight& lights);

    bool isEdgesValid() const;                                          //两对边(平方)的比是否合适
    bool isAreaGood() const;                                            //面积是否正确：面积过小的装甲板将会被过滤
    inline bool isAngleMatch(float ang1, float ang2) const;             
private:
    bool _is_enemy_blue;                                                //敌人颜色
    cv::Point2f points[4];                                              //装甲板点列的临时容器
    aim_deps::Distance_Params params;                                   //装甲板匹配参数

    // ===========================  MLP ===========================
    cv::Ptr<cv::ml::ANN_MLP> arm_clf;                                   //判定是否是armor的MLP分类器
    std::vector<std::array<float, _FEAT_NUM> > train_raw_true;
    std::vector<std::array<float, _FEAT_NUM> > train_raw_false;
    std::vector<int> label_raw_true;
    std::vector<int> label_raw_false;
    bool file_loaded;
    float ap_mean;
    int train_cnt;

    int in_fd;
    int out_fd;
};

#endif     //_ARMOR_PLATE_HPP