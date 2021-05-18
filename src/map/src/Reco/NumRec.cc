#include "Reco/NumRec.hpp"


NumRec::NumRec()
{
    std::string model_path="/home/bili/Radar/model/";
    std::string pca_path = model_path + "pca.xml";
    std::string knn_path = model_path + "knn.xml";
     cv::FileStorage fs(pca_path, cv::FileStorage::READ);                        //get pca models
     _pca_model.read(fs.root());
     fs.release();
     knn = cv::Algorithm::load<cv::ml::KNearest>(knn_path);  
     img_cnt = 0;                                                                             //get knn models
}

void NumRec::reco(aim_deps::Armor &Input_armor, const cv::Mat &grayImg, int type){
    cv::Point2f pts[4];
    getExtendedVex(Input_armor.left_light.box, Input_armor.right_light.box, pts);
    const cv::Point2f&
    tl = pts[0],
	bl = pts[1],
	br = pts[2], 
	tr = pts[3]; 

	int width, height;
	if(!Input_armor.Isbigarmor){
	    width = 50;
	    height = 50;
	}
    else {
	    width = 90;
	    height = 50;
	}
	cv::Point2f src[4]{cv::Vec2f(tl), cv::Vec2f(tr), cv::Vec2f(br), cv::Vec2f(bl)};//实际矩形
	cv::Point2f dst[4]{cv::Point2f(0.0, 0.0), cv::Point2f(width, 0.0), cv::Point2f(width, height), cv::Point2f(0.0, height)};//目标矩形
	const cv::Mat perspMat = getPerspectiveTransform(src, dst);//变换
	cv::warpPerspective(grayImg, _transformed_Img, perspMat, cv::Size(width, height));//获取矫正图像
    threshold(_transformed_Img, _transformed_Img, 6, 255, cv::THRESH_BINARY);//阈值化，准备识别
    ///imwrite(str, _transformed_Img);
    //std::cout<<width<<std::endl;
	cv::resize(_transformed_Img, _transformed_Img, cv::Size(width, height), 0, 0, cv::INTER_AREA);//防止可能的bug
    if(Input_armor.Isbigarmor)
	{
		_transformed_Img=_transformed_Img(cv::Rect(20,0,50,50));
	}
    cv::normalize(_transformed_Img, _transformed_Img, 1., 0., cv::NormTypes::NORM_MINMAX, CV_32FC1);//归一化一下
    //cv::imshow("a",_transformed_Img);cv::waitKey(0);
    cv::Mat Projection=_pca_model.eigenvectors*_transformed_Img.reshape(0, 1).t();//求出图片的投影向量
	cv::Mat _cache = _pca_model.project(_transformed_Img.reshape(0, 1));
    
    float r  = knn->predict(_cache);//预测
    //////////////////
    Input_armor.armor_number = (int)r;
     
}

void NumRec::getExtendedVex(
    aim_deps::LightBox l1,
    aim_deps::LightBox l2,
    cv::Point2f pts[]
) const 
{
    l1.extend(2.0);
    l2.extend(2.0);
    pts[0] = l1.vex[0];
    pts[1] = l1.vex[1];
    pts[2] = l2.vex[1];
    pts[3] = l2.vex[0];
}