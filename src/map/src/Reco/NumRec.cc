#include "Reco/NumRec.hpp"

NumRec::NumRec() {
    std::string cnn_path = "/home/bili/Radar/model/cnn.pt";
    cnn = torch::jit::load(cnn_path);
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
	    width = 32;
	    height = 32;
	}
    else {
	    width = 32;
	    height = 32;
	}
	cv::Point2f src[4]{cv::Vec2f(tl), cv::Vec2f(tr), cv::Vec2f(br), cv::Vec2f(bl)};//实际矩形
	cv::Point2f dst[4]{cv::Point2f(0.0, 0.0), cv::Point2f(width, 0.0), cv::Point2f(width, height), cv::Point2f(0.0, height)};//目标矩形
	const cv::Mat perspMat = cv::getPerspectiveTransform(src, dst);//变换
    cv::Mat _transformed_Img;                      //the Image of the aim_deps::Armor
	cv::warpPerspective(grayImg, _transformed_Img, perspMat, cv::Size(width, height));//获取矫正图像
    //如果要输出结果
    ///imwrite(str, _transformed_Img);
    //std::cout<<width<<std::endl;
    cv::threshold(_transformed_Img, _transformed_Img, 40, 255, cv::THRESH_TOZERO_INV);
    cv::threshold(_transformed_Img, _transformed_Img, 1, 255, cv::THRESH_BINARY | cv::THRESH_OTSU); //阈值化，准备识别

	cv::resize(_transformed_Img, _transformed_Img, cv::Size(width, height), 0, 0, cv::INTER_AREA); //防止可能的bug
    // cv::imshow("a",_transformed_Img);
    // cv::waitKey(0);

    float r = Classfier(_transformed_Img);
    Input_armor.armor_number = std::round(r);
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

float NumRec::Classfier(cv::Mat &image){
    torch::Tensor img_tensor = torch::from_blob(image.data, {1, image.rows, image.cols, 1}, torch::kByte);
    img_tensor = img_tensor.permute({0, 3, 1, 2});
    img_tensor = img_tensor.toType(torch::kDouble);
    img_tensor = img_tensor.div(255);
    torch::Tensor output = cnn.forward({img_tensor}).toTensor();
    auto max_result = output.max(1, true);
    return std::get<1>(max_result).item<float>();
}