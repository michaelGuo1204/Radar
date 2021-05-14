/**============新的MatchLight(明暗交替)============
 * 作者：hqy
 */

#include "Armor/ArmorPlate.hpp"

/// @ref https://wenku.baidu.com/view/0f3c083a172ded630a1cb6c8.html
std::map<int, float> grubbs_length = {
    {3, 1.329}, {4, 2.140}, {5, 2.796}, {6, 3.320}, {7, 3.756}, {8, 4.129}, {9, 4.452}, {10, 4.735}
};

std::map<int, float> grubbs_angle = {
    {3, 1.500}, {4, 2.238}, {5, 3.112}, {6, 3.893}, {7, 4.575}, {8, 5.171}, {9, 5.698}, {10, 6.160}
};

ArmorPlate::ArmorPlate(){
    for(int i=0; i<4 ;++i) points[i] = aim_deps::NULLPOINT2f;
    arm_clf = cv::ml::ANN_MLP::create();
    file_loaded = false;
    ap_mean = 0.0;
    train_cnt = 0;
    const std::string input = "../fifos/mlp_in.pipe";
    const std::string output = "../fifos/mlp_out.pipe";
    // if (access(input.c_str(), F_OK) != 0 || access(output.c_str(), F_OK) != 0) {
    //     printf("Error: broken pipe.\n");
    //     exit(-1);
    // }
    // mkfifo(input.c_str(), 0777);
    // mkfifo(output.c_str(), 0777);
    // in_fd = open(input.c_str(), O_SYNC | O_RDWR);
    // out_fd = open(output.c_str(), O_RDONLY);
}

ArmorPlate::~ArmorPlate(){
    printf("Training for %d trials, average ap is %f\n", train_cnt, ap_mean / float(train_cnt));
}

void ArmorPlate::matchAll(
    const std::vector<int>& car,
    const std::vector<std::pair<int, int> > &matches,
    std::vector<aim_deps::Light> &lights,
    std::vector<aim_deps::Armor> &tar_list
)
{
    tar_list.clear();
    for(size_t i = 0 ; i < matches.size() ; ++i){
        if(isMatch(lights[matches[i].first],lights[matches[i].second])){
            if(lights[matches[i].first].box.center.x < lights[matches[i].second].box.center.x)
            {
                tar_list.emplace_back(aim_deps::Armor(points, 0, lights[matches[i].first], lights[matches[i].second]));
            }
            else
            {
                tar_list.emplace_back(aim_deps::Armor(points, 0, lights[matches[i].second], lights[matches[i].first]));
            }
            amp_debug(rmlog::F_GREEN, "Matched:(", i, "), with matches(", matches[i].first, ", ", matches[i].second,")");
        }
    }
    filter(car, tar_list, lights);                   //过滤无效装甲板
}

void ArmorPlate::matchAll(
    const std::vector<std::pair<int, int> >& matches,
    std::vector<aim_deps::Light> &lights, 
    std::vector<aim_deps::Armor> &tar_list
) {
    transBuffer w_trans;
    memset(w_trans.data, 0, BUFFER_SIZE * sizeof(float));
    size_t total_num = 0, size = (matches.size() / 5 + 1) * 5;
    char results[size];
    for (size_t i = 0; i < matches.size(); i += 5) {
        // 发送过程
        for (size_t j = 0; j < 5; j++) {
            if (i + j >= matches.size()) {       // 超过大小
                memset(&w_trans.data[5 * j], 0.0, 5 * sizeof(float));
            }
            else { 
                printf("(%d, %d), ", matches[i + j].first, matches[i + j].second);
                const aim_deps::Light& left = lights[matches[i + j].first], right = lights[matches[i + j].second];
                w_trans.data[5 * j] = left.box.length;
                w_trans.data[5 * j + 1] = right.box.length;
                cv::Point2f vec = right.box.center - left.box.center;
                float cosa = std::cos(-left.box.angle), sina = std::sin(-left.box.angle);
                w_trans.data[5 * j + 2] = cosa * vec.x - sina * vec.y;
                w_trans.data[5 * j + 3] = sina * vec.x + cosa * vec.y;
                w_trans.data[5 * j + 4] = right.box.angle - left.box.angle;
            }
        }
        printf("\n Sending:");
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                printf("%f, ", w_trans.data[5 * i + j]);
            }
            printf("\n");
        }
        if (write(in_fd, (void *)w_trans.buf, 4 * BUFFER_SIZE) == -1) {
            std::cerr << "No data sent.\n";
        }
        // 接收过程（每次也收五个）
        size_t received = 0;
        while (true) {
            received += read(out_fd, (void * )(results + total_num), 5 - received);
            total_num += received;
            if (received < 5) {
                usleep(10);
                continue;
            }
            printf("\033[34mResult received, light %lu: [", lights.size());
            for (size_t i = 0; i < size; i++) {
                printf("%d, ", results[i]);
            }
            printf("]\n\033[0m");
            break;
        }
        // 接收过程
    }
    for (size_t i = 0; i < matches.size(); i++) {
        printf("Is armor plate: %d, match: %d, %d\n", results[i], matches[i].first, matches[i].second);
        if (results[i] > 0) continue;
        const aim_deps::Light& l1 = lights[matches[i].first];
        const aim_deps::Light& l2 = lights[matches[i].second];
        points[0] = l1.box.vex[0];
        points[1] = l1.box.vex[1];
        points[2] = l2.box.vex[1];
        points[3] = l2.box.vex[0];
        tar_list.emplace_back(points, 0, l1, l2);
        tar_list.back().valid = true;
    }
}

bool ArmorPlate::isMatch(const aim_deps::Light &l1, const aim_deps::Light &l2)
{
    bool judge = true;                                  //灯条角度过大（与x轴成的夹角小）时退出
    if(l1.box.center.x < l2.box.center.x)               //r1灯条在左侧
        judge = getArmorPlate(l1.box, l2.box);
    else
        judge = getArmorPlate(l2.box, l1.box);          //始终保持第一个入参是x轴坐标小的灯条
    return true;
    if(!judge) return false;                        
    if(!isAngleMatch(l1.box.angle, l2.box.angle)){
        amp_debug(rmlog::F_RED, "Angle mismatch:(", l1.index, ", ", l2.index, ")");
        return false;
    }
    // printf("Arm(%d, %d): length_ratio: ", l1.index, l2.index);

    if(isEdgesValid() && isAreaGood()){
        amp_debug(rmlog::F_BLUE, "Push in:(", l1.index, ", ", l2.index, ")");
        return true;
    }
    else{
        amp_debug(rmlog::F_RED, "Ratio mismatch or area too small:(", l1.index, ", ", l2.index, ")");
    }
    return false;
}

void ArmorPlate::drawArmorPlates(cv::Mat &src, 
    const std::vector<aim_deps::Armor>& tar_list, const int optimal) const{
	char str[2];
    cv::line(src, cv::Point(720, 0), cv::Point(720, 1080), cv::Scalar(255, 0, 0));
	cv::line(src, cv::Point(0, 540), cv::Point(1440, 540), cv::Scalar(255, 0, 0));
    for (size_t i = 0; i< tar_list.size(); ++i) {
        if(tar_list[i].armor_number != -1 && tar_list[i].valid){   //有意义的数字
            if((int)i != optimal){       //非最佳装甲板使用黄色绘制
                for (int j = 0; j < 4; ++j){
                    cv::line(src, tar_list[i].vertex[j], 
                    tar_list[i].vertex[(j + 1) % 4], cv::Scalar(0, 255, 255), 1);   
                }
            }
            else{                   //最佳装甲板使用绿色绘制
                for (int j = 0; j < 4; ++j){
                    cv::line(src, tar_list[i].vertex[j],
                    tar_list[i].vertex[(j + 1) % 4], cv::Scalar(0, 255, 0), 1);
                }
            }
            ///snprintf(str, 2, "%d", j);      //最佳装甲板位置x
	        ///cv::putText(src, str, tar_list[i].vertex[j]+cv::Point2f(2, 2),
	        ///    cv::FONT_HERSHEY_PLAIN, 1.1, cv::Scalar(0, 100, 255));
            snprintf(str, 2, "%d", tar_list[i].armor_number);
            cv::putText(src, str, tar_list[i].vertex[2] + cv::Point2f(8, 4),
                cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255));
        }
    }
}

template <bool use_angle>
void ArmorPlate::GrubbsIteration(CarLight& lights) {
    if (lights.size() < 3) return;
    for (size_t k = 0; k < lights.size(); k++) {
        size_t size = 0, max_pos = 0, max_dist = 0;
        float mean = 0.0, var = 0.0;
        for (size_t i = 0; i < lights.size(); i++) {
            if (lights[i]->valid == true) continue;
            mean += use_angle ? lights[i]->box.angle : lights[i]->box.length;
            size ++;
        }
        mean /= float(size);
        float dists[size];
        for (size_t i = 0, cnt = 0; i < lights.size(); i++) {
            if (lights[i]->valid == true) continue;
            float d2 = use_angle ? std::pow(lights[i]->box.angle - mean, 2) : std::pow(lights[i]->box.length - mean, 2);
            if (d2 > max_dist) {
                max_dist = d2;
                max_pos = i;
            }
            dists[cnt] = d2; 
            var += d2;
        }
        var /= (float(size) - 1.0);
        float threshold = use_angle ? grubbs_angle[size] * var : grubbs_length[size] * var;
        if (max_dist > threshold) {
            lights[max_pos]->valid = false;
            if (size - 1 < 3) break;
        }
        else {                              // Grubbs准则没有找到outlier
            break;
        }
    }
}

void ArmorPlate::filter(const std::vector<int> &car, std::vector<aim_deps::Armor> &tar_list, std::vector<aim_deps::Light> &lights){
    std::vector<CarLight> car_lights(_MAX_CAR_NUM);
    std::vector<CarLight> valid_lights(_MAX_CAR_NUM);
    for (size_t i = 0; i < lights.size(); i++) {
        int index = car[i];
        if (index > 0 && lights[i].valid) {
            car_lights[index - 1].emplace_back(&lights[i]);
        }
    }
    #pragma omp parallel for num_threads(4)
    for (size_t i = 0; i < car_lights.size(); i++) {
        GrubbsIteration<true>(car_lights[i]);       // 每辆车进行 长度 outlier detection （长度更加严格）
        GrubbsIteration<false>(car_lights[i]);      // 角度 outlier detection （角度没有那么严格）
        for (const LightPtr& lt: car_lights[i]) {
            if (lt->valid == true) {
                valid_lights[i].emplace_back(lt);
            }
        }
    }
    int match_lut[lights.size()];            // lookup table for matching
    memset(match_lut, -1, sizeof(int) * lights.size());
    for (size_t i = 0; i < valid_lights.size(); i++) {
        const CarLight& lts = valid_lights[i];
        size_t size = lts.size();
        if (size > 4) {                 // 5个及以上灯条
            if (lts.front()->box.length >= lts.back()->box.length) {  // 首灯条更长
                simpleKM(lts, match_lut);
            }
            else {
                float angle_score[3];
                for (int j = 1; j < 4; j++) {
                    float tmp = std::abs(lts[size - j]->box.angle - lts[size - j - 1]->box.angle);
                    angle_score[j - 1] = std::exp(-0.25 * tmp);
                }
                int start_pos = angle_score[0] + angle_score[2] > angle_score[1] + 1.0 ? size - 1 : size - 2;
                for (int j = start_pos; j > 0; j -= 2) {
                    int this_index = lts[j]->index,
                        next_index = lts[j - 1]->index;
                    match_lut[this_index] = next_index;
                    match_lut[next_index] = this_index;
                }
            }
        }
        else if (size == 4) {           // 4个灯条
            simpleKM(lts, match_lut);
        }
        else if (size == 3) {           // 3个灯条
            if (std::abs(lts[0]->box.angle - lts[1]->box.angle) <
                std::abs(lts[1]->box.angle - lts[2]->box.angle)
            ){
                match_lut[lts[0]->index] = lts[1]->index;
                match_lut[lts[1]->index] = lts[0]->index;
            }
            else {
                match_lut[lts[1]->index] = lts[2]->index;
                match_lut[lts[2]->index] = lts[1]->index;
            }
        }
        else if (size == 2) {
            match_lut[lts[0]->index] = lts[1]->index;
            match_lut[lts[1]->index] = lts[0]->index;
        }
    }
    // 根据match_lut进行匹配
    for (aim_deps::Armor& tar: tar_list){
        int left_i = tar.left_light.index, right_i = tar.right_light.index;
        if (match_lut[left_i] != right_i || match_lut[right_i] != left_i) {
            tar.valid = false;
        }
        else {
            tar.valid = true;
            tar.left_light.isLeft = 1;
            lights[left_i].isLeft = 1;
            tar.right_light.isLeft = 0;
            lights[right_i].isLeft = 0;
        }
    }
}

void ArmorPlate::simpleKM(const CarLight& lights, int* lut) {
    float angle_score[3];
    for (int j = 0; j < 3; j++) {
        float tmp = std::abs(lights[j]->box.angle - lights[j + 1]->box.angle);
        angle_score[j] = std::exp(-0.25 * tmp);
    }
    int start_pos = angle_score[0] + angle_score[2] > angle_score[1] + 1.0 ? 0 : 1;
    for (int j = start_pos; j < lights.size() - 1; j+= 2) {
        int this_index = lights[j]->index,
            next_index = lights[j + 1]->index;
        lut[this_index] = next_index;
        lut[next_index] = this_index;
    }
}

bool ArmorPlate::getArmorPlate(const aim_deps::LightBox &b1, const aim_deps::LightBox &b2){
    //lightCompensate(b1, b2);
    points[0] = b1.vex[0];
    points[1] = b1.vex[1];
    points[2] = b2.vex[1];
    points[3] = b2.vex[0];
    cv::Point2f diff = points[0] - points[1];
    /// 这个地方的意思是：需要灯条有合适的角度（cot值必须小于1.5）
    return diff.x/diff.y < 1.5;                 
}

//从最左上角开始的点，逆时针方向标号是0,1,2,3
bool ArmorPlate::isEdgesValid() const{  //对边长度平方比值是否合适
    float edges[4];
    for(int i = 0; i<4; ++i){
        edges[i] = aim_deps::getPointDist(points[i], points[(i+1)%4]);
    }

    bool judge1 = (edges[0]/edges[2] < params.OPS_RATIO_HEIGHT &&
        edges[0]/edges[2] > 1.0 / params.OPS_RATIO_HEIGHT),     //宽对边比值范围大
        judge2 = (edges[1]/edges[3] < params.OPS_RATIO_WIDTH &&
        edges[1]/edges[3] > 1.0 / params.OPS_RATIO_WIDTH);   //长对边比值范围小
    if (judge1 && judge2){
        return true;
    }
    return false;
}

bool ArmorPlate::isAngleMatch(float ang1, float ang2) const{
    //输入按照装甲板上点的顺序: 从左上角开始逆时针,
    // |0        3|
    // |1        2|   
    return (std::abs(ang1-ang2) < params.ANGLE_THRESH);
}

bool ArmorPlate::isAreaGood() const{
    std::vector<cv::Point2f> tmp = {points[0], points[1], points[2], points[3]};
    return cv::contourArea(tmp) >= aim_deps::MIN_ARMOR_AREA;
}

// ======================= MLP ========================
void ArmorPlate::getArmorDataSet(
    const std::vector<aim_deps::Light>& lights,
    const std::vector<std::pair<int, int> >& matches,
    const std::vector<aim_deps::Armor>& tar_list,
    std::vector<std::array<float, 5> >& output,
    std::vector<int>& label
) const {
    for (const std::pair<int, int>& pr: matches) {
        bool break_flag = false;
        for (const aim_deps::Armor& arm: tar_list) {
            if (arm.valid && arm.left_light.index == pr.first && arm.right_light.index == pr.second) {
                printf("True sample pushed back: %d, %d\n", pr.first, pr.second);
                label.emplace_back(1);
                break_flag = true;
                break;
            }
        }
        if (break_flag == false) {
            printf("False sample pushed back: %d, %d\n", pr.first, pr.second);
            label.emplace_back(0);
        }
        std::array<float, 5> arr;
        const aim_deps::Light& left = lights[pr.first], right = lights[pr.second];
        arr[0] = left.box.length;
        arr[1] = right.box.length;
        cv::Point2f vec = right.box.center - left.box.center;
        float cosa = std::cos(-left.box.angle * aim_deps::DEG2RAD), sina = std::sin(-left.box.angle * aim_deps::DEG2RAD);
        arr[2] = cosa * vec.x - sina * vec.y;
        arr[3] = sina * vec.x + cosa * vec.y;
        arr[4] = right.box.angle - left.box.angle;
        output.push_back(arr);
    }
}

bool ArmorPlate::saveToFile(
    std::vector<std::array<float, 5> >& output,
    std::vector<int>& label,
    std::string path, int number) const 
{
    std::ofstream data(path + "data_" + "sentry" + ".csv", std::ios::out | std::ios::binary);
    size_t size = output[0].size();
    for (size_t i = 0; i < output.size(); i++) {
        for (size_t j = 0; j < size ; j++) {
            data << output[i][j] << ',';
        }
        data << label[i] << std::endl;
    }
    data.close();
    std::ofstream info(path + "type.txt", std::ios::out);
    info << output.size() << ',' << size;                         // 输出数据个数 + 数据特征个数
    info.close();
    std::cout << "Training data saved to '" << path << "'\n";
}

void ArmorPlate::initMLP(int hidden_layer) {
    using namespace cv::ml;
    cv::Mat layers(1, 4, CV_32SC1);
    layers.at<int>(0) = _FEAT_NUM;
    layers.at<int>(1) = hidden_layer;
    layers.at<int>(2) = hidden_layer;
    layers.at<int>(3) = 2;               // 输出层
    arm_clf->setLayerSizes(layers);
    arm_clf->setActivationFunction(ANN_MLP::ActivationFunctions::SIGMOID_SYM);
    arm_clf->setTrainMethod(ANN_MLP::TrainingMethods::BACKPROP, 0.1, 0.001);
    arm_clf->setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER, 10000, 1e-6));
}

void ArmorPlate::loadAndTrain(std::string path, std::string opath) {
    if (file_loaded == false) {
        loadFromFile(train_raw_true, train_raw_false, label_raw_true, label_raw_false, path);
        file_loaded = true;
    }
    uint64_t now_time = std::chrono::system_clock().now().time_since_epoch().count();
    std::shuffle(train_raw_true.begin(), train_raw_true.end(), std::default_random_engine(now_time));
    std::shuffle(train_raw_false.begin(), train_raw_false.end(), std::default_random_engine(now_time));
    std::shuffle(label_raw_true.begin(), label_raw_true.end(), std::default_random_engine(now_time));
    std::shuffle(label_raw_false.begin(), label_raw_false.end(), std::default_random_engine(now_time));
    std::vector<std::array<float, _FEAT_NUM> > train_raw;
    std::vector<int> label_raw;
    const size_t origin_size = train_raw_true.size();
    printf("Origin size: %lu\n", train_raw_true.size());
    train_raw.assign(train_raw_true.begin(), train_raw_true.end());
    label_raw.assign(label_raw_true.begin(), label_raw_true.end());
    for (size_t i = 0; i < origin_size; i++) {
        train_raw.emplace_back(train_raw_false[i]);
        label_raw.emplace_back(0);
    }    
    printf("Before re-balance, threre are %lu true samples, and after that %lu\n", origin_size, train_raw.size());

    now_time = std::chrono::system_clock().now().time_since_epoch().count();
    std::shuffle(train_raw.begin(), train_raw.end(), std::default_random_engine(now_time));
    std::shuffle(label_raw.begin(), label_raw.end(), std::default_random_engine(now_time));

    size_t train_num = train_raw.size() * 5 / 6, test_num = train_raw.size() - train_num;
    cv::Mat train_data(train_num, _FEAT_NUM, CV_32FC1);
    cv::Mat train_label(train_num, 2, CV_32FC1);
    #pragma omp parallel for num_threads(8) 
    for (size_t i = 0; i < train_num; i++) {
        float* ptr = train_data.ptr<float>(i);
        for (int j = 0; j < _FEAT_NUM; j++) {
            *(ptr + j) = train_raw[i][j];
        }
        train_label.at<float>(i, 0) = label_raw[i];
        train_label.at<float>(i, 1) = 1 - label_raw[i];
    }

    cv::Mat test_data(test_num, _FEAT_NUM, CV_32FC1);
    cv::Mat test_label(test_num, 2, CV_32FC1);

    #pragma omp parallel for num_threads(8) 
    for (size_t i = 0; i < test_num; i++) {
        float* ptr = test_data.ptr<float>(i);
        size_t index = i + train_num;
        for (int j = 0; j < _FEAT_NUM; j++) {
            *(ptr + j) = train_raw[index][j];
        }
        test_label.at<float>(i, 0) = label_raw[index];
        test_label.at<float>(i, 1) = 1 - label_raw[index];
    }
    cv::Ptr<cv::ml::TrainData> train_set = cv::ml::TrainData::create(train_data, cv::ml::ROW_SAMPLE, train_label);
    bool is_trained = arm_clf->train(train_set);
    if (is_trained && arm_clf->empty() == false) {
        std::cout << "Training completed.\n";
        std::cout << "Testing...\n";
        int cnt = 0;
        for (size_t i = 0; i < test_num; i++) {
            cv::Mat test_mat(1, _FEAT_NUM, CV_32FC1);
            const std::array<float, 5>& arr = train_raw[i + train_num];
            for (int j = 0; j < 5; j++) {
                test_mat.at<float>(j) = arr[j];
            }
            int label = label_raw[i + train_num];
            cv::Mat dst;
            float out = 1 - arm_clf->predict(test_mat, dst);
            if (int(out) == label) cnt++;
        }
        float ap =  float(cnt) / float(test_num);
        printf("(%d trial) Predict AP: %f， is classifier: %d\n", train_cnt, ap, arm_clf->isClassifier());
        ap_mean += ap;
        train_cnt ++;
        arm_clf->save(opath);
        arm_clf->clear();
    }
    else {
        std::cout << "Training failed.\n";
    }
}

void ArmorPlate::loadFromFile(
    std::vector<std::array<float, _FEAT_NUM> >& train_raw_true,
    std::vector<std::array<float, _FEAT_NUM> >& train_raw_false,
    std::vector<int>& label_raw_true,
    std::vector<int>& label_raw_false,
    std::string path
) {
    std::ifstream info(path + "type.txt", std::ios::in);
    if (info.fail()) {
        std::cerr << "Unable to load type.csv from '" << path << "'\n";
        return;
    }
    std::string line;
    getline(info, line);
    std::stringstream ss(line);
    std::string str;
    getline(ss, str, ',');
    int rows = atoi(str.c_str());
    getline(ss, str, ',');
    
    
    std::ifstream data(path + "data.csv", std::ios::in);
    for (int i = 0; i < rows; i++) {
        getline(data, line);
        std::stringstream ss(line);
        std::string str;
        std::array<float, _FEAT_NUM> arr;
        for (int i = 0; i < _FEAT_NUM; i++) {
            getline(ss, str, ',');
            arr[i] = atof(str.c_str());
        }
        getline(ss, str, ',');
        int label = atoi(str.c_str());
        if (label == 1) {
            label_raw_true.emplace_back(1);
            train_raw_true.push_back(arr);
        }
        else {
            label_raw_false.emplace_back(0);
            train_raw_false.push_back(arr);
        }
    }
}