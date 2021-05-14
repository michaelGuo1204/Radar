#include"background/background.hpp"

BinCount::BinCount(uint cols, uint rows, uint time):
		time(time) {
		map.resize(cols * rows);
		map_max.resize(cols * rows);
	}

void BinCount::run(const cv::Mat& frame, cv::Mat& bg_frame_out, cv::Mat& bg_count_out) {
	queue.push(frame);
	cv::Mat bg_frame2;
	frame.copyTo(bg_frame2);
	cv::Mat bg_count(frame.size(), CV_8UC1);
	cv::Mat pop_frame = queue.front();

	for (int i=0; i<frame.rows; i++) {
		for (int j=0; j<frame.cols; j++) {
			if (true) {
				uint32_t c = zip_rgb(frame.at<cv::Vec3b>(i, j));
				uint8_t& map_c = map[i*frame.cols + j][c];
				map_c += 1;
				std::pair<uint32_t, uint8_t>& maxc = map_max[i*frame.cols + j];
				if (maxc.second < map_c) {
					maxc.second = map_c;
					maxc.first = c;
				}
			}
			if (queue.size() > time) {
				uint32_t c = zip_rgb(pop_frame.at<cv::Vec3b>(i, j));
				uint8_t& map_c = map[i*frame.cols + j][c];
				map_c -= 1;
				if (map_c == 0)
					map[i*frame.cols + j].erase(c);
				std::pair<uint32_t, uint8_t>& maxc = map_max[i*frame.cols + j];
				if (maxc.first == c) {
					maxc.second -= 1;
					const auto& mapij = map[i*frame.cols + j];
					for (const auto& mapijk : mapij) {
						if (mapijk.second > maxc.second) {
							maxc = mapijk;
						}
					}
				}
			}
			const std::pair<uint32_t, uint8_t>& max_color = map_max[i*frame.cols + j];
			bg_frame2.at<cv::Vec3b>(i, j) = unzip_rgb(max_color.first);
			bg_count.at<uint8_t>(i, j) = max_color.second;
		}
	}

	if (queue.size() > time)
		queue.pop();
	bg_frame2.copyTo(bg_frame_out);
	bg_count.copyTo(bg_count_out);
}


Background::Background(int frame_cols,int frame_rows){
    this->bc1=new BinCount(frame_cols,frame_rows,8);
    this->bc2=new BinCount(frame_cols,frame_rows,8);
    this->count=0;
}

void Background::backgroundProcess(cv::Mat frame,cv::Mat& frontend){
    bc1->run(frame, bg_frame, bg_count);
	/*if (count > 64) {
		for (int i=0; i<frame.rows; i++) {
			for (int j=0; j<frame.cols; j++) {
				if (bg_count1.at<uint8_t>(i, j) < 4)
					bg_frame1.at<cv::Vec3b>(i, j) = bg_frame2.at<cv::Vec3b>(i, j);
				}
			}
		}
	if (count % 8 == 0) {
		bc2->run(bg_frame1, bg_frame2, bg_count2);
	}
    cv::imshow("frame", frame);
    cv::imshow("bg1", bg_frame1);
    cv::imshow("bg2", bg_frame2);
    cv::imshow("count", bg_count1 < 4);*/
    frontend=frame - bg_frame;
    count++;
    cv::cvtColor(frontend, frontend,CV_RGB2GRAY);
    cv::threshold(frontend, frontend, 50, 255.0, cv::THRESH_BINARY);
    cv::Mat kernel_e = getStructuringElement(cv::MORPH_RECT,cv::Size(10,10),cv::Point(3,3));
	cv::Mat kernel_d = getStructuringElement(cv::MORPH_RECT,cv::Size(10,10),cv::Point(3,3));
	cv::dilate(frontend,frontend,kernel_d);
	//cv::erode(frontend,frontend,kernel_e);
}