#pragma once
#include <iostream>
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


class BinCount {
private:
	inline uint32_t zip_rgb(cv::Vec3b c) {
		uint32_t zip = (c[0] & 0xf8) << 7;
		zip |= (c[1] & 0xf8) << 2;
		zip |= (c[2] & 0xf8) >> 3;
		return zip;
	}

	inline cv::Vec3b unzip_rgb(uint32_t c) {
		return cv::Vec3b((c >> 7) & 0xf8, (c >> 2) & 0xf8, (c << 3) & 0xf8);
	}

	std::queue<cv::Mat> queue;
	std::vector<std::unordered_map<uint32_t, uint8_t>> map;
	std::vector<std::pair<uint32_t, uint8_t>> map_max;
	uint time;
public:
    BinCount(){}
	BinCount(uint cols, uint rows, uint time);

	void run(const cv::Mat& frame, cv::Mat& bg_frame_out, cv::Mat& bg_count_out);
};

class Background{
    private:
        cv::Mat bg_frame, bg_count;
	    cv::Mat bg_frame2, bg_count2;
        BinCount* bc1;
        BinCount* bc2;
        int count;
    public:
        Background(){}
        Background(int frame_cols,int frame_rows);
        void backgroundProcess(cv::Mat frame,cv::Mat& frontend);
};
