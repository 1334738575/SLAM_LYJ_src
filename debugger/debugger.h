#ifndef SLAM_LYJ_DEBUGGER_H
#define SLAM_LYJ_DEBUGGER_H


#include <base/PreDefine.h>
#include <opencv2/opencv.hpp>
#include <base/Frame.h>
#include <matcher/PatchMatcher.h>


NSP_SLAM_LYJ_DEBUGGER_BEGIN



//debug
void drawFeatures(const cv::Mat& _img, const std::vector<Eigen::Vector2f>& _kps, cv::Mat& _imgOut,
    const cv::Scalar _color, const int _radius = 3, const int _thick = 1);
void drawFeatures(const cv::Mat& _img, const std::vector<cv::KeyPoint>& _kps, cv::Mat& _imgOut,
    const cv::Scalar _color, const int _radius = 3, const int _thick = 1, bool _showEvery = false);
void drawLineFeatures(const cv::Mat& _img, const std::vector<cv::Vec4f>& _lines, cv::Mat& _imgOut,
    const cv::Scalar _color, const int _thick = 1);
void drawLineFeatures(const cv::Mat& _img, const std::vector<cv::Vec4f>& _lines,
    const cv::Ptr<cv::LineSegmentDetector>& _lsd, cv::Mat& _imgOut);
void drawEdgeFeatures(const cv::Mat& _img, const std::shared_ptr<Frame::EdgeFeatures> _edges, cv::Mat& _imgOut,
    const cv::Scalar _color);
void drawGrid(const cv::Mat& _img, const SLAM_LYJ_MATH::Grid2Df& _grid, cv::Mat& _imgOut,
    const cv::Scalar _color, const int _thick = 1);

void drawPointMatches(const cv::Mat& _img1, const std::vector<cv::KeyPoint>& _kps1,
	const cv::Mat& _img2, const std::vector<cv::KeyPoint>& _kps2,
	const std::vector<std::pair<int, int>>& _matches, cv::Mat& _imgOut,
	const cv::Scalar _color = cv::Scalar(255, 255, 255), const int _thick = 1);
void drawPatchMatches(const cv::Mat& _img1,	const cv::Mat& _img2,
	const std::vector<PatchMatchResult>& _matches, cv::Mat& _imgOut,
	const cv::Scalar _color = cv::Scalar(255, 255, 255), const int _thick = 1);


NSP_SLAM_LYJ_DEBUGGER_END




#endif //SLAM_LYJ_DEBUGGER_H