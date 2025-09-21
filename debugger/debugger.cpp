#include "debugger.h"

NSP_SLAM_LYJ_DEBUGGER_BEGIN


void drawFeatures(const cv::Mat& _img, const std::vector<Eigen::Vector2f>& _kps, cv::Mat& _imgOut,
    const cv::Scalar _color, const int _radius, const int _thick) {
    _imgOut = _img.clone();
    for (const auto& kp : _kps)
        cv::circle(_imgOut, cv::Point2f(kp.x(), kp.y()), _radius, _color, _thick);
}
void drawFeatures(const cv::Mat& _img, const std::vector<cv::KeyPoint>& _kps, cv::Mat& _imgOut,
    const cv::Scalar _color, const int _radius, const int _thick, bool _showEvery
) {
    //cv::drawKeypoints(_img, _kps, _imgOut, cv::Scalar(0));
    _imgOut = _img.clone();
    for (const auto& kp : _kps) {
        cv::circle(_imgOut, kp.pt, _radius, _color, _thick);
        if (_showEvery) {
            cv::imshow("keypoint", _imgOut);
            cv::waitKey();
        }
    }
}
void drawLineFeatures(const cv::Mat& _img, const std::vector<cv::Vec4f>& _lines, cv::Mat& _imgOut,
    const cv::Scalar _color, const int _thick)
{
    _imgOut = _img.clone();
    for (const auto& line : _lines)
        cv::line(_imgOut, cv::Point2f(line(0), line(1)), cv::Point2f(line(2), line(3)), _color, _thick);
}
void drawLineFeatures(const cv::Mat& _img, const std::vector<cv::Vec4f>& _lines,
    const cv::Ptr<cv::LineSegmentDetector>& _lsd, cv::Mat& _imgOut) {
    _imgOut = _img.clone();
    _lsd->drawSegments(_imgOut, _lines);
}
//void drawEdgeFeatures(const cv::Mat& _img, const std::shared_ptr<Frame::EdgeFeatures> _edges, cv::Mat& _imgOut,
//    const cv::Scalar _color) {
//    if (_edges->cannyM.cols != 0 && _edges->cannyM.rows != 0) {
//        _imgOut = _edges->cannyM.clone();
//        return;
//    }
//    else if (!_edges->edges_.empty()) {
//        drawFeatures(_img, _edges->edges_, _imgOut, _color, 0);
//        return;
//    }
//}
void drawGrid(const cv::Mat& _img, const SLAM_LYJ_MATH::Grid2Df& _grid, cv::Mat& _imgOut, const cv::Scalar _color, const int _thick)
{
    _imgOut = _img.clone();
    int gw = _grid.getGridSize(0);
    int gh = _grid.getGridSize(1);
    int resolution = _grid.getResolution();
    for (int i = 0; i < gh; ++i) {
        for (int j = 0; j < gw; ++j) {
			Eigen::Vector2f p1 = _grid.toRealCoord2Df(j, i);
			cv::rectangle(_imgOut, cv::Point2f(p1.x(), p1.y()), cv::Point2f(p1.x() + resolution, p1.y() + resolution), _color, _thick);
            //cv::imshow("rect", _imgOut);
            //cv::waitKey();
        }
    }

}

void drawPointMatches(const cv::Mat& _img1, const std::vector<cv::KeyPoint>& _kps1,
    const cv::Mat& _img2, const std::vector<cv::KeyPoint>& _kps2, 
    const std::vector<std::pair<int, int>>& _matches, cv::Mat& _imgOut,
    const cv::Scalar _color, const int _thick)
{
    if (_img1.type() != _img2.type()) {
        std::cout << __FUNCTION__ << " type of img1 is not equal to img2" << std::endl;
        return;
    }
	_imgOut = cv::Mat(std::max(_img1.rows, _img2.rows), _img1.cols + _img2.cols, _img1.type());
	cv::Mat left = _imgOut(cv::Rect(0, 0, _img1.cols, _img1.rows));
	cv::Mat right = _imgOut(cv::Rect(_img1.cols, 0, _img2.cols, _img2.rows));
	_img1.copyTo(left);
	_img2.copyTo(right);
	for (const auto& match : _matches) {
		const auto& kp1 = _kps1[match.first];
		const auto& kp2 = _kps2[match.second];
		cv::line(_imgOut, kp1.pt, kp2.pt + cv::Point2f(_img1.cols, 0), _color, _thick);
	}
}
void drawPointMatches(const cv::Mat& _img1, const std::vector<cv::KeyPoint>& _kps1,
    const cv::Mat& _img2, const std::vector<cv::KeyPoint>& _kps2,
    const std::vector<int>& _matches, cv::Mat& _imgOut,
    const cv::Scalar _color, const int _thick)
{
    if (_img1.type() != _img2.type()) {
        std::cout << __FUNCTION__ << " type of img1 is not equal to img2" << std::endl;
        return;
    }
    _imgOut = cv::Mat(std::max(_img1.rows, _img2.rows), _img1.cols + _img2.cols, _img1.type());
    cv::Mat left = _imgOut(cv::Rect(0, 0, _img1.cols, _img1.rows));
    cv::Mat right = _imgOut(cv::Rect(_img1.cols, 0, _img2.cols, _img2.rows));
    _img1.copyTo(left);
    _img2.copyTo(right);
    for (int i = 0; i < _kps1.size();++i) {
        if (_matches[i] == -1)
            continue;
        const auto& kp1 = _kps1[i];
        const auto& kp2 = _kps2[_matches[i]];
        cv::line(_imgOut, kp1.pt, kp2.pt + cv::Point2f(_img1.cols, 0), _color, _thick);
    }
}
//void drawPatchMatches(const cv::Mat& _img1, const cv::Mat& _img2, const std::vector<PatchMatchResult>& _matches, cv::Mat& _imgOut, const cv::Scalar _color, const int _thick)
//{
//    if (_img1.type() != _img2.type()) {
//        std::cout << __FUNCTION__ << " type of img1 is not equal to img2" << std::endl;
//        return;
//    }
//    _imgOut = cv::Mat(std::max(_img1.rows, _img2.rows), _img1.cols + _img2.cols, _img1.type());
//    cv::Mat left = _imgOut(cv::Rect(0, 0, _img1.cols, _img1.rows));
//    cv::Mat right = _imgOut(cv::Rect(_img1.cols, 0, _img2.cols, _img2.rows));
//    _img1.copyTo(left);
//    _img2.copyTo(right);
//    for (const auto& match : _matches) {
//        if (!match.valid)
//            continue;
//        const auto& kp1 = match.p1;
//        const auto& kp2 = match.p1 + match.offset;
//        cv::line(_imgOut, cv::Point(kp1(0), kp1(1)), cv::Point(kp2(0) + _img1.cols, kp2(1)), _color, _thick);
//    }
//}



NSP_SLAM_LYJ_DEBUGGER_END