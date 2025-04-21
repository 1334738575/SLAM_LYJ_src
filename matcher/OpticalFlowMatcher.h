#ifndef SLAM_LYJ_OPTICALFLOW_H
#define SLAM_LYJ_OPTICALFLOW_H

#include "matcherAbr.h"

NSP_SLAM_LYJ_BEGIN


class OpticalFlowMatcher : public MatcherAbr
{
public:
	struct Option
	{
		cv::Size winSize = cv::Size(21, 21);
		int level = 3;
	};

	OpticalFlowMatcher(Option _opt);
	~OpticalFlowMatcher();



	// Í¨¹ý MatcherAbr ¼Ì³Ð
	int match(const Frame& _frame1, const Frame& _frame2, std::vector<int>& _match2to1, std::vector<float>& _weights) override;
	int matchOpticalFlow(const cv::Mat& _m1, const cv::Mat& _m2,
		const std::vector<cv::KeyPoint>& _kps1, std::vector<cv::KeyPoint>& _kps2,
		std::vector<int>& _match2to1, std::vector<float>& _weights);
	int matchOpticalFlow(const cv::Mat& _m1, const cv::Mat& _m2,
		const std::vector<cv::Point2f>& _kps1, std::vector<cv::Point2f>& _kps2,
		std::vector<int>& _match2to1, std::vector<float>& _weights);

private:
	Option m_opt;
};






NSP_SLAM_LYJ_END

#endif // !SLAM_LYJ_OPTICALFLOW_H
