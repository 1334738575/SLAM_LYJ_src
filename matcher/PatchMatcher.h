#ifndef SLAM_LYJ_PATCHMATCHER_H
#define SLAM_LYJ_PATCHMATCHER_H

#include "matcherAbr.h"


NSP_SLAM_LYJ_BEGIN

struct PatchMatchResult
{
	int id1 = -1;
	Eigen::Vector2f p1 = Eigen::Vector2f::Zero();
	int id2 = -1; //reserve for keypoint2
	Eigen::Vector2f p2 = Eigen::Vector2f::Zero();
	Eigen::Vector2f offset = Eigen::Vector2f::Zero();
	float d = FLT_MAX;
	bool valid = false;
};

class PatchMatcher : public MatcherAbr
{
public:
	struct Option
	{
		float th = 50;
		int searchR = 50;
		int resolution = 100;
		int maxIterNum = 4;
		int candOffsetSize = 3;
	};


	PatchMatcher(Option _opt);
	~PatchMatcher();



	// Í¨¹ý MatcherAbr ¼Ì³Ð
	int match(const Frame& _frame1, const Frame& _frame2, std::vector<int>& _match2to1) override;

	int matchPatch(const cv::Mat& _m1, std::vector<cv::KeyPoint>& _kps1,
		const cv::Mat& _m2,	const bool _bGuass, 
		std::vector<PatchMatchResult>& _matches);

private:

	void initPoints(const cv::Mat& _m, std::vector<cv::KeyPoint>& _kps);
	std::shared_ptr<SLAM_LYJ_MATH::Grid2Df> generateGrid(const int _w, const int _h,
		const std::vector<cv::KeyPoint>& _kps);
	void initMatchResults(const std::vector<cv::KeyPoint>& _kps1, std::shared_ptr<SLAM_LYJ_MATH::Grid2Df> _grid, std::vector<PatchMatchResult>& _matches);
	void initOffsets(const cv::Mat& _m1, const cv::Mat& _m2, std::shared_ptr<SLAM_LYJ_MATH::Grid2Df> _grid, std::vector<PatchMatchResult>& _matches, const int _candSize);
	bool findMatchPoint(const cv::Mat& _m1, const cv::Mat& _m2,
		const Eigen::Vector2f& _p1, Eigen::Vector2f& _offset, float& _bestD,
		const int _r, const float _th);
	void resetMatchResults(std::vector<PatchMatchResult>& _matches);

private:
	Option m_opt;
};


NSP_SLAM_LYJ_END


#endif // !SLAM_LYJ_PATCHMATCHER_H
