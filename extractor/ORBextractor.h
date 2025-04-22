#ifndef SLAM_LYJ_ORBEXTRACTOR_H
#define SLAM_LYJ_ORBEXTRACTOR_H

#include "extractorAbr.h"
#include "base/Frame.h"

NSP_SLAM_LYJ_BEGIN

class ORBExtractor : public ExtractorAbr
{
public:
	struct Option
	{
		bool justFast = false;
	};
	ORBExtractor(Option _opt);
	~ORBExtractor();
	struct ORBExtractOutput
	{
		ORBExtractOutput(std::vector<cv::KeyPoint> *_kps, cv::Mat *_descriptors)
		{
			kps = _kps;
			descriptors = _descriptors;
		}
		std::vector<cv::KeyPoint> *kps = nullptr;
		cv::Mat *descriptors = nullptr;
	};

	// Í¨¹ý ExtractorAbr ¼Ì³Ð
	void extract(cv::Mat _img, ExtractorOutput _output) override;

	void extract(cv::Mat _img, Frame &_frame) override;

private:
	cv::Ptr<cv::ORB> orb_;
	Option opt_;
};

namespace FROM_ORB_SLAM3
{

	class ExtractorNode
	{
	public:
		ExtractorNode() : bNoMore(false) {}

		void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

		std::vector<cv::KeyPoint> vKeys;
		cv::Point2i UL, UR, BL, BR;
		std::list<ExtractorNode>::iterator lit;
		bool bNoMore;
	};

	class ORBextractor
	{
	public:
		enum
		{
			HARRIS_SCORE = 0,
			FAST_SCORE = 1
		};

		ORBextractor(int nfeatures, float scaleFactor, int nlevels,
					 int iniThFAST, int minThFAST);

		~ORBextractor() {}

		// Compute the ORB features and descriptors on an image.
		// ORB are dispersed on the image using an octree.
		// Mask is ignored in the current implementation.
		int operator()(cv::InputArray _image, cv::InputArray _mask,
					   std::vector<cv::KeyPoint> &_keypoints,
					   cv::OutputArray _descriptors, std::vector<int> &vLappingArea);

		int inline GetLevels()
		{
			return nlevels;
		}

		float inline GetScaleFactor()
		{
			return scaleFactor;
		}

		std::vector<float> inline GetScaleFactors()
		{
			return mvScaleFactor;
		}

		std::vector<float> inline GetInverseScaleFactors()
		{
			return mvInvScaleFactor;
		}

		std::vector<float> inline GetScaleSigmaSquares()
		{
			return mvLevelSigma2;
		}

		std::vector<float> inline GetInverseScaleSigmaSquares()
		{
			return mvInvLevelSigma2;
		}

		std::vector<cv::Mat> mvImagePyramid;

	protected:
		void ComputePyramid(cv::Mat image);
		void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint>> &allKeypoints);
		std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
													const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

		void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint>> &allKeypoints);
		std::vector<cv::Point> pattern;

		int nfeatures;
		double scaleFactor;
		int nlevels;
		int iniThFAST;
		int minThFAST;

		std::vector<int> mnFeaturesPerLevel;

		std::vector<int> umax;

		std::vector<float> mvScaleFactor;
		std::vector<float> mvInvScaleFactor;
		std::vector<float> mvLevelSigma2;
		std::vector<float> mvInvLevelSigma2;
	};

} // from ORB_SLAM

NSP_SLAM_LYJ_END

#endif // SLAM_LYJ_ORBEXTRACTOR_H