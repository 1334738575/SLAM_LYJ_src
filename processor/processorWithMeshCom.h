#ifndef SLAM_LYJ_PROCESSORWITHMESHCOM_H
#define SLAM_LYJ_PROCESSORWITHMESHCOM_H

#include "processorVP.h"
#include <CUDAInclude.h>
#include <DBoW3/DBoW3.h>
#include <common/CommonAlgorithm.h>


NSP_SLAM_LYJ_SRC_BEGIN


struct ObserveDataCom
{
	ObserveDataCom(const Eigen::Vector3f& _P3D, const int& _imgId, const int& _kpId)
	{
		P3D = _P3D.cast<double>();
		obs.emplace_back(_imgId, _kpId);
	}
	Eigen::Vector3d P3D;
	std::vector<Eigen::Vector2i> obs;
};
struct ExtractDataCom
{
	int id = -1;//0开始，和输入image的索引相同

	COMMON_LYJ::CompressedImage* img = nullptr;
	COMMON_LYJ::PinholeCamera* cam = nullptr;
	COMMON_LYJ::Pose3D Tcw;

	std::vector<cv::KeyPoint> kps;
	cv::Mat descs;
	DBoW3::FeatureVector featureVec;
	DBoW3::BowVector bowVec;

	std::vector<Eigen::Vector3f> P3Ds;
	std::vector<int> fIds;

	std::vector<ObserveDataCom*> landmarks;
};
struct PairDataCom
{
	COMMON_LYJ::BitFlagVec status;
};
struct MatchDataCom
{
	int id1 = -1;
	int id2 = -1;
	int64_t pairId = 0;
	std::vector<Eigen::Vector2i> matches;
};
struct ProBufferCom
{
	ProBufferCom() {}
	ProBufferCom(const int& _w, const int& _h, COMMON_LYJ::BaseTriMesh* _btmPtr, const float& _minD = 0.01f, const float& _maxD = FLT_MAX, const float& _csTh = 0.5f, const float& _detDTh = 0.01f)
	{
		init(_w, _h, _btmPtr, _minD, _maxD, _csTh, _detDTh);
	}
	void updateTcw(const COMMON_LYJ::Pose3D& _Tcw)
	{
		_Tcw.getMatrix34f(Tcw);
	}

	void init(const int& _w, const int& _h, COMMON_LYJ::BaseTriMesh* _btmPtr, const float& _minD = 0.01f, const float& _maxD = FLT_MAX, const float& _csTh = 0.5f, const float& _detDTh = 0.01f)
	{
		w = _w;
		h = _h;
		btmPtr = _btmPtr;
		minD = _minD;
		maxD = _maxD;
		csTh = _csTh;
		detDTh = _detDTh;
		depthsM = cv::Mat(h, w, CV_32FC1);
		fIds.assign(w * h, UINT32_MAX);
		allVisiblePIds.assign(_btmPtr->getVn(), 0);
		allVisibleFIds.assign(_btmPtr->getFn(), 0);
	}

	int w = 0;
	int h = 0;
	COMMON_LYJ::BaseTriMesh* btmPtr = nullptr;
	Eigen::Matrix<float, 3, 4> Tcw;
	cv::Mat depthsM;
	std::vector<uint> fIds;
	std::vector<char> allVisiblePIds;
	std::vector<char> allVisibleFIds;
	float minD = 0.01f;
	float maxD = FLT_MAX;
	float csTh = 0.5f;
	float detDTh = 0.01f;

};

class ProcessorWithMeshCom
{
public:
	ProcessorWithMeshCom();
	~ProcessorWithMeshCom();

	virtual void process(COMMON_LYJ::BaseTriMesh& _btm, std::vector<COMMON_LYJ::CompressedImage*>& _imgs, std::vector<COMMON_LYJ::Pose3D>& _Tcws, std::vector<COMMON_LYJ::PinholeCamera>& _cams, ProcessComOption _opt);

protected:
	virtual bool extract();
	virtual bool generatePairs();
	virtual bool match();
	void dump2D(std::string _path);
	virtual bool generate3DInfos();
	virtual bool optimize(int _i);
	virtual bool optimizeCeres();

protected:
	COMMON_LYJ::BaseTriMesh* btm_ = nullptr;
	std::vector<COMMON_LYJ::CompressedImage*> imgs_;
	std::vector<COMMON_LYJ::Pose3D> Tcws_;
	COMMON_LYJ::PinholeCamera cam_;
	ProcessComOption opt_;
	int threadNum_ = 1;

	std::vector<ExtractDataCom> extractDatas_;
	std::vector<PairDataCom> pairDatas_;
	std::vector<MatchDataCom> matchDatas_;
	CUDA_LYJ::ProHandle proHandle_ = nullptr;
	std::vector<CUDA_LYJ::ProjectorCache> proCaches_;
	std::vector<ProBufferCom> pBuffers_;

	std::shared_ptr<DBoW3::Vocabulary> voc_ = nullptr;
	std::shared_ptr<DBoW3::Database> vocDB_ = nullptr;

	double lastErr_ = DBL_MAX;
};



NSP_SLAM_LYJ_SRC_END


#endif // !SLAM_LYJ_PROCESSORWITHMESHCOM_H
