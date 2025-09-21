#include "processorWithMesh.h"

NSP_SLAM_LYJ_SRC_BEGIN




ProcessorWithMesh::ProcessorWithMesh()
{}
ProcessorWithMesh::~ProcessorWithMesh()
{
	if (proHandle_)
		CUDA_LYJ::release(proHandle_);
}


void ProcessorWithMesh::setData(const ProcessOption& _opt)
{
	ProcessorVP::setData(_opt);
	if (!stlplus::file_exists(_opt.meshPath))
	{
		std::cout << "mesh file is not exist!" << std::endl;
		return;
	}
	SLAM_LYJ::readPLYMesh(_opt.meshPath, btm_);
	btm_.enableFCenters();
	btm_.calculateFCenters();
	btm_.enableFNormals();
	btm_.calculateFNormals();
	std::vector<float> camParams(4);
	camParams[0] = cam_.fx();
	camParams[1] = cam_.fy();
	camParams[2] = cam_.cx();
	camParams[3] = cam_.cy();
	proHandle_ = CUDA_LYJ::initProjector(btm_.getVertexs()[0].data(), btm_.getVn(), btm_.getFCenters()[0].data(), btm_.getFNormals()[0].data(), btm_.getFaces()[0].vId_, btm_.getFn(), camParams.data(), cam_.wide(), cam_.height());
	proCache_.init(btm_.getVn(), btm_.getFn(), cam_.wide(), cam_.height());
	for (int i = 0; i < imageExtractDatasPtr_.size(); ++i)
	{
		imageExtractDatasPtr_[i]->btmPtr = &btm_;
	}
}

struct ProBuffer
{
	ProBuffer() {}
	ProBuffer(const int& _w, const int& _h, SLAM_LYJ::BaseTriMesh* _btmPtr, const float& _minD=0.01f, const float& _maxD=FLT_MAX, const float& _csTh=0.5f, const float& _detDTh=0.01f)
		:w(_w),h(_h), btmPtr(_btmPtr), minD(_minD), maxD(_maxD), csTh(_csTh), detDTh(_detDTh)
	{
		depthsM = cv::Mat(h, w, CV_32FC1);
		fIds.assign(w * h, UINT32_MAX);
		allVisiblePIds.assign(_btmPtr->getVn(), 0);
		allVisibleFIds.assign(_btmPtr->getFn(), 0);
	}
	void updateTcw(const SLAM_LYJ::Pose3D& _Tcw)
	{
		_Tcw.getMatrix34f(Tcw);
	}

	int w = 0;
	int h = 0;
	SLAM_LYJ::BaseTriMesh* btmPtr = nullptr;
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
bool ProcessorWithMesh::extractFeature()
{
	if (!proHandle_)
		return false;
	int imgSize = imageExtractDatasPtr_.size();
	int w = cam_.wide();
	int h = cam_.height();
	ProBuffer pBuffer(w, h, &btm_);
	for (int i = 0; i < imgSize; ++i)
	{
		pBuffer.updateTcw(imageExtractDatasPtr_[i]->Tcw);
		CUDA_LYJ::project(proHandle_, proCache_, pBuffer.Tcw.data(), (float*)pBuffer.depthsM.data, pBuffer.fIds.data(), pBuffer.allVisiblePIds.data(), pBuffer.allVisibleFIds.data(), pBuffer.minD, pBuffer.maxD, pBuffer.csTh, pBuffer.detDTh);
		imageExtractDatasPtr_[i]->depths = pBuffer.depthsM.clone();
		imageExtractDatasPtr_[i]->fIds = pBuffer.fIds;
		cv::Mat depthsShow(h, w, CV_8UC1);
		depthsShow.setTo(0);
		for (int r = 0; r < h; ++r)
		{
			for (int c = 0; c < w; ++c)
			{
				float d = imageExtractDatasPtr_[i]->depths.at<float>(r, c);
				if (d == FLT_MAX)
					continue;
				int di = d * 10;
				depthsShow.at<uchar>(r, c) = (uchar)di;
			}
		}
		cv::imshow("depth", depthsShow);
		cv::waitKey();
	}
	bool ret = ProcessorVP::extractFeature();
	//for (int i = 0; i < imgSize; ++i)
	//{
	//	cv::Mat depthsShow(h, w, CV_8UC1);
	//	depthsShow.setTo(0);
	//	for (int r = 0; r < h; ++r)
	//	{
	//		for (int c = 0; c < w; ++c)
	//		{
	//			float d = imageExtractDatasPtr_[i]->depths.at<float>(r, c);
	//			if (d == FLT_MAX)
	//				continue;
	//			int di = d * 10;
	//			depthsShow.at<uchar>(r, c) = (uchar)di;
	//		}
	//	}
	//	cv::imshow("depth", depthsShow);
	//	cv::imshow("image", imageExtractDatasPtr_[i]->img);
	//	cv::waitKey();
	//	auto P3Ds = imageExtractDatasPtr_[i]->kp3Ds_;
	//	SLAM_LYJ::Pose3D Twc = imageExtractDatasPtr_[i]->Tcw.inversed();
	//	for (int j = 0; j < P3Ds.size(); ++j)
	//	{
	//		P3Ds[j] = Twc * imageExtractDatasPtr_[i]->kp3Ds_[j];
	//	}
	//	SLAM_LYJ::BaseTriMesh btmTmp;
	//	btmTmp.setVertexs(P3Ds);
	//	SLAM_LYJ::writePLYMesh("D:/tmp/p3ds.ply", btmTmp);
	//	continue;
	//}

	return ret;
}
inline bool ProcessorWithMesh::generateMap()
{
	return false;
}
inline bool ProcessorWithMesh::optimize()
{
	return false;
}


NSP_SLAM_LYJ_SRC_END