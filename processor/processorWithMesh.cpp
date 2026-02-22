#include "processorWithMesh.h"
//#include <common/FlannSearch.h>
#include <common/Timer.h>

NSP_SLAM_LYJ_SRC_BEGIN




ProcessorWithMesh::ProcessorWithMesh()
{
}
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
	COMMON_LYJ::readPLYMesh(_opt.meshPath, btm_);
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
	proCaches_.resize(opt_.threadNum);
	for(int i=0;i< opt_.threadNum;++i)
		proCaches_[i].init(btm_.getVn(), btm_.getFn(), cam_.wide(), cam_.height());
	for (int i = 0; i < imageExtractDatasPtr_.size(); ++i)
	{
		imageExtractDatasPtr_[i]->btmPtr = &btm_;
	}
}

struct ProBuffer
{
	ProBuffer() {}
	ProBuffer(const int& _w, const int& _h, COMMON_LYJ::BaseTriMesh* _btmPtr, const float& _minD=0.01f, const float& _maxD=FLT_MAX, const float& _csTh=0.5f, const float& _detDTh=0.01f)
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
bool ProcessorWithMesh::extractFeature()
{
	if (!proHandle_)
		return false;
	int imgSize = imageExtractDatasPtr_.size();
	int w = cam_.wide();
	int h = cam_.height();
	std::vector<ProBuffer> pBuffers(opt_.threadNum);
	for(int i=0;i< opt_.threadNum;++i)
		pBuffers[i].init(w, h, &btm_);
	auto funcProject = [&](uint64_t _s, uint64_t _e, uint32_t _id) {
		for (int i = _s; i < _e; ++i)
		{
			COMMON_LYJ::Timer q;
			pBuffers[_id].updateTcw(imageExtractDatasPtr_[i]->Tcw);
			CUDA_LYJ::project(proHandle_, proCaches_[_id], pBuffers[_id].Tcw.data(), (float*)pBuffers[_id].depthsM.data, pBuffers[_id].fIds.data(), pBuffers[_id].allVisiblePIds.data(), pBuffers[_id].allVisibleFIds.data(), pBuffers[_id].minD, pBuffers[_id].maxD, pBuffers[_id].csTh, pBuffers[_id].detDTh);
			double t = q.elapsed();
			std::cout << "cost time: " << t << " ms" << std::endl;
			imageExtractDatasPtr_[i]->depths = pBuffers[_id].depthsM.clone();
			imageExtractDatasPtr_[i]->fIds = pBuffers[_id].fIds;
			//cv::Mat depthsShow(h, w, CV_8UC1);
			//depthsShow.setTo(0);
			//for (int r = 0; r < h; ++r)
			//{
			//	for (int c = 0; c < w; ++c)
			//	{
			//		float d = imageExtractDatasPtr_[i]->depths.at<float>(r, c);
			//		if (d == FLT_MAX)
			//			continue;
			//		int di = d * 10;
			//		depthsShow.at<uchar>(r, c) = (uchar)di;
			//	}
			//}
			//cv::imshow("depth", depthsShow);
			//cv::waitKey();
			if (true)
			{
				std::vector<Eigen::Vector3f> clrs(btm_.getVn());
				std::string objPath = "D:/SLAM_LYJ_Packages/SLAM_LYJ_qt/data/";
				cv::Mat immm = cv::imread(imageExtractDatasPtr_[i]->path);
				cv::imwrite(objPath + "texture.png", immm);
				const auto& Tcw = imageExtractDatasPtr_[i]->Tcw;
				const auto& cam = imageExtractDatasPtr_[i]->cam;
				const auto& pValids = pBuffers[_id].allVisiblePIds;
				int pSize = pBuffers[_id].allVisiblePIds.size();
				std::vector<float> uvs(pSize * 2);
				Eigen::Vector2f uv;
				Eigen::Vector3f Pc;
				int ww = cam->wide();
				int hh = cam->height();
				for (int vi = 0; vi < pSize; ++vi)
				{
					if (pValids[vi] == 0)
					{
						uvs[vi * 2] = -1;
						uvs[vi * 2 + 1] = -1;
						clrs[vi] = Eigen::Vector3f(0, 0, 0);
					}
					else
					{
						const Eigen::Vector3f& Pw = btm_.getVertex(vi);
						Pc = Tcw * Pw;
						cam->world2Image(Pc, uv);
						uvs[vi * 2] = uv[0] / ww;
						uvs[vi * 2 + 1] = 1.f - uv[1] / hh;
						clrs[vi] = Eigen::Vector3f(1, 1, 1);
					}
				}
				std::ofstream fff(objPath + "uvs.txt");
				for (int ii = 0; ii < pSize * 2; ++ii)
					fff << uvs[ii] << " ";
				fff << std::endl;
				fff.close();
				COMMON_LYJ::BaseTriMesh btm2 = btm_;
				btm2.enableVColors();
				btm2.setVColors(clrs);
				COMMON_LYJ::writePLYMesh(objPath + "btm.ply", btm2);
				continue;
			}
		}
	};
	//int threadsNum = std::thread::hardware_concurrency();
	//std::cout << "threads num: " << threadsNum << std::endl;
	{
		auto t_start = std::chrono::high_resolution_clock::now();
		COMMON_LYJ::ThreadPool threadPool(opt_.threadNum);
		threadPool.processWithId(funcProject, 0, imgSize);
		std::cout << "project time: "
			<< std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::high_resolution_clock::now() - t_start).count() << " ms\n";
	}
	bool ret = true;
	ret = ProcessorVP::extractFeature();
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
	//	cv::pyrDown(depthsShow, depthsShow);
	//	cv::pyrDown(depthsShow, depthsShow);
	//	cv::imshow("depth", depthsShow);
	//	//cv::imshow("image", imageExtractDatasPtr_[i]->img);
	//	cv::waitKey();
	//	//auto P3Ds = imageExtractDatasPtr_[i]->kp3Ds_;
	//	//COMMON_LYJ::Pose3D Twc = imageExtractDatasPtr_[i]->Tcw.inversed();
	//	//for (int j = 0; j < P3Ds.size(); ++j)
	//	//{
	//	//	P3Ds[j] = Twc * imageExtractDatasPtr_[i]->kp3Ds_[j];
	//	//}
	//	//COMMON_LYJ::BaseTriMesh btmTmp;
	//	//btmTmp.setVertexs(P3Ds);
	//	//COMMON_LYJ::writePLYMesh("D:/tmp/p3ds.ply", btmTmp);
	//	//continue;
	//}

	return ret;
}
inline bool ProcessorWithMesh::generateMap()
{
	return false;
}
bool ProcessorWithMesh::incrementalMap()
{
	return false;
}
inline bool ProcessorWithMesh::optimize()
{
	return false;
}


NSP_SLAM_LYJ_SRC_END