#include "processorWithMeshComAft.h"

#include <common/Timer.h>

#include <Optimize_LYJ.h>
#include <Optimizer/optimizer.h>
#include <Factor/Factor.h>
#include <Variable/Variable.h>
#include <CeresCheck/CeresProblem/CeresProblem.h>
#include <ImageCommon/FeatureGrid.h>

#include <debugger/debugger.h>

NSP_SLAM_LYJ_SRC_BEGIN


static void ComputeThreeMaxima(std::vector<int>* histo, const int L, int& ind1, int& ind2, int& ind3)
{
	int max1 = 0;
	int max2 = 0;
	int max3 = 0;

	for (int i = 0; i < L; i++)
	{
		const int s = histo[i].size();
		if (s > max1)
		{
			max3 = max2;
			max2 = max1;
			max1 = s;
			ind3 = ind2;
			ind2 = ind1;
			ind1 = i;
		}
		else if (s > max2)
		{
			max3 = max2;
			max2 = s;
			ind3 = ind2;
			ind2 = i;
		}
		else if (s > max3)
		{
			max3 = s;
			ind3 = i;
		}
	}

	if (max2 < 0.1f * (float)max1)
	{
		ind2 = -1;
		ind3 = -1;
	}
	else if (max3 < 0.1f * (float)max1)
	{
		ind3 = -1;
	}
}
static int DescriptorDistance(const cv::Mat& a, const cv::Mat& b)
{
	const int* pa = a.ptr<int32_t>();
	const int* pb = b.ptr<int32_t>();

	int dist = 0;

	for (int i = 0; i < 8; i++, pa++, pb++)
	{
		unsigned int v = *pa ^ *pb;
		v = v - ((v >> 1) & 0x55555555);
		v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
		dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
	}

	return dist;
}
static int matchByPro(
	int w2, int h2,
	const Eigen::Matrix3d& K2,
	const std::vector<cv::KeyPoint>& features1, const std::vector<cv::KeyPoint>& features2,
	const cv::Mat& desc1, const cv::Mat& desc2,
	ImageProcess_LYJ::FeatureGrid* grid2,
	COMMON_LYJ::Pose3D& _Tcw1, COMMON_LYJ::Pose3D& _Tcw2,
	std::vector<Eigen::Vector3f>& _P3Ds1, std::vector<Eigen::Vector3f>& _P3Ds2, float _squareDistTh,
	std::vector<std::shared_ptr<ObserveDataCom>>& _landmarks1,
	std::vector<int>& _matches2to1,
	double th = 50, double nnTh = 0.9)
{
	int cnt = 0;
	auto& matches = _matches2to1;
	matches.assign(features1.size(), -1);
	std::vector<int> matches2(features2.size(), -1);
	// compute F matrix
	COMMON_LYJ::Pose3D T21 = _Tcw2 * _Tcw1.inversed();
	Eigen::Vector3f Pw1;
	Eigen::Vector3f Pw2;
	COMMON_LYJ::Pose3D Twc1 = _Tcw1.inversed();
	COMMON_LYJ::Pose3D Twc2 = _Tcw2.inversed();

	for (size_t i = 0; i < features1.size(); ++i)
	{
		// Eigen::Vector3d p1(features1[i].pt.x, features1[i].pt.y, 1);
		const cv::Mat& des1 = desc1.row(i);
		if (_landmarks1[i] == nullptr)
			continue;    
		Pw1(2) = -1;
		if (!_P3Ds1.empty() && _P3Ds1[i](2) > 0)
			Pw1 = Twc1 * _P3Ds1[i];

		// compute project
		Eigen::Vector3d Pc2 = _Tcw2 * _landmarks1[i]->P3D;
		if (Pc2(2) <= 0)
			continue;
		Pc2 /= Pc2(2);
		Eigen::Vector3d p2 = K2 * Pc2;
		if (p2(0) >= w2 || p2(0) < 0 || p2(1) >= h2 || p2(1) < 0)
			continue;
		// get ids
		std::vector<size_t> ids;
		grid2->getKeypointIdsAround((float)p2(0), (float)p2(1), ids);
		if (ids.empty())
		{
			continue;
		}

		// match
		int best_id = -1;
		int best_dist = 128 * 255;
		int second_id = -1;
		int second_dist = 128 * 255;
		for (size_t j = 0; j < ids.size(); ++j)
		{
			if (_P3Ds2[ids[j]](2) <= 0)
				continue;
			Pw2 = Twc2 * _P3Ds2[ids[j]];
			if ((Pw1 - Pw2).squaredNorm() > _squareDistTh)
				continue;
			if ((_landmarks1[i]->P3D.cast<float>() - Pw2).squaredNorm() > _squareDistTh)
				continue;
			// compute distance
			const cv::Mat& des2 = desc2.row(ids[j]);
			const int dist = DescriptorDistance(des1, des2);

			// judge
			if (dist < best_dist)
			{
				second_id = (int)best_id;
				second_dist = best_dist;
				best_id = (int)ids[j];
				best_dist = dist;
			}
			else if (dist < second_dist)
			{
				second_id = (int)ids[j];
				second_dist = dist;
			}
		}

		// record
		if (best_dist > th || second_dist * nnTh < best_dist)
			continue;
		if (matches2[best_id] != -1)
		{
			continue;
		}
		matches[i] = best_id;
		matches2[best_id] = i;
		++cnt;
	}

	return cnt;
}


ProcessorWithMeshComAft::ProcessorWithMeshComAft()
{
}
ProcessorWithMeshComAft::~ProcessorWithMeshComAft()
{
	if (proHandle_)
		CUDA_LYJ::release(proHandle_);
	proHandle_ = nullptr;
}


void ProcessorWithMeshComAft::process(COMMON_LYJ::BaseTriMesh& _btm, std::vector<COMMON_LYJ::CompressedImage*>& _imgs, std::vector<COMMON_LYJ::Pose3D>& _Tcws, std::vector<COMMON_LYJ::PinholeCamera>& _cams, ProcessComOption _opt)
{
	btm_ = &_btm;
	imgs_ = _imgs;
	Tcws_ = _Tcws;
	cam_ = _cams[0];
	opt_ = _opt;
	threadNum_ = opt_.threadNum;
	if (opt_.threadNum == -1)
		threadNum_ = std::thread::hardware_concurrency() - 2;

	extract();
	generatePairs();
	generate3DInfos();
	dump2D("D:/tmp");
	optimize(0);

	for (int i = 0; i < Tcws_.size(); ++i)
	{
		_Tcws[i] = extractDatas_[i].Tcw;
	}
}

bool ProcessorWithMeshComAft::generatePairs()
{
	int imgSize = imgs_.size();
	pairDatas_.resize(imgSize);
	for (int i = 0; i < imgSize; ++i)
	{
		pairDatas_[i].status.assign(imgSize, false);
	}

	//seq
	int seqNN = 10;
	for (int i = 0; i < imgSize; ++i)
	{
		for (int j = i - seqNN; j <= i + seqNN; ++j)
		{
			if (j < 0 || j == i || j >= imgSize)
				continue;
			pairDatas_[i].status.setFlag(j, true);
			pairDatas_[j].status.setFlag(i, true);
		}
	}

	return true;
}
bool ProcessorWithMeshComAft::generate3DInfos()
{
	int imgSize = imgs_.size();
	int w = cam_.wide();
	int h = cam_.height();
	matchDatas_.reserve(50 * imgSize);
	observeDatas_.reserve(btm_->getFn());
	std::vector<std::vector<std::shared_ptr<ObserveDataCom>>> landmarksEveryFrames(imgSize);
	std::vector<std::shared_ptr<ImageProcess_LYJ::FeatureGrid>> grids(imgSize);
	for (int i = 0; i < imgSize; ++i)
	{
		landmarksEveryFrames[i].assign(extractDatas_[i].kps.size(), nullptr);
		grids[i] = std::make_shared<ImageProcess_LYJ::FeatureGrid>(w, h, 20, extractDatas_[i].kps);
	}


	bool bDebugMatch = false;
	auto funcMatchMulti = [&](uint64_t _s, uint64_t _e) {
		std::vector<int> match2to1;
		for (int i = _s; i < _e; ++i)
		{
			int id1 = matchDatas_[i].id1;
			int id2 = matchDatas_[i].id2;
			std::vector<Eigen::Vector2i>& matches = matchDatas_[i].matches;
			int cnt;
			auto& _frame1 = extractDatas_[id1];
			auto& _frame2 = extractDatas_[id2];
			cnt = matchByPro(
				w, h,
				_frame2.cam->getK(),
				_frame1.kps, _frame2.kps,
				_frame1.descs, _frame2.descs,
				grids[id2].get(),
				_frame1.Tcw, _frame2.Tcw,
				_frame1.P3Ds, _frame2.P3Ds, 900,
				landmarksEveryFrames[id1],
				match2to1
			);
			if (cnt < 10)
				continue;
			matches.reserve(cnt);
			for (int j = 0; j < extractDatas_[id1].kps.size(); ++j)
			{
				if (match2to1[j] == -1)
					continue;
				matches.push_back(Eigen::Vector2i(j, match2to1[j]));
			}
			if (bDebugMatch)
			{
				if (id1 == 9 && id2 == 12 && false)
				{
					std::cout << "match image " << id1 << " and image " << id2 << " cnt " << cnt << std::endl;
					for (int k = 0; k < extractDatas_[id1].kps.size(); ++k)
					{
						if (match2to1[k] == -1)
							continue;
						std::vector<int> mmm(match2to1.size(), -1);
						mmm[k] = match2to1[k];
						cv::Mat imgMatch;
						cv::Mat img1;
						cv::Mat img2;
						extractDatas_[id1].img->decompressCVMat(img1);
						extractDatas_[id2].img->decompressCVMat(img2);
						SLAM_LYJ_src::drawPointMatches(img1, extractDatas_[id1].kps, img2, extractDatas_[id2].kps, mmm, imgMatch, cv::Scalar(255, 255, 255), 3);
						cv::imshow("match", imgMatch);
						cv::waitKey();
					}
				}
				cv::Mat imgMatch;
				cv::Mat img1;
				cv::Mat img2;
				extractDatas_[id1].img->decompressCVMat(img1);
				extractDatas_[id2].img->decompressCVMat(img2);
				SLAM_LYJ_src::drawPointMatches(img1, extractDatas_[id1].kps, img2, extractDatas_[id2].kps, match2to1, imgMatch, cv::Scalar(255, 255, 255), 1);
				cv::imshow("match", imgMatch);
				cv::waitKey();
			}
		}
	};
	auto funcUpdateObserve = [&](uint64_t _s, uint64_t _e)
		{
			for (int i = _s; i < _e; ++i)
			{
				int id1 = matchDatas_[i].id1;
				int id2 = matchDatas_[i].id2;
				std::vector<Eigen::Vector2i>& matches = matchDatas_[i].matches;
				auto& _frame1 = extractDatas_[id1];
				auto& _frame2 = extractDatas_[id2];
				int cnt = matches.size();
				for (int j = 0; j < cnt; ++j)
				{
					landmarksEveryFrames[id2][matches[j](1)] = landmarksEveryFrames[id1][matches[j](0)];
					landmarksEveryFrames[id1][matches[j](0)]->obs.emplace_back(id2, matches[j](1));
				}
				auto Twc2 = _frame2.Tcw.inversed();
				for (int j = 0; j < _frame2.P3Ds.size(); ++j)
				{
					if (landmarksEveryFrames[id2][j])
						continue;
					if (_frame2.P3Ds[j](2) <= 0)
						continue;
					Eigen::Vector3f Pw = Twc2 * _frame2.P3Ds[j];
					std::shared_ptr<ObserveDataCom> obTmp = std::make_shared<ObserveDataCom>(Pw, id2, j);
					observeDatas_.push_back(obTmp);
					landmarksEveryFrames[id2][j] = obTmp;
				}
			}
		};

	auto Twc1Tmp = extractDatas_[0].Tcw.inversed();
	for (int j = 0; j < extractDatas_[0].P3Ds.size(); ++j)
	{
		if (extractDatas_[0].P3Ds[j](2) <= 0)
			continue;
		Eigen::Vector3f Pw = Twc1Tmp * extractDatas_[0].P3Ds[j];
		std::shared_ptr<ObserveDataCom> obTmp = std::make_shared<ObserveDataCom>(Pw, 0, j);
		observeDatas_.push_back(obTmp);
		landmarksEveryFrames[0][j] = obTmp;
	}
	uint64_t key;
	MatchDataCom matchDataTmp;
	for (int i = 0; i < imgSize - 1; ++i)
	{
		int st = matchDatas_.size();
		for (int j = i + 1; j < imgSize; ++j)
		{
			if (!pairDatas_[i].status[j])
				continue;
			matchDataTmp.id1 = i;
			matchDataTmp.id2 = j;
			ImageProcess_LYJ::site2Key(i, j, key);
			matchDataTmp.pairId = key;
			matchDatas_.push_back(matchDataTmp);
		}
		int ed = matchDatas_.size();

		{
			auto t_start = std::chrono::high_resolution_clock::now();
			int thdNum = threadNum_;
			if (bDebugMatch)
				thdNum = 1;
			COMMON_LYJ::ThreadPool threadPool(thdNum);
			threadPool.process(funcMatchMulti, st, ed);
			std::cout << "match time: "
				<< std::chrono::duration_cast<std::chrono::milliseconds>(
					std::chrono::high_resolution_clock::now() - t_start).count() << " ms\n";
		}
		funcUpdateObserve(st, ed);
	}



	return true;
}
bool ProcessorWithMeshComAft::optimize(int _i)
{
	std::vector<std::shared_ptr<ObserveDataCom>> observeDatas2Opt;
	observeDatas2Opt.reserve(observeDatas_.size());
	for (int i = 0; i < observeDatas_.size(); ++i)
	{
		if (observeDatas_[i]->obs.size() < 5)
			continue;
		observeDatas2Opt.push_back(observeDatas_[i]);
	}

	using namespace OPTIMIZE_LYJ;
	int imgSz = extractDatas_.size();
	int pointSz = observeDatas2Opt.size();

	OptimizerLargeSparse optimizer;
	//OptimizeLargeSRBA optimizer;
	std::vector<std::shared_ptr<OptVarAbr<double>>> Tcws;
	std::vector<std::shared_ptr<OptVarAbr<double>>> Pws;
	std::vector<double> K(4);
	K[0] = cam_.fx();
	K[1] = cam_.fy();
	K[2] = cam_.cx();
	K[3] = cam_.cy();

	auto funcGeneratePointVertex = [&](Eigen::Vector3d& _Pw, uint64_t& _vId, bool _fix = false)
		{
			std::shared_ptr<OptVarAbr<double>> varPtr = std::make_shared<OptVarPoint3d>(_vId);
			varPtr->setData(_Pw.data());
			varPtr->setFixed(_fix);
			optimizer.addVariable(varPtr);
			Pws.push_back(varPtr);
			++_vId;
		};
	auto funcGeneratePoseVertex = [&](Eigen::Matrix<double, 3, 4>& _Tcw, uint64_t& _vId, bool _fix = false)
		{
			std::shared_ptr<OptVarAbr<double>> varPtr = std::make_shared<OptVarPose3d>(_vId);
			varPtr->setData(_Tcw.data());
			varPtr->setFixed(_fix);
			optimizer.addVariable(varPtr);
			Tcws.push_back(varPtr);
			++_vId;
		};


	auto funcGenerateUVFactor = [&](Eigen::Vector2d& _ob, uint64_t _vId1, uint64_t _vId2, uint64_t& _fId)
		{
			std::shared_ptr<OptFactorAbr<double>> factorPtr = std::make_shared<OptFactorUV_Pose3d_Point3d>(_fId);
			OptFactorUV_Pose3d_Point3d* factor = dynamic_cast<OptFactorUV_Pose3d_Point3d*>(factorPtr.get());
			factor->setObs(_ob.data(), K.data());
			std::vector<uint64_t> vIds;
			vIds.push_back(_vId1);
			vIds.push_back(_vId2);
			optimizer.addFactor(factorPtr, vIds);
			++_fId;
		};
	auto funcGenerateScaleFactor = [&](double _ob, uint64_t _vId1, uint64_t _vId2, uint64_t& _fId)
		{
			std::shared_ptr<OptFactorAbr<double>> factorPtr = std::make_shared<OptFactorScale_Pose3d_Point3d>(_fId);
			OptFactorScale_Pose3d_Point3d* factor = dynamic_cast<OptFactorScale_Pose3d_Point3d*>(factorPtr.get());
			factor->setObs(_ob);
			std::vector<uint64_t> vIds;
			vIds.push_back(_vId1);
			vIds.push_back(_vId2);
			optimizer.addFactor(factorPtr, vIds);
			++_fId;
		};


	Eigen::Matrix<double, 3, 4> TcwTmp;
	uint64_t vId = 0;
	for (int i = 0; i < imgSz; ++i)
	{
		extractDatas_[i].Tcw.getMatrix34d(TcwTmp);
		if (i == 0)
			funcGeneratePoseVertex(TcwTmp, vId, true);
		else
			funcGeneratePoseVertex(TcwTmp, vId);
	}
	for (int i = 0; i < pointSz; ++i)
	{
		funcGeneratePointVertex(observeDatas2Opt[i]->P3D, vId);
	}

	uint64_t fId = 0;
	bool sAdded = false;
	Eigen::Vector2d uvTmp;
	for (int i = 0; i < pointSz; ++i)
	{
		uint64_t varPId = i + imgSz;
		const auto& obs = observeDatas2Opt[i]->obs;
		const auto& obSz = obs.size();
		for (int j = 0; j < obSz; ++j)
		{
			const auto& imgId = obs[j](0);
			const auto& uvId = obs[j](1);
			uint64_t varTId = imgId;
			if (!sAdded)
			{
				const auto& Tcw = extractDatas_[imgId].Tcw;
				const auto& Pw = observeDatas2Opt[i]->P3D;
				Eigen::Vector3d Pc = Tcw * Pw;
				double ss = Pc.squaredNorm();
				funcGenerateScaleFactor(ss, varTId, varPId, fId);
				sAdded = true;
			}
			uvTmp(0) = extractDatas_[imgId].kps[uvId].pt.x;
			uvTmp(1) = extractDatas_[imgId].kps[uvId].pt.y;
			funcGenerateUVFactor(uvTmp, varTId, varPId, fId);
		}
	}

	optimizer.run();
	for (int i = 0; i < Tcws.size(); ++i)
	{
		OptVarPose3d* v = dynamic_cast<OptVarPose3d*>(Tcws[i].get());
		TcwTmp = v->getEigen();
		extractDatas_[i].Tcw = COMMON_LYJ::Pose3D(TcwTmp);
	}
	return true;
}

bool ProcessorWithMeshComAft::optimizeCeres()
{
	std::vector<std::shared_ptr<ObserveDataCom>> observeDatas2Opt;
	observeDatas2Opt.reserve(observeDatas_.size());
	for (int i = 0; i < observeDatas_.size(); ++i)
	{
		if (observeDatas_[i]->obs.size() < 5)
			continue;
		observeDatas2Opt.push_back(observeDatas_[i]);
	}

	using namespace OPTIMIZE_LYJ;
	int imgSz = extractDatas_.size();
	int mPairSz = matchDatas_.size();

	Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
	K(0, 0) = cam_.fx();
	K(1, 1) = cam_.fy();
	K(0, 2) = cam_.cx();
	K(1, 2) = cam_.cy();


	std::vector<Eigen::Matrix<double, 7, 1>> ceresPoses(imgSz);
	CeresProblem ceresPro;
	ceresPro.setMaxIter(10);
	for (int i = 0; i < imgSz; ++i)
	{
		const Eigen::Matrix3d& Rcw = extractDatas_[i].Tcw.getR();
		const Eigen::Vector3d& tcw = extractDatas_[i].Tcw.gett();
		Eigen::Quaterniond qcw(Rcw);
		ceresPoses[i][0] = qcw.w();
		ceresPoses[i][1] = qcw.x();
		ceresPoses[i][2] = qcw.y();
		ceresPoses[i][3] = qcw.z();
		ceresPoses[i][4] = tcw.x();
		ceresPoses[i][5] = tcw.y();
		ceresPoses[i][6] = tcw.z();
		if (i == 0)
			ceresPro.addPose3DParameter(ceresPoses[i].data(), true);
		else
			ceresPro.addPose3DParameter(ceresPoses[i].data(), false);
	}

	int pointSz = observeDatas2Opt.size();
	int allPSz = 0;
	Eigen::Vector2d uvd;
	for (int i = 0; i < pointSz; ++i)
	{
		ceresPro.addPoint3DParameter(observeDatas2Opt[i]->P3D.data(), false);
		const auto& obs = observeDatas2Opt[i]->obs;
		const auto& obSz = obs.size();
		for (int j = 0; j < obSz; ++j)
		{
			const auto& imgId = obs[j](0);
			const auto& uvId = obs[j](1);

			uvd(0) = extractDatas_[imgId].kps[uvId].pt.x;
			uvd(1) = extractDatas_[imgId].kps[uvId].pt.y;
			ceresPro.addUVFactor(uvd, K, ceresPoses[imgId].data(), observeDatas2Opt[i]->P3D.data(), 1);
		}
	}
	ceresPro.solve();

	for (int i = 0; i < imgSz; ++i)
	{
		Eigen::Quaterniond qcw;
		Eigen::Matrix3d& Rcw = extractDatas_[i].Tcw.getR();
		Eigen::Vector3d& tcw = extractDatas_[i].Tcw.gett();
		auto& pose = ceresPoses[i];
		qcw.w() = pose(0);
		qcw.x() = pose(1);
		qcw.y() = pose(2);
		qcw.z() = pose(3);
		Rcw = qcw.toRotationMatrix();
		tcw(0) = pose(4);
		tcw(1) = pose(5);
		tcw(2) = pose(6);
	}
	return true;
}







NSP_SLAM_LYJ_SRC_END
