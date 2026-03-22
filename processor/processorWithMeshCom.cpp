#include "processorWithMeshCom.h"
#include <common/Timer.h>
#include <common/FlannSearch.h> 

#include <IO/DataWin2D.h>
#include <debugger/debugger.h>

#include <Optimize_LYJ.h>
#include <Optimizer/optimizer.h>
#include <Factor/Factor.h>
#include <Variable/Variable.h>
#include <CeresCheck/CeresProblem/CeresProblem.h>

NSP_SLAM_LYJ_SRC_BEGIN

const static int TH_HIGH = 100;
const static int TH_LOW = 50;
const static int HISTO_LENGTH = 30;
// static int imgi = 0;
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
// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
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
static int matchByBoW(
	const std::vector<cv::KeyPoint>& _kps1, const std::vector<cv::KeyPoint>& _kps2,
	const cv::Mat& _desc1, const cv::Mat& _desc2,
	const DBoW3::FeatureVector& _fv1, const DBoW3::FeatureVector& _fv2,
	std::vector<int>& _matches2to1,
	COMMON_LYJ::Pose3D& _Tcw1, COMMON_LYJ::Pose3D& _Tcw2,
	std::vector<Eigen::Vector3f>& _P3Ds1, std::vector<Eigen::Vector3f>& _P3Ds2, const float& _squareDistTh, bool _lowTh = true, bool _checkOri = true
)
{
	bool mbCheckOrientation = _checkOri;
	float mfNNratio = 0.6;
	int size1 = _kps1.size();
	_matches2to1.assign(size1, -1);
	int nmatches = 0;
	std::vector<int> rotHist[HISTO_LENGTH];
	for (int i = 0; i < HISTO_LENGTH; i++)
		rotHist[i].reserve(500);
	const float factor = 1.0f / HISTO_LENGTH;

	// We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
	DBoW3::FeatureVector::const_iterator KFit = _fv2.begin();
	DBoW3::FeatureVector::const_iterator Fit = _fv1.begin();
	DBoW3::FeatureVector::const_iterator KFend = _fv2.end();
	DBoW3::FeatureVector::const_iterator Fend = _fv1.end();

	COMMON_LYJ::Pose3D Twc1 = _Tcw1.inversed();
	COMMON_LYJ::Pose3D Twc2 = _Tcw2.inversed();
	Eigen::Vector3f Pw1;
	Eigen::Vector3f Pw2;

	while (KFit != KFend && Fit != Fend)
	{
		if (KFit->first == Fit->first)
		{
			const std::vector<unsigned int> vIndicesKF = KFit->second;
			const std::vector<unsigned int> vIndicesF = Fit->second;

			for (size_t iKF = 0; iKF < vIndicesKF.size(); iKF++)
			{
				const unsigned int realIdxKF = vIndicesKF[iKF];
				const cv::Mat& dKF = _desc2.row(realIdxKF);

				if (!_P3Ds1.empty() && !_P3Ds2.empty() && realIdxKF < _P3Ds2.size() && _P3Ds2[realIdxKF](2) > 0)
					Pw2 = Twc2 * _P3Ds2[realIdxKF];
				if (!_P3Ds1.empty() && !_P3Ds2.empty() && realIdxKF < _P3Ds2.size() && _P3Ds2[realIdxKF](2) <= 0)
					continue;

				int bestDist1 = 256;
				int bestIdxF = -1;
				int bestDist2 = 256;
				for (size_t iF = 0; iF < vIndicesF.size(); iF++)
				{
					const unsigned int realIdxF = vIndicesF[iF];
					if (_matches2to1[realIdxF] != -1)
						continue;
					if (!_P3Ds1.empty() && !_P3Ds2.empty() && realIdxF < _P3Ds1.size() && _P3Ds1[realIdxF](2) > 0 && realIdxKF < _P3Ds2.size() && _P3Ds2[realIdxKF](2) > 0)
					{
						Pw1 = Twc1 * _P3Ds1[realIdxF];
						//std::cout << (Pw1 - Pw2).squaredNorm() << std::endl;
						if ((Pw1 - Pw2).squaredNorm() > _squareDistTh)
							continue;
					}
					if (!_P3Ds1.empty() && !_P3Ds2.empty() && realIdxF < _P3Ds1.size() && _P3Ds1[realIdxF](2) <= 0)
					{
						continue;
					}
					const cv::Mat& dF = _desc1.row(realIdxF);
					const int dist = DescriptorDistance(dKF, dF);
					if (dist < bestDist1)
					{
						bestDist2 = bestDist1;
						bestDist1 = dist;
						bestIdxF = realIdxF;
					}
					else if (dist < bestDist2)
					{
						bestDist2 = dist;
					}

				}

				bool mmm = false;
				if ((_lowTh && bestDist1 <= TH_LOW) || (!_lowTh && bestDist1 <= TH_LOW))
					mmm = true;
				if (mmm)
				{
					if (static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2))
					{
						_matches2to1[bestIdxF] = realIdxKF;


						if (mbCheckOrientation)
						{
							const cv::KeyPoint& kp = _kps2[realIdxKF];
							const cv::KeyPoint& Fkp = _kps1[bestIdxF];
							float rot = kp.angle - Fkp.angle;
							if (rot < 0.0)
								rot += 360.0f;
							int bin = round(rot * factor);
							if (bin == HISTO_LENGTH)
								bin = 0;
							assert(bin >= 0 && bin < HISTO_LENGTH);
							rotHist[bin].push_back(bestIdxF);
						}
						nmatches++;
					}
				}

			}

			KFit++;
			Fit++;
		}
		else if (KFit->first < Fit->first)
		{
			KFit = _fv2.lower_bound(Fit->first);
		}
		else
		{
			Fit = _fv1.lower_bound(KFit->first);
		}
	}

	if (mbCheckOrientation)
	{
		int ind1 = -1;
		int ind2 = -1;
		int ind3 = -1;

		ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

		for (int i = 0; i < HISTO_LENGTH; i++)
		{
			if (i == ind1 || i == ind2 || i == ind3)
				continue;
			for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
			{
				_matches2to1[rotHist[i][j]] = -1;
				nmatches--;
			}
		}
	}

	return nmatches;
}


ProcessorWithMeshCom::ProcessorWithMeshCom()
{
}
ProcessorWithMeshCom::~ProcessorWithMeshCom()
{
	if (proHandle_)
		CUDA_LYJ::release(proHandle_);
}


void ProcessorWithMeshCom::process(COMMON_LYJ::BaseTriMesh& _btm, std::vector<COMMON_LYJ::CompressedImage*>& _imgs, std::vector<COMMON_LYJ::Pose3D>& _Tcws, std::vector<COMMON_LYJ::PinholeCamera>& _cams, ProcessComOption _opt)
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
	match();
	dump2D("D:/tmp");
	for (int i = 0; i < 10; ++i)
	{
		generate3DInfos();
		if (!optimize(i))
			break;
	}
	for (int i = 0; i < Tcws_.size(); ++i)
	{
		_Tcws[i] = extractDatas_[i].Tcw;
	}
}

bool ProcessorWithMeshCom::extract()
{
	btm_->enableFCenters();
	btm_->calculateFCenters();
	btm_->enableFNormals();
	btm_->calculateFNormals();
	std::vector<float> camParams(4);
	camParams[0] = cam_.fx();
	camParams[1] = cam_.fy();
	camParams[2] = cam_.cx();
	camParams[3] = cam_.cy();
	int w = cam_.wide();
	int h = cam_.height();
	proHandle_ = CUDA_LYJ::initProjector(btm_->getVertexs()[0].data(), btm_->getVn(), btm_->getFCenters()[0].data(), btm_->getFNormals()[0].data(), btm_->getFaces()[0].vId_, btm_->getFn(), camParams.data(), w, h);
	if (!proHandle_)
		return false;
	proCaches_.resize(threadNum_);
	pBuffers_.resize(threadNum_);
	for (int i = 0; i < threadNum_; ++i)
	{
		proCaches_[i].init(btm_->getVn(), btm_->getFn(), w, h);
		pBuffers_[i].init(w, h, btm_);
	}

	int imgSize = imgs_.size();
	extractDatas_.resize(imgSize);
	for (int i = 0; i < imgSize; ++i)
	{
		extractDatas_[i].id = i;
		extractDatas_[i].cam = &cam_;
		extractDatas_[i].img = imgs_[i];
		extractDatas_[i].Tcw = Tcws_[i];
	}

	std::string debugPath = "D:/tmp";
	bool bDebugExtract = false;
	auto funcExtracOrb = [&](uint64_t _s, uint64_t _e, uint32_t _id) {
		cv::Mat imgCV;
		cv::Mat imgCVGray;
		cv::Ptr<cv::ORB> orb = cv::ORB::create(8192);
		for (int ind = _s; ind < _e; ++ind)
		{
			pBuffers_[_id].updateTcw(extractDatas_[ind].Tcw);
			CUDA_LYJ::project(proHandle_, proCaches_[_id], pBuffers_[_id].Tcw.data(), (float*)pBuffers_[_id].depthsM.data, pBuffers_[_id].fIds.data(), pBuffers_[_id].allVisiblePIds.data(), pBuffers_[_id].allVisibleFIds.data(), pBuffers_[_id].minD, pBuffers_[_id].maxD, pBuffers_[_id].csTh, pBuffers_[_id].detDTh);

			extractDatas_[ind].img->decompressCVMat(imgCV);
			cv::cvtColor(imgCV, imgCVGray, cv::COLOR_BGR2GRAY);
			cv::GaussianBlur(imgCVGray, imgCVGray, cv::Size(3, 3), 0);
			orb->detectAndCompute(imgCVGray, cv::Mat(), extractDatas_[ind].kps, extractDatas_[ind].descs);

			auto& P3Ds = extractDatas_[ind].P3Ds;
			auto& kps = extractDatas_[ind].kps;
			P3Ds.assign(kps.size(), Eigen::Vector3f(0, 0, 0));
			extractDatas_[ind].fIds.assign(kps.size(), -1);
			for (int i = 0; i < kps.size(); ++i)
			{
				int r = kps[i].pt.y;
				int c = kps[i].pt.x;
				const float& d = pBuffers_[_id].depthsM.at<float>(r, c);
				const unsigned int& fId = pBuffers_[_id].fIds[r * w + c];
				if (d == FLT_MAX || fId == UINT32_MAX)
					continue;
				extractDatas_[ind].cam->image2World(kps[i].pt.x, kps[i].pt.y, d, P3Ds[i]);
				extractDatas_[ind].fIds[i] = fId;
			}


			if (bDebugExtract)
			{
				cv::Mat depthsShow(h, w, CV_8UC1);
				depthsShow.setTo(0);
				for (int r = 0; r < h; ++r)
				{
					for (int c = 0; c < w; ++c)
					{
						float d = pBuffers_[_id].depthsM.at<float>(r, c);
						if (d == FLT_MAX)
							continue;
						int di = d / 10;
						depthsShow.at<uchar>(r, c) = (uchar)di;
					}
				}
				cv::imshow("depth", depthsShow);
				cv::waitKey();
				const std::vector<Eigen::Vector3f>& ctrs = btm_->getFCenters();
				std::vector<Eigen::Vector3f> ps;
				for (int r = 0; r < h; ++r)
				{
					for (int c = 0; c < w; ++c)
					{
						uint fId = pBuffers_[_id].fIds[r * w + c];
						if (fId == UINT32_MAX)
							continue;
						ps.push_back(ctrs[fId]);
					}
				}
				COMMON_LYJ::BaseTriMesh btmTmp;
				btmTmp.setVertexs(ps);
				COMMON_LYJ::writePLYMesh(debugPath + "/proFaces.ply", btmTmp);
				cv::Mat kpM;
				cv::drawKeypoints(imgCV, extractDatas_[ind].kps, kpM);
				cv::imshow("kps", kpM);
				cv::waitKey();
			}
		}
		};
	{
		auto t_start = std::chrono::high_resolution_clock::now();
		int thdNum = threadNum_;
		if (bDebugExtract)
			thdNum = 1;
		COMMON_LYJ::ThreadPool threadPool(thdNum);
		threadPool.processWithId(funcExtracOrb, 0, imgSize);
		std::cout << "extract time: "
			<< std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::high_resolution_clock::now() - t_start).count() << " ms\n";
	}

	return true;
}
bool ProcessorWithMeshCom::generatePairs()
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

	//voc
	voc_.reset(new DBoW3::Vocabulary(9, 3));
	voc_->load(opt_.vocPath);
	auto funcTransform = [&](uint64_t _s, uint64_t _e) {
		std::vector<cv::Mat> features;
		features.reserve(8192);
		for (int i = _s; i < _e; ++i)
		{
			features.clear();
			for (int j = 0; j < extractDatas_[i].descs.rows; ++j)
				features.push_back(extractDatas_[i].descs.row(j));
			voc_->transform(features, extractDatas_[i].bowVec, extractDatas_[i].featureVec, 0);
		}
		};
	{
		auto t_start = std::chrono::high_resolution_clock::now();
		COMMON_LYJ::ThreadPool threadPool(threadNum_);//thdNum_
		threadPool.process(funcTransform, 0, imgSize);
		std::cout << "transform time: "
			<< std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::high_resolution_clock::now() - t_start).count() << " ms\n";
	}
	vocDB_.reset(new DBoW3::Database(*voc_));
	for (int i = 0; i < imgSize; ++i)
		vocDB_->add(extractDatas_[i].bowVec);
	auto funcQury = [&](uint64_t _s, uint64_t _e) {
		DBoW3::QueryResults qRets;
		for (int i = _s; i < _e; ++i)
		{
			vocDB_->query(extractDatas_[i].bowVec, qRets, 20);
			for (int j = 0; j < qRets.size(); ++j)
			{
				if (qRets[j].Score > 0.5 && qRets[j].Id != i)
				{
					pairDatas_[i].status.setFlag(qRets[j].Id, true);
					pairDatas_[qRets[j].Id].status.setFlag(i, true);
				}
			}
		}
		};
	{
		auto t_start = std::chrono::high_resolution_clock::now();
		COMMON_LYJ::ThreadPool threadPool(threadNum_);
		threadPool.process(funcQury, 0, imgSize);
		std::cout << "qury time: "
			<< std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::high_resolution_clock::now() - t_start).count() << " ms\n";
	}


	//loc
	int imgSz = imgSize;
	std::vector<Eigen::Vector3f> locs(imgSz);
	for (int i = 0; i < imgSz; ++i)
	{
		COMMON_LYJ::Pose3D Twc = extractDatas_[i].Tcw.inversed();
		locs[i] = Twc.gett().cast<float>();
	}
	size_t nn = 10;
	float distSquareTh = 10000;
	COMMON_LYJ::FLANNWrapper<float, 3> flann;
	flann.build_index(locs[0].data(), imgSz);
	std::vector<int> inds(nn);
	std::vector<float> dists(nn);
	for (int i = 0; i < imgSz; ++i)
	{
		inds.assign(nn, -1);
		flann.batch_search<1>(locs[i].data(), nn, inds, dists);
		if (inds.empty())
			continue;
		for (int j = 0; j < inds.size(); ++j)
		{
			if (inds[j] == i || inds[j] == -1 || inds[j] < 0 || inds[j] >= imgSz)
				continue;
			if (dists[j] > distSquareTh)
				continue;
			pairDatas_[i].status.setFlag(inds[j], true);
			pairDatas_[inds[j]].status.setFlag(i, true);
		}
	}

	return true;
}
bool ProcessorWithMeshCom::match()
{
	int imgSize = imgs_.size();
	matchDatas_.reserve(50 * imgSize);
	uint64_t key;
	MatchData matchDataTmp;
	for (int i = 0; i < imgSize - 1; ++i)
	{
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
	}

	bool bDebugMatch = false;
	auto funcMatchMulti = [&](uint64_t _s, uint64_t _e) {
		std::vector<int> match2to1;
		for (int i = _s; i < _e; ++i)
		{
			int id1 = matchDatas_[i].id1;
			int id2 = matchDatas_[i].id2;
			std::vector<Eigen::Vector2i>& matches = matchDatas_[i].matches;
			int cnt = matchByBoW(extractDatas_[id1].kps, extractDatas_[id2].kps, 
				extractDatas_[id1].descs, extractDatas_[id2].descs, 
				extractDatas_[id1].featureVec, extractDatas_[id2].featureVec, 
				match2to1,
				extractDatas_[id1].Tcw, extractDatas_[id2].Tcw, extractDatas_[id1].P3Ds, extractDatas_[id2].P3Ds,
				10000, false, false);
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
				if (id1 == 9 && id2 == 12)
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
			}
		}
		};
	{
		auto t_start = std::chrono::high_resolution_clock::now();
		int thdNum = threadNum_;
		if (bDebugMatch)
			thdNum = 1;
		COMMON_LYJ::ThreadPool threadPool(thdNum);
		threadPool.process(funcMatchMulti, 0, matchDatas_.size());
		std::cout << "match time: "
			<< std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::high_resolution_clock::now() - t_start).count() << " ms\n";
	}

	return true;
}
void ProcessorWithMeshCom::dump2D(std::string _path)
{
	using namespace COMMON_LYJ;

	Data2DPoint data2DPoint;
	Data2DLine data2DLine;
	Data2DEdge data2DEdge;

	std::vector<std::vector<cv::Point>>& allKPs = data2DPoint.m_allKeyPoints;
	std::vector<std::vector<cv::Vec4f>>& allKLs = data2DLine.m_allKeyLines;
	std::vector<std::vector<cv::Point>>& allKEs = data2DEdge.m_allEdgePoints;
	int sz = extractDatas_.size();
	allKPs.resize(sz);
	allKLs.resize(sz);
	allKEs.resize(sz);
	std::vector<COMMON_LYJ::CompressedImage> comImgs(sz);
	for (int i = 0; i < sz; ++i)
	{
		auto imgData = &extractDatas_[i];
		const auto& kps = imgData->kps;
		int szP = kps.size();
		auto& kpsWin = allKPs[i];
		kpsWin.resize(szP);
		for (int j = 0; j < szP; ++j)
		{
			kpsWin[j].x = kps[j].pt.x;
			kpsWin[j].y = kps[j].pt.y;
		}
		//const auto& kls = imgData->vecLines;
		//int szL = kls.size();
		//auto& klsWin = allKLs[i];
		//klsWin = kls;
		//const auto& kes = imgData->edges;
		//int szE = kes.size();
		//auto& kesWin = allKEs[i];
		//kesWin.resize(szE);
		//for (int j = 0; j < szE; ++j)
		//{
		//	kesWin[j].x = kes[j](0);
		//	kesWin[j].y = kes[j](1);
		//}
		comImgs[i] = *imgData->img;
	}

	std::map<int64_t, std::vector<Mth>>& allPMs = data2DPoint.m_allPointMatches;
	std::map<int64_t, std::vector<Mth>>& allLMs = data2DLine.m_allLineMatches;
	std::map<int64_t, std::vector<Mth>>& allEMs = data2DEdge.m_allEdgeMatches;
	int szM = matchDatas_.size();
	for (const auto& mDatas : matchDatas_)
	{
		uint64_t ind = mDatas.pairId;
		const auto& mps = mDatas.matches;
		int mSz = mps.size();
		allPMs[ind].resize(mSz);
		memcpy(&allPMs[ind][0].first, mDatas.matches[0].data(), 2 * mSz * sizeof(int));
		allLMs[ind].resize(0);
		allEMs[ind].resize(0);
	}

	if (!stlplus::folder_exists(_path))
		stlplus::folder_create(_path);
	COMMON_LYJ::recordBin2DWithComImg(_path, comImgs, data2DPoint, data2DLine, data2DEdge);
}
bool ProcessorWithMeshCom::generate3DInfos()
{
	int imgSize = imgs_.size();
	int w = cam_.wide();
	int h = cam_.height();
	auto funcGenerate3D = [&](uint64_t _s, uint64_t _e, uint32_t _id) {
		for (int ind = _s; ind < _e; ++ind)
		{
			pBuffers_[_id].updateTcw(extractDatas_[ind].Tcw);
			CUDA_LYJ::project(proHandle_, proCaches_[_id], pBuffers_[_id].Tcw.data(), (float*)pBuffers_[_id].depthsM.data, pBuffers_[_id].fIds.data(), pBuffers_[_id].allVisiblePIds.data(), pBuffers_[_id].allVisibleFIds.data(), pBuffers_[_id].minD, pBuffers_[_id].maxD, pBuffers_[_id].csTh, pBuffers_[_id].detDTh);

			auto& P3Ds = extractDatas_[ind].P3Ds;
			auto& kps = extractDatas_[ind].kps;
			P3Ds.assign(kps.size(), Eigen::Vector3f(0, 0, 0));
			extractDatas_[ind].fIds.assign(kps.size(), -1);
			for (int i = 0; i < kps.size(); ++i)
			{
				int r = kps[i].pt.y;
				int c = kps[i].pt.x;
				const float& d = pBuffers_[_id].depthsM.at<float>(r, c);
				const unsigned int& fId = pBuffers_[_id].fIds[r * w + c];
				if (d == FLT_MAX || fId == UINT32_MAX)
					continue;
				extractDatas_[ind].cam->image2World(kps[i].pt.x, kps[i].pt.y, d, P3Ds[i]);
				extractDatas_[ind].fIds[i] = fId;
			}

		}
		};
	{
		auto t_start = std::chrono::high_resolution_clock::now();
		int thdNum = threadNum_;
		COMMON_LYJ::ThreadPool threadPool(thdNum);
		threadPool.processWithId(funcGenerate3D, 0, imgSize);
		std::cout << "generate 3D time: "
			<< std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::high_resolution_clock::now() - t_start).count() << " ms\n";
	}

	return true;
}
bool ProcessorWithMeshCom::optimize(int _i)
{
	using namespace OPTIMIZE_LYJ;
	int imgSz = extractDatas_.size();
	int mPairSz = matchDatas_.size();
	bool addPlane = true;

	//OptimizerLargeSparse optimizer;
	OptimizeLargeSRBA optimizer;
	optimizer.setMaxIter(1);
	uint64_t vId = 0;
	uint64_t ftrId = 0;
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
	auto funcGeneratePlaneFactor = [&](Eigen::Vector4d& _ob, uint64_t _vId, uint64_t& _fId)
		{
			std::shared_ptr<OptFactorAbr<double>> factorPtr = std::make_shared<OptFactorPlane_Point3d>(_fId);
			OptFactorPlane_Point3d* factor = dynamic_cast<OptFactorPlane_Point3d*>(factorPtr.get());
			factor->setObs(_ob.data());
			std::vector<uint64_t> vIds;
			vIds.push_back(_vId);
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
	for (int i = 0; i < imgSz; ++i)
	{
		extractDatas_[i].Tcw.getMatrix34d(TcwTmp);
		if (i == 0)
		//if(!addPlane && i ==0)
			funcGeneratePoseVertex(TcwTmp, vId, true);
		else
			funcGeneratePoseVertex(TcwTmp, vId);
	}

	const auto& fNvrs = btm_->getFNormals();
	const auto& fCtrs = btm_->getFCenters();
	Eigen::Vector3f Pwf;
	Eigen::Vector3d Pwd;
	Eigen::Vector2d uvd;
	Eigen::Vector4d planewd;
	uint64_t vPId = vId;
	bool sAdded = false;
	for (int i = 0; i < mPairSz; ++i)
	{
		if (matchDatas_[i].matches.empty())
			continue;
		int imgId1 = matchDatas_[i].id1;
		int imgId2 = matchDatas_[i].id2;
		auto& ms = matchDatas_[i].matches;
		int mSz = ms.size();
		for (int j = 0; j < mSz; ++j)
		{
			int kpId1 = ms[j](0);
			int fId1 = extractDatas_[imgId1].fIds[kpId1];
			int kpId2 = ms[j](1);
			int fId2 = extractDatas_[imgId2].fIds[kpId2];
			if (fId1 == -1 || fId2 == -1)
				continue;
			const auto& c1 = fCtrs[fId1];
			const auto& n1 = fNvrs[fId1];
			const auto& c2 = fCtrs[fId2];
			const auto& n2 = fNvrs[fId2];
			if ((c1 - c2).norm() > (100.0 / (_i+1)))
				continue;
			Pwf = c1;
			Pwd = Pwf.cast<double>();
			funcGeneratePointVertex(Pwd, vId);


			if (!addPlane && !sAdded)
			{
				const auto& Tcw = extractDatas_[imgId1].Tcw;
				Eigen::Vector3d Pc = Tcw * Pwd;
				double ss = Pc.squaredNorm();
				funcGenerateScaleFactor(ss, imgId1, vPId, ftrId);
				sAdded = true;
			}
			else
			{
				uvd(0) = extractDatas_[imgId1].kps[kpId1].pt.x;
				uvd(1) = extractDatas_[imgId1].kps[kpId1].pt.y;
				funcGenerateUVFactor(uvd, imgId1, vPId, ftrId);
			}
			uvd(0) = extractDatas_[imgId2].kps[kpId2].pt.x;
			uvd(1) = extractDatas_[imgId2].kps[kpId2].pt.y;
			funcGenerateUVFactor(uvd, imgId2, vPId, ftrId);
			if (addPlane)
			{
				planewd.block(0, 0, 3, 1) = n1.cast<double>();
				planewd(3) = -1 * n1.dot(c1);
				funcGeneratePlaneFactor(planewd, vPId, ftrId);
			}

			vPId = vId;
		}
	}

	optimizer.run();
	double err = optimizer.getError();
	if (err >= lastErr_)
		return false;
	for (int i = 0; i < Tcws.size(); ++i)
	{
		OptVarPose3d* v = dynamic_cast<OptVarPose3d*>(Tcws[i].get());
		TcwTmp = v->getEigen();
		extractDatas_[i].Tcw = COMMON_LYJ::Pose3D(TcwTmp);
	}
	lastErr_ = err;
	return true;
}

bool ProcessorWithMeshCom::optimize2()
{
	using namespace OPTIMIZE_LYJ;
	int imgSz = extractDatas_.size();
	int mPairSz = matchDatas_.size();

	Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
	K(0,0) = cam_.fx();
	K(1,1) = cam_.fy();
	K(0,2) = cam_.cx();
	K(1,2) = cam_.cy();


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
		if(i == 0)
			ceresPro.addPose3DParameter(ceresPoses[i].data(), true);
		else
			ceresPro.addPose3DParameter(ceresPoses[i].data(), false);
	}

	const auto& fNvrs = btm_->getFNormals();
	const auto& fCtrs = btm_->getFCenters();
	Eigen::Vector3f Pwf;
	Eigen::Vector3d Pwd;
	Eigen::Vector2d uvd;
	Eigen::Vector4d planewd;
	std::vector<Eigen::Vector3d> ceresPoints;
	int allPSz = 0;
	for (int i = 0; i < mPairSz; ++i)
	{
		if (matchDatas_[i].matches.empty())
			continue;
		int imgId1 = matchDatas_[i].id1;
		int imgId2 = matchDatas_[i].id2;
		auto& ms = matchDatas_[i].matches;
		int mSz = ms.size();
		allPSz += mSz;
	}
	ceresPoints.reserve(allPSz);
	for (int i = 0; i < mPairSz; ++i)
	{
		if (matchDatas_[i].matches.empty())
			continue;
		int imgId1 = matchDatas_[i].id1;
		int imgId2 = matchDatas_[i].id2;
		auto& ms = matchDatas_[i].matches;
		int mSz = ms.size();
		for (int j = 0; j < mSz; ++j)
		{
			int kpId1 = ms[j](0);
			int fId1 = extractDatas_[imgId1].fIds[kpId1];
			int kpId2 = ms[j](1);
			int fId2 = extractDatas_[imgId2].fIds[kpId2];
			const auto& c1 = fCtrs[fId1];
			const auto& n1 = fNvrs[fId1];
			const auto& c2 = fCtrs[fId2];
			const auto& n2 = fNvrs[fId2];
			if (fId1 == -1)
				continue;
			Pwf = c1;
			Pwd = Pwf.cast<double>();
			ceresPoints.push_back(Pwd);
			ceresPro.addPoint3DParameter(ceresPoints.back().data(), false);

			uvd(0) = extractDatas_[imgId1].kps[kpId1].pt.x;
			uvd(1) = extractDatas_[imgId1].kps[kpId1].pt.y;
			ceresPro.addUVFactor(uvd, K, ceresPoses[imgId1].data(), ceresPoints.back().data(), 1);
			uvd(0) = extractDatas_[imgId2].kps[kpId2].pt.x;
			uvd(1) = extractDatas_[imgId2].kps[kpId2].pt.y;
			ceresPro.addUVFactor(uvd, K, ceresPoses[imgId2].data(), ceresPoints.back().data(), 1);
			planewd.block(0, 0, 3, 1) = n1.cast<double>();
			planewd(3) = -1 * n1.dot(c1);
			ceresPro.addPlaneFactor(planewd, ceresPoints.back().data(), 1);

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