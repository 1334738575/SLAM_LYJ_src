#include "PatchMatcher.h"
#include "debugger/debugger.h"
#include "common/Diffuser.h"

NSP_SLAM_LYJ_BEGIN

PatchMatcher::PatchMatcher(Option _opt)
	:MatcherAbr(TYPE::PATCH), m_opt(_opt)
{
}
PatchMatcher::~PatchMatcher()
{}
int PatchMatcher::match(const Frame& _frame1, const Frame& _frame2, std::vector<int>& _match2to1)
{
	const auto& kps1 = _frame1.getKeyPoints();
	const auto& kps2 = _frame2.getKeyPoints();
	return 0;
}
int PatchMatcher::matchPatch(const cv::Mat& _m1, std::vector<cv::KeyPoint>& _kps1,
	const cv::Mat& _m2, const bool _bGuass,
	std::vector<PatchMatchResult>& _matches)
{
	cv::Mat m1, m2;
	if (_bGuass) {
		cv::GaussianBlur(_m1, m1, cv::Size(3, 3), 10, 20);
		cv::GaussianBlur(_m2, m2, cv::Size(3, 3), 10, 20);
	}
	else {
		m1 = _m1.clone();
		m2 = _m2.clone();
	}
	// //do outside
	//while(m1.cols > 800 || m1.rows > 600)
	//	cv::pyrDown(m1, m1);
	//while (m2.cols > 800 || m2.rows > 600)
	//	cv::pyrDown(m2, m2);

	if (_kps1.empty())
		initPoints(m1, _kps1);
	std::shared_ptr<SLAM_LYJ_MATH::Grid2Df> grid = generateGrid(m1.cols, m1.rows, _kps1);
	initMatchResults(_kps1, grid, _matches);
	initOffsets(m1, m2, grid, _matches, m_opt.candOffsetSize);

	int gw = grid->getGridSize(0);
	int gh = grid->getGridSize(1);
	int gSize = gw * gh;
	float th = m_opt.th;
	th = 10;
	int searchR = m_opt.searchR;
	searchR = 10;
	int step = m_opt.searchR / m_opt.maxIterNum;
	for (int it = 0; it < m_opt.maxIterNum; ++it) {
		//searchR = m_opt.searchR - it * step;
		if (it % 2 == 0) {
			for (int i = 0; i < gh; ++i) {
				for (int j = 0; j < gw; ++j) {
					int id = i * gw + j;
					if (_matches[id].id1 == -1)	continue;
					Eigen::Vector2f offset = _matches[id].offset;
					float d = _matches[id].d;
					bool valid = findMatchPoint(m1, m2, _matches[id].p1, offset, d, searchR, th);

					int idl1 = id - 1;
					if (idl1 >= 0 && idl1 < gSize && _matches[idl1].id1 != -1) {
						Eigen::Vector2f offsetl1 = _matches[idl1].offset;
						if (findMatchPoint(m1, m2, _matches[id].p1, offsetl1, d, searchR, th)) {
							offset = offsetl1;
							valid = true;
						}
					}

					int idu1 = id - gw;
					if (idu1 >= 0 && idu1 < gSize && _matches[idu1].id1 != -1) {
						Eigen::Vector2f offsetu1 = _matches[idu1].offset;
						if (findMatchPoint(m1, m2, _matches[id].p1, offsetu1, d, searchR, th)) {
							offset = offsetu1;
							valid = true;
						}
					}

					int idl1u1 = id - gw - 1;
					if (idl1u1 >= 0 && idl1u1 < gSize && _matches[idl1u1].id1 != -1) {
						Eigen::Vector2f offsetl1u1 = _matches[idl1u1].offset;
						if (findMatchPoint(m1, m2, _matches[id].p1, offsetl1u1, d, searchR, th)) {
							offset = offsetl1u1;
							valid = true;
						}
					}

					int idr1u1 = id - gw + 1;
					if (idr1u1 >= 0 && idr1u1 < gSize && _matches[idr1u1].id1 != -1) {
						Eigen::Vector2f offsetr1u1 = _matches[idr1u1].offset;
						if (findMatchPoint(m1, m2, _matches[id].p1, offsetr1u1, d, searchR, th)) {
							offset = offsetr1u1;
							valid = true;
						}
					}

					_matches[id].offset = offset;
					_matches[id].d = d;
					_matches[id].valid = valid;
					if (_matches[id].valid && false) {
						cv::Mat mShow;
						std::vector<PatchMatchResult> matches;
						matches.push_back(_matches[id]);
						SLAM_LYJ_DEBUGGER::drawPatchMatches(m1, m2, matches, mShow, cv::Scalar(255), 1);
						cv::imshow("one match", mShow);
						cv::waitKey();
					}
				}
			}
		}
		else {
			for (int i = gh-1; i >= 0; --i) {
				for (int j = gw-1; j >= 0; --j) {
					int id = i * gw + j;
					if (_matches[id].id1 == -1)	continue;
					Eigen::Vector2f offset = _matches[id].offset;
					float d = _matches[id].d;
					bool valid = findMatchPoint(m1, m2, _matches[id].p1, offset, d, searchR, th);

					int idr1 = id + 1;
					if (idr1 >= 0 && idr1 < gSize && _matches[idr1].id1 != -1) {
						Eigen::Vector2f offsetr1 = _matches[idr1].offset;
						if (findMatchPoint(m1, m2, _matches[id].p1, offsetr1, d, searchR, th)) {
							offset = offsetr1;
							valid = true;
						}
					}

					int idd1 = id + gw;
					if (idd1 >= 0 && idd1 < gSize && _matches[idd1].id1 != -1) {
						Eigen::Vector2f offsetd1 = _matches[idd1].offset;
						if (findMatchPoint(m1, m2, _matches[id].p1, offsetd1, d, searchR, th)) {
							offset = offsetd1;
							valid = true;
						}
					}

					int idr1d1 = id - gw - 1;
					if (idr1d1 >= 0 && idr1d1 < gSize && _matches[idr1d1].id1 != -1) {
						Eigen::Vector2f offsetr1d1 = _matches[idr1d1].offset;
						if (findMatchPoint(m1, m2, _matches[id].p1, offsetr1d1, d, searchR, th)) {
							offset = offsetr1d1;
							valid = true;
						}
					}

					int idl1d1 = id - gw + 1;
					if (idl1d1 >= 0 && idl1d1 < gSize && _matches[idl1d1].id1 != -1) {
						Eigen::Vector2f offsetl1d1 = _matches[idl1d1].offset;
						if (findMatchPoint(m1, m2, _matches[id].p1, offsetl1d1, d, searchR, th)) {
							offset = offsetl1d1;
							valid = true;
						}
					}

					_matches[id].offset = offset;
					_matches[id].d = d;
					_matches[id].valid = valid;
					if (_matches[id].valid && false) {
						cv::Mat mShow;
						std::vector<PatchMatchResult> matches;
						matches.push_back(_matches[id]);
						SLAM_LYJ_DEBUGGER::drawPatchMatches(m1, m2, matches, mShow, cv::Scalar(255), 1);
						cv::imshow("one match", mShow);
						cv::waitKey();
					}
				}
			}
		}
	}
	cv::Mat mShow;
	SLAM_LYJ_DEBUGGER::drawPatchMatches(m1, m2, _matches, mShow, cv::Scalar(255), 1);
	cv::imshow("patch matches", mShow);
	cv::waitKey();
	return 0;
}
void PatchMatcher::initPoints(const cv::Mat& _m, std::vector<cv::KeyPoint>& _kps)
{
	//cv::Ptr<cv::FastFeatureDetector> faster = cv::FastFeatureDetector::create(20, true);
	//std::vector<cv::KeyPoint> kps;
	//faster->detect(_m, kps);
	//cv::Mat m1 = _m.clone();
	//cv::drawKeypoints(m1, kps, m1);
	//cv::imshow("fast1", m1);
	std::vector<cv::KeyPoint> kps2;
	cv::FAST(_m, kps2, 20);
	cv::Mat m2 = _m.clone();
	cv::drawKeypoints(m2, kps2, m2);
	cv::imshow("fast2", m2);
	cv::waitKey();
	SLAM_LYJ_MATH::Grid2Df grid(20, _m.rows, _m.cols, kps2);
	int gCols = grid.getGridSize(0);
	int gRows = grid.getGridSize(1);
	for (int i = 0; i < gRows; ++i) {
		for (int j = 0; j < gRows; ++j) {
			std::list<int> inds = grid.getInds(i, j);
			float resp = 0;
			int indMax = -1;
			for (const auto& ind : inds) {
				if (kps2[ind].response > resp) {
					resp = kps2[ind].response;
					indMax = ind;
				}
			}
			if (indMax != -1)
				_kps.push_back(kps2[indMax]);
		}
	}
	cv::Mat m3 = _m.clone();
	cv::drawKeypoints(m3, _kps, m3);
	cv::imshow("fast3", m3);
	cv::waitKey();
}
std::shared_ptr<SLAM_LYJ_MATH::Grid2Df> PatchMatcher::generateGrid(const int _w, const int _h, 
	const std::vector<cv::KeyPoint>& _kps)
{
	std::shared_ptr<SLAM_LYJ_MATH::Grid2Df> grid;
	grid.reset(new SLAM_LYJ_MATH::Grid2Df(m_opt.resolution, _h, _w, _kps));
	return grid;
}
void PatchMatcher::initMatchResults(const std::vector<cv::KeyPoint>& _kps1, std::shared_ptr<SLAM_LYJ_MATH::Grid2Df> _grid, std::vector<PatchMatchResult>& _matches)
{
	_matches.clear();
	int gw = _grid->getGridSize(0);
	int gh = _grid->getGridSize(1);
	_matches.resize(gw * gh);

	for (int i = 0; i < gh; ++i) {
		for (int j = 0; j < gw; ++j) {
			std::list<int> inds = _grid->getInds(i, j);
			if (inds.empty())
				continue;
			int bestInd = -1;
			float bestResp = -1;
			for (const int ind : inds) {
				if (bestResp < _kps1[ind].response) {
					bestInd = ind;
					bestResp = _kps1[ind].response;
				}
			}
			int id = i * gw + j;
			_matches[id].p1(0) = _kps1[bestInd].pt.x;
			_matches[id].p1(1) = _kps1[bestInd].pt.y;
			_matches[id].d = FLT_MAX;
			_matches[id].id1 = bestInd;
		}
	}
}
void PatchMatcher::initOffsets(const cv::Mat& _m1, const cv::Mat& _m2, 
	std::shared_ptr<SLAM_LYJ_MATH::Grid2Df> _grid, std::vector<PatchMatchResult>& _matches, const int _candSize)
{
	int gw = _grid->getGridSize(0);
	int gh = _grid->getGridSize(1);
	int gSize = gw * gh;
	float th = m_opt.th;
	int searchR = m_opt.searchR;
	std::vector<std::pair<int, float>> cands(gSize);
	for (int i = 0; i < gh; ++i) {
		for (int j = 0; j < gw; ++j) {
			int id = i * gw + j;
			if (_matches[id].id1 == -1) {
				cands[id].first = id;
				cands[id].second = FLT_MAX;
				continue;
			}
			_matches[id].valid = findMatchPoint(_m1, _m2, _matches[id].p1, _matches[id].offset, _matches[id].d, searchR, th);
			if (_matches[id].valid) {
				cands[id].first = id;
				cands[id].second = _matches[id].d;
			}
			else {
				cands[id].first = id;
				cands[id].second = FLT_MAX;
			}
		}
	}
	std::sort(cands.begin(), cands.end(), [](const std::pair<int, float>& _v1, const std::pair<int, float>& _v2) {
		return _v1.second <= _v2.second;
		});
	searchR = 10;
	th = 10;
	SLAM_LYJ_MATH::Diffuser2D diffuser(gw, gh);
	int bestCandi = -1;
	int maxCnt = 0;
	for (int candi = 0; candi < _candSize; ++candi) {
		if (cands[candi].second == FLT_MAX)
			continue;
		std::vector<Eigen::Vector2i> locs;
		std::vector<Eigen::Vector2i> froms;
		Eigen::Vector2i sss = _grid->toGridCoord2Df(_matches[cands[candi].first].p1(0), _matches[cands[candi].first].p1(1));
		Eigen::Vector2f offset = _matches[cands[candi].first].offset;
		bool bfind = false;
		float d = FLT_MAX;
		diffuser.reset(sss);
		int cnt = 0;
		std::vector<PatchMatchResult> matchesTmp = _matches;
		while (diffuser.next(locs)) {
			for (int i = 0; i < locs.size(); ++i) {
				int id = locs[i](1) * gw + locs[i](0);
				if (matchesTmp[id].id1 == -1)	continue;
				diffuser.getDiffuseFrom(locs[i], froms);
				bfind = false;
				d = FLT_MAX;
				for (int j = 0; j < froms.size(); ++j) {
					int id2 = froms[j](1) * gw + froms[j](0);
					offset = matchesTmp[id2].offset;
					bool valid = findMatchPoint(_m1, _m2, matchesTmp[id].p1, offset, d, searchR, th);
					if (valid) {
						matchesTmp[id].offset = offset;
						matchesTmp[id].d = d;
						matchesTmp[id].valid = valid;
						bfind = true;
					}
				}
				if (!bfind) {
					diffuser.addBlock(locs[i]);
					continue;
				}
				++cnt;
			}
		}
		if (cnt > maxCnt) {
			maxCnt = cnt;
			bestCandi = candi;
		}
	}
	if (bestCandi != -1) {
		std::vector<Eigen::Vector2i> locs;
		std::vector<Eigen::Vector2i> froms;
		Eigen::Vector2i sss = _grid->toGridCoord2Df(_matches[cands[bestCandi].first].p1(0), _matches[cands[bestCandi].first].p1(1));
		Eigen::Vector2f offset = _matches[cands[bestCandi].first].offset;
		bool bfind = false;
		float d = FLT_MAX;
		diffuser.reset(sss);
		std::vector<PatchMatchResult>& matchesTmp = _matches;
		while (diffuser.next(locs)) {
			for (int i = 0; i < locs.size(); ++i) {
				int id = locs[i](1) * gw + locs[i](0);
				if (matchesTmp[id].id1 == -1)	continue;
				diffuser.getDiffuseFrom(locs[i], froms);
				bfind = false;
				d = FLT_MAX;
				for (int j = 0; j < froms.size(); ++j) {
					int id2 = froms[j](1) * gw + froms[j](0);
					offset = matchesTmp[id2].offset;
					bool valid = findMatchPoint(_m1, _m2, matchesTmp[id].p1, offset, d, searchR, th);
					if (valid) {
						matchesTmp[id].offset = offset;
						matchesTmp[id].d = d;
						matchesTmp[id].valid = valid;
						bfind = true;
					}
				}
				if (!bfind) {
					diffuser.addBlock(locs[i]);
					continue;
				}
			}
		}
	}
}
bool PatchMatcher::findMatchPoint(const cv::Mat& _m1, const cv::Mat& _m2,
	const Eigen::Vector2f& _p1, Eigen::Vector2f& _offset, float& _bestD,
	const int _r, const float _th)
{
	bool bFind = false;
	if (_m1.type() != _m2.type()) {
		std::cout << __FUNCTION__ << "type of m1 is not equal to m2" << std::endl;
		return bFind;
	}
	const int& x1 = _p1(0);
	const int& y1 = _p1(1);
	int x2 = x1 + _offset(0);
	int y2 = y1 + _offset(1);
	int x2min = std::clamp(x2 - _r, 0, _m2.cols - 1);
	int x2max = std::clamp(x2 + _r, 0, _m2.cols - 1);
	int y2min = std::clamp(y2 - _r, 0, _m2.rows - 1);
	int y2max = std::clamp(y2 + _r, 0, _m2.rows - 1);
	if (_m1.type() == CV_8UC1) {
		const uchar& v1 = _m1.at<uchar>(y1, x1);
		for (int i = y2min; i < y2max; ++i) {
			for (int j = x2min; j < x2max; ++j) {
				const uchar& v2 = _m2.at<uchar>(i, j);
				float d = std::abs(v1 - v2);
				if (d < _th && d <= _bestD) {
					_bestD = d;
					_offset(0) = j - x1;
					_offset(1) = i - y1;
					bFind = true;
				}
			}
		}
	}
	else if (_m1.type() == CV_8UC3) {
		const cv::Vec3b& v1 = _m1.at<cv::Vec3b>(y1, x1);
		for (int i = y2min; i < y2max; ++i) {
			for (int j = x2min; j < x2max; ++j) {
				const cv::Vec3b& v2 = _m2.at<cv::Vec3b>(i, j);
				float d = 0;
				for(int k=0;k<3;++k)
					d += std::abs(v1[k] - v2[k]);
				if (d < _th && d <= _bestD) {
					_bestD = d;
					_offset(0) = j - x1;
					_offset(1) = i - y1;
					bFind = true;
				}
			}
		}
	}
	return bFind;
}

void PatchMatcher::resetMatchResults(std::vector<PatchMatchResult>& _matches)
{
	for (auto& match : _matches) {
		match.d = FLT_MAX;
		match.offset.setZero();
		match.valid = false;
	}
}


NSP_SLAM_LYJ_END