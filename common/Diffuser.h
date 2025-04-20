#ifndef SLAM_LYJ_DIFFSER_H
#define SLAM_LYJ_DIFFSER_H

#include "base/Base.h"

NSP_SLAM_LYJ_MATH_BEGIN


class Diffuser2D
{
public:
	Diffuser2D(const int _w, const int _h, const std::vector<Eigen::Vector2i>& _blocks= std::vector<Eigen::Vector2i>(), const Eigen::Vector2i& _src=Eigen::Vector2i(0, 0), const int _level=-1);
	~Diffuser2D();

	void reset(const Eigen::Vector2i& _src, const std::vector<Eigen::Vector2i>& _blocks = std::vector<Eigen::Vector2i>(),const int _level = -1);
	bool next(std::vector<Eigen::Vector2i>& _locs);
	void addBlock(const Eigen::Vector2i& _block);
	void getDiffuseFrom(const Eigen::Vector2i& _curLoc, std::vector<Eigen::Vector2i>& _fromLocs);

private:
	void addLoc(const Eigen::Vector2i& _loc, std::vector<Eigen::Vector2i>& _locs);

private:
	int m_w = 0;
	int m_h = 0;
	int m_level = 0;
	Eigen::Vector2i m_src = Eigen::Vector2i::Zero();
	int m_curLevel = 1;
	std::vector<std::vector<char>> m_graph;
};




NSP_SLAM_LYJ_MATH_END

#endif // !SLAM_LYJ_DIFFSER_H
