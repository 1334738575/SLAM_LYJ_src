#include "Diffuser.h"

NSP_SLAM_LYJ_MATH_BEGIN


Diffuser2D::Diffuser2D(const int _w, const int _h, const std::vector<Eigen::Vector2i>& _blocks, const Eigen::Vector2i& _src, const int _level)
	:m_w(_w), m_h(_h)
{
	m_graph.resize(m_h);
	reset(_src, _blocks, _level);
}
Diffuser2D::~Diffuser2D()
{
}

void Diffuser2D::reset(const Eigen::Vector2i& _src, const std::vector<Eigen::Vector2i>& _blocks, const int _level)
{
	m_level = _level;
	m_src = _src;
	m_curLevel = 1;
	if (m_level == -1) {
		m_level = std::max(m_level, m_src(0));
		m_level = std::max(m_level, m_w - 1 - m_src(0));
		m_level = std::max(m_level, m_src(1));
		m_level = std::max(m_level, m_h - 1 - m_src(1));
	}
	for (int i = 0; i < m_h; ++i)
		m_graph[i].assign(m_w, 0);
	for (const auto& block : _blocks)
		m_graph[block(1)][block(0)] = -1;
	m_graph[m_src(1)][m_src(0)] = 1;
}

bool Diffuser2D::next(std::vector<Eigen::Vector2i>& _locs)
{
	_locs.clear();
	if (m_level != -1 && m_curLevel > m_level)
		return false;
	_locs.reserve(8 * m_curLevel);
	int minx = std::clamp(m_src(0) - m_curLevel, 0, m_w - 1);
	int maxx = std::clamp(m_src(0) + m_curLevel, 0, m_w - 1);
	int miny = std::clamp(m_src(1) - m_curLevel, 0, m_h - 1);
	int maxy = std::clamp(m_src(1) + m_curLevel, 0, m_h - 1);
	for (int i = minx; i <= maxx; ++i) {
		if (m_graph[miny][i] == 0) {
			m_graph[miny][i] = 1;
			_locs.push_back(Eigen::Vector2i(i, miny));
		}
		if (m_graph[maxy][i] == 0) {
			m_graph[maxy][i] = 1;
			_locs.push_back(Eigen::Vector2i(i, maxy));
		}
	}
	for (int i = miny; i <= maxy; ++i) {
		if (m_graph[i][minx] == 0) {
			m_graph[i][minx] = 1;
			_locs.push_back(Eigen::Vector2i(minx, i));
		}
		if (m_graph[i][maxx] == 0) {
			m_graph[i][maxx] = 1;
			_locs.push_back(Eigen::Vector2i(maxx, i));
		}
	}
	//std::cout << m_level << std::endl;
	//for (int i = 0; i < m_h; ++i) {
	//	for (int j = 0; j < m_w; ++j) {
	//		std::cout << int(m_graph[i][j]) << " ";
	//	}
	//	std::cout << std::endl;
	//}
	++m_curLevel;
	return true;
}

void Diffuser2D::addBlock(const Eigen::Vector2i& _block)
{
	if (_block(0) < 0 || _block(0) >= m_w || _block(1) < 0 || _block(1) >= m_h)
		return;
	m_graph[_block(1)][_block(0)] = -1;
}

void Diffuser2D::getDiffuseFrom(const Eigen::Vector2i& _curLoc, std::vector<Eigen::Vector2i>& _fromLocs)
{
	_fromLocs.clear();
	int level1 = m_curLevel - 1;
	if (level1 < 1)
		return;
	_fromLocs.reserve(3);
	if (level1 == 1) {
		_fromLocs.push_back(m_src);
		return;
	}
	//int level2 = level1 - 1;
	int detx = _curLoc(0) - m_src(0);
	int dety = _curLoc(1) - m_src(1);
	int det = std::abs(detx) + std::abs(dety);
	Eigen::Vector2i tmp;
	if (det == 2 * level1) {
		if (detx == level1)	tmp(0) = _curLoc(0) - 1;
		else tmp(0) = _curLoc(0) + 1;
		if (dety == level1)	tmp(1) = _curLoc(1) - 1;
		else tmp(1) = _curLoc(1) + 1;
		addLoc(tmp, _fromLocs);
	}
	else if (det == 2 * level1 - 1) {
		if (detx == level1) {
			if (dety == level1 - 1) {
				tmp(0) = _curLoc(0) - 1;
				tmp(1) = _curLoc(1);
				addLoc(tmp, _fromLocs);
				tmp(1) = _curLoc(1) - 1;
				addLoc(tmp, _fromLocs);
			}
			else if (dety == -1 * (level1 - 1)) {
				tmp(0) = _curLoc(0) - 1;
				tmp(1) = _curLoc(1);
				addLoc(tmp, _fromLocs);
				tmp(1) = _curLoc(1) + 1;
				addLoc(tmp, _fromLocs);
			}
		}
		else if (detx == -1 * level1) {
			if (dety == level1 - 1) {
				tmp(0) = _curLoc(0) + 1;
				tmp(1) = _curLoc(1);
				addLoc(tmp, _fromLocs);
				tmp(1) = _curLoc(1) - 1;
				addLoc(tmp, _fromLocs);
			}
			else if (dety == -1 * (level1 - 1)) {
				tmp(0) = _curLoc(0) + 1;
				tmp(1) = _curLoc(1);
				addLoc(tmp, _fromLocs);
				tmp(1) = _curLoc(1) + 1;
				addLoc(tmp, _fromLocs);
			}
		}
		else if (dety == level1) {
			if (detx == level1 - 1) {
				tmp(0) = _curLoc(0);
				tmp(1) = _curLoc(1) - 1;
				addLoc(tmp, _fromLocs);
				tmp(0) = _curLoc(0) - 1;
				addLoc(tmp, _fromLocs);
			}
			else if (detx == -1 * (level1 - 1)) {
				tmp(0) = _curLoc(0);
				tmp(1) = _curLoc(1) - 1;
				addLoc(tmp, _fromLocs);
				tmp(0) = _curLoc(0) + 1;
				addLoc(tmp, _fromLocs);
			}
		}
		else if (dety == -1 * level1) {
			if (detx == level1 - 1) {
				tmp(0) = _curLoc(0);
				tmp(1) = _curLoc(1) + 1;
				addLoc(tmp, _fromLocs);
				tmp(0) = _curLoc(0) - 1;
				addLoc(tmp, _fromLocs);
			}
			else if (detx == -1 * (level1 - 1)) {
				tmp(0) = _curLoc(0);
				tmp(1) = _curLoc(1) + 1;
				addLoc(tmp, _fromLocs);
				tmp(0) = _curLoc(0) + 1;
				addLoc(tmp, _fromLocs);
			}
		}
	}
	else {
		if (detx == level1) {
			tmp(0) = _curLoc(0) - 1;
			tmp(1) = _curLoc(1);
			addLoc(tmp, _fromLocs);
			tmp(1) = _curLoc(1) - 1;
			addLoc(tmp, _fromLocs);
			tmp(1) = _curLoc(1) + 1;
			addLoc(tmp, _fromLocs);
		}
		else if (detx == -1 * level1) {
			tmp(0) = _curLoc(0) + 1;
			tmp(1) = _curLoc(1);
			addLoc(tmp, _fromLocs);
			tmp(1) = _curLoc(1) - 1;
			addLoc(tmp, _fromLocs);
			tmp(1) = _curLoc(1) + 1;
			addLoc(tmp, _fromLocs);
		}
		else if (dety == level1) {
			tmp(0) = _curLoc(0);
			tmp(1) = _curLoc(1) - 1;
			addLoc(tmp, _fromLocs);
			tmp(0) = _curLoc(0) - 1;
			addLoc(tmp, _fromLocs);
			tmp(0) = _curLoc(0) + 1;
			addLoc(tmp, _fromLocs);
		}
		else if (dety == -1 * level1) {
			tmp(0) = _curLoc(0);
			tmp(1) = _curLoc(1) + 1;
			addLoc(tmp, _fromLocs);
			tmp(0) = _curLoc(0) - 1;
			addLoc(tmp, _fromLocs);
			tmp(0) = _curLoc(0) + 1;
			addLoc(tmp, _fromLocs);
		}
	}

}

void Diffuser2D::addLoc(const Eigen::Vector2i& _loc, std::vector<Eigen::Vector2i>& _locs)
{
	if (_loc(0) < 0 || _loc(0) >= m_w || _loc(1) < 0 || _loc(1) >= m_h || m_graph[_loc(1)][_loc(0)] != 1)
		return;
	_locs.push_back(_loc);
}


NSP_SLAM_LYJ_MATH_END