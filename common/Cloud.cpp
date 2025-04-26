#include "Cloud.h"

NSP_SLAM_LYJ_MATH_BEGIN

Cloud::Cloud()
{}
Cloud::~Cloud()
{}

void Cloud::addVertex(const Eigen::Vector3f& _vertex)
{
	m_vertexs.push_back(_vertex);
	if (m_enableVNr)
		m_vNormals.resize(m_vertexs.size());
}

void Cloud::enableVNormals()
{
	m_enableVNr = true;
	m_vNormals.assign(m_vertexs.size(), Eigen::Vector3f(0,0,0));
}

void Cloud::disableVNormals()
{
	m_enableVNr = false;
	m_vNormals.swap(std::vector<Eigen::Vector3f>());
}

bool Cloud::setVNormals(const std::vector<Eigen::Vector3f>& _vNormals)
{
	if (!m_enableVNr)
		return false;
	m_vNormals = _vNormals;
	return true;
}

bool Cloud::setVNormal(uint32_t _id, const Eigen::Vector3f& _vNormal)
{
	if (!m_enableVNr)
		return false;
	m_vNormals[_id] = _vNormal;
	return true;
}




NSP_SLAM_LYJ_MATH_END