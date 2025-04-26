#include "BaseTriMesh.h"

NSP_SLAM_LYJ_MATH_BEGIN


BaseTriMesh::BaseTriMesh()
{}
BaseTriMesh::~BaseTriMesh()
{}

std::vector<Eigen::Vector3f>& BaseTriMesh::getFNormals()
{
	// TODO: 在此处插入 return 语句
	if (!m_enableFNr)
		std::cout << "face normal is disable." << std::endl;
	return m_fNormals;
}

const std::vector<Eigen::Vector3f>& BaseTriMesh::getFNormals() const
{
	// TODO: 在此处插入 return 语句
	if (!m_enableFNr)
		std::cout << "face normal is disable." << std::endl;
	return m_fNormals;
}

bool BaseTriMesh::setFNormals(const std::vector<Eigen::Vector3f>& _fNormals)
{
	if (!m_enableFNr) {
		std::cout << "face normal is disable." << std::endl;
		return false;
	}
	m_fNormals = _fNormals;
	return true;
}

bool BaseTriMesh::setFNormal(uint32_t _id, const Eigen::Vector3f& _fNormal)
{
	if (!m_enableFNr) {
		std::cout << "face normal is disable." << std::endl;
		return false;
	}
	m_fNormals[_id] = _fNormal;
	return true;
}

void BaseTriMesh::calculateFNormals()
{
	if (!m_enableFNr)
		return;
	uint32_t fSize = m_faces.size();
	m_fNormals.resize(fSize);
	Eigen::Vector3f e1, e2;
	for (size_t i = 0; i < fSize; ++i)
	{
		e1 = m_vertexs[m_faces[i].vId_[1]] - m_vertexs[m_faces[i].vId_[0]];
		e2 = m_vertexs[m_faces[i].vId_[2]] - m_vertexs[m_faces[i].vId_[0]];
		m_fNormals[i] = e1.cross(e2);
		m_fNormals[i].normalize();
	}
}

void BaseTriMesh::setFCenters(const std::vector<Eigen::Vector3f>& _fCenters)
{
	if (!m_hasFCtr) {
		std::cout << "face center is disable." << std::endl;
		return;
	}
	m_centers = _fCenters;
}

void BaseTriMesh::setFCenter(uint32_t _id, const Eigen::Vector3f& _fCenter)
{
	if (!m_hasFCtr) {
		std::cout << "face center is disable." << std::endl;
		return;
	}
	m_centers[_id] = _fCenter;
}

std::vector<Eigen::Vector3f>& BaseTriMesh::getFCenters()
{
	// TODO: 在此处插入 return 语句
	if (!m_hasFCtr) {
		std::cout << "face center is disable." << std::endl;
	}
	return m_centers;
}

const std::vector<Eigen::Vector3f>& BaseTriMesh::getFCenters() const
{
	// TODO: 在此处插入 return 语句
	if (!m_hasFCtr) {
		std::cout << "face center is disable." << std::endl;
	}
	return m_centers;
}

const Eigen::Vector3f& BaseTriMesh::getFCenter(uint32_t _id) const
{
	// TODO: 在此处插入 return 语句
	if (!m_hasFCtr)
		std::cout << "face center is disable." << std::endl;
	return m_centers[_id];
}

void BaseTriMesh::calculateFCenters()
{
	if (!m_hasFCtr) {
		std::cout << "face center is disable." << std::endl;
		return;
	}
	uint32_t fSize = m_faces.size();
	m_centers.resize(fSize);
	for (int i = 0; i < fSize; ++i) {
		m_centers[i].setZero();
		for (int j = 0; j < 3; ++j) {
			m_centers[i] += m_vertexs[m_faces[i].vId_[j]];
		}
		m_centers[i] /= 3;
	}
	return;
}

bool BaseTriMesh::calculateVNormals()
{
	if (!m_enableFNr)
		return false;
	uint32_t vSize = m_vertexs.size();
	uint32_t fSize = m_faces.size();
	m_vNormals.resize(vSize);
	memset(m_vNormals[0].data(), 0, vSize * 3 * sizeof(float));
	std::vector<int> cnts(vSize, 0);
	for (int i = 0; i < fSize; ++i) {
		for (int j = 0; j < 3; ++j) {
			m_vNormals[m_faces[i].vId_[j]] += m_fNormals[i];
			++cnts[m_faces[i].vId_[j]];
		}
	}
	for (size_t i = 0; i < vSize; ++i)
		if (cnts[i]) {
			m_vNormals[i] /= cnts[i];
			m_vNormals[i].normalize();
		}
	return true;
}





NSP_SLAM_LYJ_MATH_END