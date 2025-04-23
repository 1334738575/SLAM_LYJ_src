#ifndef SLAM_LYJ_BASETRIMESH_H
#define SLAM_LYJ_BASETRIMESH_H

#include "Cloud.h"

NSP_SLAM_LYJ_MATH_BEGIN


struct BaseTriFace
{
	BaseTriFace(uint32_t _v1, uint32_t _v2, uint32_t _v3)
	{
		vId_[0] = _v1;
		vId_[1] = _v2;
		vId_[2] = _v3;
	}
	uint32_t vId_[3];
};

class BaseTriMesh : public Cloud
{
public:
	BaseTriMesh();
	~BaseTriMesh();

	void setFaces(const std::vector<BaseTriFace>& _faces);
	void setFace(uint32_t _id, const BaseTriFace& _face);
	void addFace(const BaseTriFace& _face);
	std::vector<BaseTriFace>& getFaces();
	const std::vector<BaseTriFace>& getFaces() const;
	const BaseTriFace& getFace(uint32_t _id);
	void enableFNormals();
	void disableFNormals();
	bool isEnableFNormals() const;
	std::vector<Eigen::Vector3f>& getFNormals();
	const std::vector<Eigen::Vector3f>& getFNormals() const;
	bool setFNormals(const std::vector<Eigen::Vector3f>& _fNormals);
	void calculateFNormals();
	bool hasFCenters() const;
	void setFCenters(const std::vector<Eigen::Vector3f>& _fCenters);
	std::vector<Eigen::Vector3f>& getFCenters();
	const std::vector<Eigen::Vector3f>& getFCenters() const;
	const Eigen::Vector3f& getFCenter(uint32_t _id) const;
	void calculateFCenters();
	bool calculateVNormals();

private:
	std::vector<BaseTriFace> m_faces;
	bool m_enableFNr = false;
	std::vector<Eigen::Vector3f> m_fNormals;
	bool m_hasFCtr = false;
	std::vector<Eigen::Vector3f> m_centers;
};


inline void BaseTriMesh::setFaces(const std::vector<BaseTriFace>& _faces)
{
	m_faces = _faces;
}

inline void BaseTriMesh::setFace(uint32_t _id, const BaseTriFace& _face)
{
	m_faces[_id] = _face;
}

inline void BaseTriMesh::addFace(const BaseTriFace& _face)
{
	m_faces.push_back(_face);
	if (m_enableFNr) {

	}
}



NSP_SLAM_LYJ_MATH_END

#endif // !SLAM_LYJ_BASETRIMESH_H
