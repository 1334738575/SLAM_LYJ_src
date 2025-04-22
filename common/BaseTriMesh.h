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

private:
	std::vector<BaseTriFace> m_faces;
	bool m_enableFNr = false;
	std::vector<Eigen::Vector3f> m_fNormals;
	std::vector<Eigen::Vector3f> m_centers;
};




NSP_SLAM_LYJ_MATH_END

#endif // !SLAM_LYJ_BASETRIMESH_H
