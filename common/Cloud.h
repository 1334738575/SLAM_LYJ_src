#ifndef SLAM_LYJ_CLOUD_H
#define SLAM_LYJ_CLOUD_H

#include "base/Base.h"

NSP_SLAM_LYJ_MATH_BEGIN


class Cloud
{
public:
	Cloud();
	~Cloud();

	void setVertexs(const std::vector<Eigen::Vector3f>& _vertexs);
	std::vector<Eigen::Vector3f>& getVertexs();
	const std::vector<Eigen::Vector3f>& getVertexs() const;
	void enableVNormals();
	void disableVNormals();
	bool isEnableVNormals() const;
	std::vector<Eigen::Vector3f>& getVNormals();
	const std::vector<Eigen::Vector3f>& getVNormals() const;
	bool setVNormals(const std::vector<Eigen::Vector3f>& _vNormals);


private:
	std::vector<Eigen::Vector3f> m_vertexs;
	bool m_enableVNr = false;
	std::vector<Eigen::Vector3f> m_vNormals;
};






NSP_SLAM_LYJ_MATH_END

#endif // !SLAM_LYJ_CLOUD_H
