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
	void setVertex(uint32_t _id, const Eigen::Vector3f& _vertex);
	void addVertex(const Eigen::Vector3f& _vertex);
	std::vector<Eigen::Vector3f>& getVertexs();
	const std::vector<Eigen::Vector3f>& getVertexs() const;
	const Eigen::Vector3f& getVertex(uint32_t _id) const;
	void enableVNormals();
	void disableVNormals();
	bool isEnableVNormals() const;
	std::vector<Eigen::Vector3f>& getVNormals();
	const std::vector<Eigen::Vector3f>& getVNormals() const;
	const Eigen::Vector3f& getVNormal(uint32_t _id) const;
	bool setVNormals(const std::vector<Eigen::Vector3f>& _vNormals);
	bool setVNormal(uint32_t _id, const Eigen::Vector3f& _vNormal);


protected:
	std::vector<Eigen::Vector3f> m_vertexs;
	bool m_enableVNr = false;
	std::vector<Eigen::Vector3f> m_vNormals;

};


inline void Cloud::setVertexs(const std::vector<Eigen::Vector3f>& _vertexs)
{
	m_vertexs = _vertexs;
}

inline void Cloud::setVertex(uint32_t _id, const Eigen::Vector3f& _vertex)
{
	m_vertexs[_id] = _vertex;
}

inline std::vector<Eigen::Vector3f>& Cloud::getVertexs()
{
	// TODO: 在此处插入 return 语句
	return m_vertexs;
}

inline const std::vector<Eigen::Vector3f>& Cloud::getVertexs() const
{
	// TODO: 在此处插入 return 语句
	return m_vertexs;
}

inline const Eigen::Vector3f& Cloud::getVertex(uint32_t _id) const
{
	// TODO: 在此处插入 return 语句
	return m_vertexs[_id];
}

inline bool Cloud::isEnableVNormals() const
{
	return m_enableVNr;
}

inline std::vector<Eigen::Vector3f>& Cloud::getVNormals()
{
	// TODO: 在此处插入 return 语句
	return m_vNormals;
}

inline const std::vector<Eigen::Vector3f>& Cloud::getVNormals() const
{
	// TODO: 在此处插入 return 语句
	return m_vNormals;
}

inline const Eigen::Vector3f& Cloud::getVNormal(uint32_t _id) const
{
	// TODO: 在此处插入 return 语句
	return m_vNormals[_id];
}





NSP_SLAM_LYJ_MATH_END

#endif // !SLAM_LYJ_CLOUD_H
