#ifndef SLAM_LYJ_SRC_MAP_POINT_H
#define SLAM_LYJ_SRC_MAP_POINT_H

#include "SLAM_LYJ_src_Defines.h"


namespace SLAM_LYJ_src
{
	//struct MapPointObserve
	//{
	//	MapPointObserve() {}
	//	MapPointObserve(const int& _frameId, const int& _pointId)
	//		:frameId(_frameId), pointId(_pointId)
	//	{}
	//	int frameId = -1;
	//	int pointId = -1;
	//};
	class MapPoint
	{
	public:
		MapPoint(int _id) 
			:id_(_id)
		{};
		~MapPoint() {};

		inline const int& getId() const { return id_; }

		inline Eigen::Vector3d& getPw() { return Pw_; }
		inline const Eigen::Vector3d& getPw() const { return Pw_; }

		inline std::map<int, int>& getObs() { return obs_; }
		inline const std::map<int, int>& getObs() const { return obs_; }
		inline void addOb(const int& _frameId, const int& _pointId) { obs_[_frameId] = _pointId; }
		inline void removeOb(const int& _frameId, const int& _pointId) { obs_.erase(_frameId); }
		inline bool existOb(const int& _frameId)const{	return obs_.count(_frameId) > 0;}

	private:
		int id_ = -1;
		Eigen::Vector3d Pw_;
		std::map<int, int> obs_;
	};
}



#endif // !SLAM_LYJ_SRC_MAP_POINT_H
