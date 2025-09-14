#ifndef SLAM_LYJ_SRC_MAP_H
#define SLAM_LYJ_SRC_MAP_H


#include "mapPoint.h"
#include "mapFrame.h"


namespace SLAM_LYJ_src
{
	class Map
	{
	public:
		Map(const int& _id) 
			:id_(_id)
		{};
		~Map() {};

		inline const int& getId() const { return id_; }
		std::shared_ptr<MapFrame> getMapFrame(const int& _id);
		std::shared_ptr<MapPoint> getMapPoint(const int& _id);

	private:
		int id_ = -1;
		std::unordered_map<int, std::shared_ptr<MapFrame>> mapFrames_;
		std::unordered_map<int, std::shared_ptr<MapPoint>> mapPoints_;

	};

}




#endif // !SLAM_LYJ_SRC_MAP_H
