#ifndef SLAM_LYJ_SRC_MAP_FRAME_H
#define SLAM_LYJ_SRC_MAP_FRAME_H


#include "SLAM_LYJ_src_Defines.h"
#include "mapPoint.h"

namespace SLAM_LYJ_src
{
	class MapFrame
	{
	public:
		MapFrame(const int& _id)
			:id_(_id)
		{ }
		~MapFrame() {};

		inline const int& getId() const { return id_; }
		inline std::shared_ptr<ImageProcess_LYJ::ImageExtractData> getImageExtractData() { return imageExtractData_; }
		inline std::shared_ptr<MapPoint> getMapPoint(const int& _id) { return mapPoints_[_id]; }
		inline void setMapPoint(const int& _id, std::shared_ptr<MapPoint> _mapPoint) { mapPoints_[_id] = _mapPoint; }

	private:
		int id_ = -1;
		std::shared_ptr<ImageProcess_LYJ::ImageExtractData> imageExtractData_ = nullptr;
		std::vector<std::shared_ptr<MapPoint>> mapPoints_;
	};
}



#endif // !SLAM_LYJ_SRC_MAP_FRAME_H
