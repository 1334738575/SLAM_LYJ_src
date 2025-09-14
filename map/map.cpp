#include "map.h"



namespace SLAM_LYJ_src
{
    std::shared_ptr<MapFrame> Map::getMapFrame(const int& _id)
    {
        if (mapFrames_.count(_id))
            return mapFrames_[_id];
        return nullptr;
    }
    std::shared_ptr<MapPoint> Map::getMapPoint(const int& _id)
    {
        if (mapPoints_.count(_id))
            return mapPoints_[_id];
        return nullptr;
    }
}