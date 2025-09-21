#include "map.h"



namespace SLAM_LYJ_src
{
    void Map::setMapFrames(std::vector<std::shared_ptr<MapFrame>>& _mapFrames)
    {
        for (int i = 0; i < _mapFrames.size(); ++i)
            mapFrames_[_mapFrames[i]->getId()] = _mapFrames[i];
    }
    void Map::setMapPoints(std::vector<std::shared_ptr<MapPoint>>& _mapPoints)
    {
        for (int i = 0; i < _mapPoints.size(); ++i)
            mapPoints_[_mapPoints[i]->getId()] = mapPoints_[i];
    }
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