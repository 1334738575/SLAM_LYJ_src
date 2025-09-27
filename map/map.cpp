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
    void Map::updateMapPointsDescriptors()
    {
        for (auto& mapPoint : mapPoints_)
            updateMapPointsDescriptor(mapPoint.first);
    }
    void Map::updateMapPointsDescriptor(const int& _id)
    {
        std::vector<cv::Mat> descs;
        const auto& obs = mapPoints_[_id]->getObs();
        for (const auto& ob : obs)
        {
			auto mapFrame = getMapFrame(ob.first);
			if (mapFrame)
			{
				auto imgExData = mapFrame->getImageExtractData();
				if (ob.second >= 0 && ob.second < imgExData->descriptors_.rows)
					descs.push_back(imgExData->descriptors_.row(ob.second));
			}
        }
    }
}