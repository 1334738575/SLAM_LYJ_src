

#ifndef SLAM_LYJ_PROCESSORVP_H
#define SLAM_LYJ_PROCESSORVP_H

#include "processor/processorAbr.h"
#include <DBoW3/Vocabulary.h>
#include <DBoW3/Database.h>
#include "map/map.h"

NSP_SLAM_LYJ_SRC_BEGIN


class ProcessorVP : public ProcessorAbr
{
protected:
    /* data */
    ProcessOption opt_;
    SLAM_LYJ::PinholeCamera cam_;
    std::vector<std::shared_ptr<ImageProcess_LYJ::ImageExtractData>> imageExtractDatasPtr_;//与imageExtractDatas_相同
    std::map<uint32_t, std::shared_ptr<ImageProcess_LYJ::ImageExtractData>> imageExtractDatas_;
    std::unordered_map<uint64_t, std::shared_ptr<ImageProcess_LYJ::ImageMatchData>> imageMatchDatas_;
    std::shared_ptr<ImageProcess_LYJ::CorrespondGraph> correspondGraph_;
    std::shared_ptr<DBoW3::Vocabulary> voc_ = nullptr;
    std::shared_ptr<DBoW3::Database> vocDB_ = nullptr;
    std::shared_ptr<Map> map_ = nullptr;
public:
    ProcessorVP(/* args */);
    ~ProcessorVP();

    virtual void setData(const ProcessOption& _opt);
    int searchByProjection(std::shared_ptr<MapFrame> _mapFrame1, std::shared_ptr<MapFrame> _mapFrame2, float _th);

    // 通过 processorAbr 继承
    bool extractFeature() override;
    bool matchFeature() override;
    bool generateCorrespondGraph(bool _bCompress = true) override;
    bool generateMap() override;
    bool incrementalMap() override;
    bool optimize() override;

    bool writeData(const std::string& _path) override;
    bool readData(const std::string& _path) override;

protected:
    bool trianleMapPoint();

};






NSP_SLAM_LYJ_SRC_END


#endif //SLAM_LYJ_PROCESSORVP_H
