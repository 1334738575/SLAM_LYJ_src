

#ifndef SLAM_LYJ_PROCESSORVP_H
#define SLAM_LYJ_PROCESSORVP_H

#include "processor/processorAbr.h"

NSP_SLAM_LYJ_SRC_BEGIN


class ProcessorVP : public ProcessorAbr
{
private:
    /* data */
    ProVPOpt opt_;
    SLAM_LYJ::PinholeCamera cam_;
    std::map<uint32_t, std::shared_ptr<ImageProcess_LYJ::ImageExtractData>> imageExtractDatas_;
    std::unordered_map<uint64_t, std::shared_ptr<ImageProcess_LYJ::ImageMatchData>> imageMatchDatas_;
    std::shared_ptr<ImageProcess_LYJ::CorrespondGraph> correspondGraph_;
public:
    ProcessorVP(/* args */);
    ~ProcessorVP();

    void setData(const ProVPOpt& _opt);


    // Í¨¹ý processorAbr ¼Ì³Ð
    bool extractFeature() override;
    bool matchFeature() override;
    bool generateCorrespondGraph() override;

    bool writeData(const std::string& _path) override;
    bool readData(const std::string& _path) override;

};






NSP_SLAM_LYJ_SRC_END


#endif //SLAM_LYJ_PROCESSORVP_H
