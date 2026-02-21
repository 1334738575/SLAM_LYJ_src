#ifndef SLAM_LYJ_PROCESSORABR_H
#define SLAM_LYJ_PROCESSORABR_H

#include "SLAM_LYJ_src_Defines.h"
#include "base/Base.h"

NSP_SLAM_LYJ_SRC_BEGIN



class ProcessorAbr
{
protected:
    /* data */
public:
    ProcessorAbr(/* args */){};
    ~ProcessorAbr(){};


    virtual bool extractFeature() = 0;
    virtual bool matchFeature() = 0;
    virtual bool generateCorrespondGraph(bool _bCompress=true) = 0;
    virtual bool generateMap() = 0;
    virtual bool incrementalMap() = 0;
    virtual bool optimize() = 0;

    virtual bool writeData(const std::string& _path) = 0;
    virtual bool readData(const std::string& _path) = 0;
};



NSP_SLAM_LYJ_SRC_END

#endif //SLAM_LYJ_PROCESSORABR_H