#ifndef SLAM_LYJ_SRC_DEFINES_H
#define SLAM_LYJ_SRC_DEFINES_H

#include <string>
#include <vector>
#include <ImageProcess_LYJ_Include.h>
#include <IO/SimpleIO.h>
#include <base/CameraModule.h>
#include <ImageCommon/CorrespondGraph.h>

#define NSP_SLAM_LYJ_SRC_BEGIN \
    namespace SLAM_LYJ_src     \
    {
#define NSP_SLAM_LYJ_SRC_END }


#ifdef WIN32
#ifdef _MSC_VER
#define SLAM_LYJ_SRC_API __declspec(dllexport)
#else
#define SLAM_LYJ_SRC_API
#endif
#else
#define SLAM_LYJ_SRC_API
#endif



namespace SLAM_LYJ_src
{

    struct ProVPOpt
    {
        std::string imgDir = "";
        std::vector<std::string> imgNames;
        std::string camFile = "";
        std::string priTcwDir = "";
        std::vector<std::string> priTcwNames;
    };


}

#endif//SLAM_LYJ_src_DEFINES_H