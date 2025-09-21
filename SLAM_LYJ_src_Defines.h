#ifndef SLAM_LYJ_SRC_DEFINES_H
#define SLAM_LYJ_SRC_DEFINES_H

#include <string>
#include <vector>

#include <IO/SimpleIO.h>
#include <base/CameraModule.h>
#include <common/CommonAlgorithm.h>

#include <ImageProcess_LYJ_Include.h>
#include <ImageCommon/CorrespondGraph.h>

#include "STLPlus/include/file_system.h"

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

    struct ProcessOption
    {
        //visual
        std::string imgDir = "";
        std::vector<std::string> imgNames;
        std::string camFile = "";
        std::string priTcwDir = "";
        std::vector<std::string> priTcwNames;
        std::string vocPath = "";
        bool writeVoc = true;
        bool readCache = false;
        ImageProcess_LYJ::ImageExtractOption imageExtractOpt;
        ImageProcess_LYJ::ImageMatchOption imageMatchOpt;

        //with mesh
        std::string meshPath = "";
    };


}

#endif//SLAM_LYJ_src_DEFINES_H